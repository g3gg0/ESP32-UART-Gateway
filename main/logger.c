#include "sdkconfig.h"

#ifdef CONFIG_LOGGER_ENABLED

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <unistd.h>

#include "esp_vfs_fat.h"
#include "wear_levelling.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "logger.h"
#include "uart_gateway.h"

/* VFS mount point - must start with '/' */
#define LOGGER_MOUNT_POINT  "/logger"

/* Maximum characters in a full log file path */
#define LOGGER_MAX_PATH_LEN 64

/*
 * A single item posted to the logger queue.
 * 'data' points to a heap-allocated copy of received bytes; the logger task
 * is responsible for freeing it after writing.
 */
typedef struct {
    uint8_t *data;
    size_t   len;
} logger_chunk_t;

static wl_handle_t   s_wl_handle          = WL_INVALID_HANDLE;
static QueueHandle_t s_queue              = NULL;
static TaskHandle_t  s_task_handle        = NULL;
static volatile bool s_task_running       = false;
static volatile bool s_logging_stopped    = false;
static FILE         *s_log_file           = NULL;
static uint32_t      s_file_counter       = 0;
static size_t        s_current_file_bytes = 0;
static char          s_log_path[LOGGER_MAX_PATH_LEN];

/* ------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------ */

/*
 * Scan the mount point directory and return the highest numeric counter
 * found in filenames matching <prefix><digits>.bin.
 * Returns 0 when no matching files exist.
 */
static uint32_t find_highest_counter(void)
{
    uint32_t      highest    = 0;
    size_t        prefix_len = strlen(CONFIG_LOGGER_FILENAME_PREFIX);
    DIR          *dir;
    struct dirent *entry;

    dir = opendir(LOGGER_MOUNT_POINT);
    if (dir == NULL)
    {
        return 0;
    }

    while ((entry = readdir(dir)) != NULL)
    {
        /* Skip entries that do not start with the configured prefix */
        if (strncmp(entry->d_name, CONFIG_LOGGER_FILENAME_PREFIX, prefix_len) != 0)
        {
            continue;
        }

        /* The counter is the digit run immediately after the prefix */
        const char *num_start = entry->d_name + prefix_len;
        char       *endptr    = NULL;
        unsigned long val     = strtoul(num_start, &endptr, 10);

        /* Accept only entries whose suffix is exactly ".bin" */
        if (endptr != NULL && strcmp(endptr, ".bin") == 0)
        {
            if ((uint32_t)val > highest)
            {
                highest = (uint32_t)val;
            }
        }
    }

    closedir(dir);
    return highest;
}

/*
 * Store the full VFS path for the given counter value into s_log_path.
 */
static void build_log_path(uint32_t counter)
{
    snprintf(s_log_path, sizeof(s_log_path),
             "%s/%s%06lu.bin",
             LOGGER_MOUNT_POINT,
             CONFIG_LOGGER_FILENAME_PREFIX,
             (unsigned long)counter);
}

/*
 * Flush and close the currently open log file (if any), increment the
 * counter and open the next file for writing.
 */
static esp_err_t open_next_log_file(void)
{
    if (s_log_file != NULL)
    {
        fflush(s_log_file);
        fsync(fileno(s_log_file));
        fclose(s_log_file);
        s_log_file = NULL;
    }

    s_file_counter++;
    s_current_file_bytes = 0;
    build_log_path(s_file_counter);

    s_log_file = fopen(s_log_path, "wb");
    if (s_log_file == NULL)
    {
        send_message("LOGGER ERR: open failed: %s", s_log_path);
        return ESP_FAIL;
    }

    send_message("LOGGER: opened %s", s_log_path);
    return ESP_OK;
}

/* ------------------------------------------------------------------
 * Space management helpers
 * ------------------------------------------------------------------ */

static uint64_t get_free_bytes(void)
{
    uint64_t total  = 0;
    uint64_t free_b = 0;
    esp_vfs_fat_info(LOGGER_MOUNT_POINT, &total, &free_b);
    return free_b;
}

/*
 * Return the lowest counter among all log files except the currently-open
 * one.  Returns UINT32_MAX when no older file exists.
 */
static uint32_t find_oldest_counter(void)
{
    uint32_t      oldest     = UINT32_MAX;
    size_t        prefix_len = strlen(CONFIG_LOGGER_FILENAME_PREFIX);
    DIR          *dir        = opendir(LOGGER_MOUNT_POINT);
    struct dirent *entry;

    if (dir == NULL)
    {
        return UINT32_MAX;
    }

    while ((entry = readdir(dir)) != NULL)
    {
        if (strncmp(entry->d_name, CONFIG_LOGGER_FILENAME_PREFIX, prefix_len) != 0)
        {
            continue;
        }

        char       *endptr = NULL;
        unsigned long val  = strtoul(entry->d_name + prefix_len, &endptr, 10);

        if (endptr == NULL || strcmp(endptr, ".bin") != 0)
        {
            continue;
        }

        if ((uint32_t)val == s_file_counter) /* skip currently-open file */
        {
            continue;
        }

        if ((uint32_t)val < oldest)
        {
            oldest = (uint32_t)val;
        }
    }

    closedir(dir);
    return oldest;
}

/*
 * Close and delete every log file, then reset the counter to 0 so the next
 * open_next_log_file() call creates file 000001 again.
 */
static void erase_all_log_files(void)
{
    if (s_log_file != NULL)
    {
        fclose(s_log_file);
        s_log_file = NULL;
    }

    size_t        prefix_len = strlen(CONFIG_LOGGER_FILENAME_PREFIX);
    DIR          *dir        = opendir(LOGGER_MOUNT_POINT);
    struct dirent *entry;

    if (dir == NULL)
    {
        return;
    }

    while ((entry = readdir(dir)) != NULL)
    {
        if (strncmp(entry->d_name, CONFIG_LOGGER_FILENAME_PREFIX, prefix_len) != 0)
        {
            continue;
        }

        char path[LOGGER_MAX_PATH_LEN];
        snprintf(path, sizeof(path), "%s/%s", LOGGER_MOUNT_POINT, entry->d_name);
        remove(path);
    }

    closedir(dir);
    s_file_counter = 0; /* open_next_log_file() will start at 000001 */
}

static void apply_space_policy(void)
{
    if (s_logging_stopped)
    {
        return;
    }

    uint64_t free_b    = get_free_bytes();
    uint64_t threshold = (uint64_t)CONFIG_LOGGER_SPACE_THRESHOLD_KB * 1024u;

    if (free_b >= threshold)
    {
        return;
    }

#if defined(CONFIG_LOGGER_POLICY_STOP)

    s_logging_stopped = true;
    if (s_log_file != NULL)
    {
        fflush(s_log_file);
        fsync(fileno(s_log_file));
        fclose(s_log_file);
        s_log_file = NULL;
    }
    send_message("LOGGER: storage low (%u B), stopped", (unsigned int)free_b);

#elif defined(CONFIG_LOGGER_POLICY_ROTATE)

    uint32_t oldest = find_oldest_counter();
    if (oldest == UINT32_MAX)
    {
        /* Only the current file exists - cannot reclaim space */
        send_message("LOGGER: low space, no older file (%u B)", (unsigned int)free_b);
        return;
    }

    char path[LOGGER_MAX_PATH_LEN];
    snprintf(path, sizeof(path), "%s/%s%06lu.bin",
             LOGGER_MOUNT_POINT, CONFIG_LOGGER_FILENAME_PREFIX, (unsigned long)oldest);
    remove(path);
    send_message("LOGGER: deleted %s (%u B free)", path, (unsigned int)free_b);

#elif defined(CONFIG_LOGGER_POLICY_ERASE)

    send_message("LOGGER: low space (%u B), erasing all", (unsigned int)free_b);
    erase_all_log_files();
    open_next_log_file();

#endif
}

/* ------------------------------------------------------------------
 * Logger task
 * ------------------------------------------------------------------ */

static void logger_task(void *pvParameters)
{
    logger_chunk_t chunk;
    TickType_t     last_flush        = xTaskGetTickCount();
    size_t         bytes_since_flush = 0;

    s_task_running = true;

    while (s_task_running)
    {
        /*
         * Compute how long to block on the queue: wake up at least every
         * LOGGER_FLUSH_INTERVAL_MS milliseconds for the periodic flush.
         */
        TickType_t flush_ticks = pdMS_TO_TICKS(CONFIG_LOGGER_FLUSH_INTERVAL_MS);
        TickType_t now         = xTaskGetTickCount();
        TickType_t elapsed     = now - last_flush;
        TickType_t wait        = (elapsed >= flush_ticks) ? 0 : (flush_ticks - elapsed);

        if (xQueueReceive(s_queue, &chunk, wait) == pdTRUE)
        {
            if (s_log_file != NULL && !s_logging_stopped &&
                chunk.data != NULL && chunk.len > 0)
            {
                size_t written        = fwrite(chunk.data, 1, chunk.len, s_log_file);
                bytes_since_flush    += written;
                s_current_file_bytes += written;

#if defined(CONFIG_LOGGER_POLICY_ROTATE)
                /*
                 * ROTATE mode: start a new file once the current one reaches
                 * the threshold size, forming fixed-size ring-buffer blocks.
                 */
                if (s_current_file_bytes >=
                    (size_t)CONFIG_LOGGER_SPACE_THRESHOLD_KB * 1024u)
                {
                    fflush(s_log_file);
                    fsync(fileno(s_log_file));
                    open_next_log_file();
                    bytes_since_flush = 0;
                }
#elif CONFIG_LOGGER_MAX_FILE_SIZE_KB > 0
                /* Legacy per-file size cap (inactive when ROTATE policy is used) */
                if (s_current_file_bytes >=
                    (size_t)(CONFIG_LOGGER_MAX_FILE_SIZE_KB) * 1024u)
                {
                    fflush(s_log_file);
                    fsync(fileno(s_log_file));
                    open_next_log_file();
                    bytes_since_flush = 0;
                }
#endif
            }

            free(chunk.data);
        }

        /* Periodic flush and space-policy check */
        now = xTaskGetTickCount();
        if ((now - last_flush) >= flush_ticks)
        {
            if (s_log_file != NULL && !s_logging_stopped && bytes_since_flush > 0)
            {
                fflush(s_log_file);
                fsync(fileno(s_log_file));
                bytes_since_flush = 0;
            }
            apply_space_policy();
            last_flush = now;
        }
    }

    /* Drain any remaining items before exiting */
    while (xQueueReceive(s_queue, &chunk, 0) == pdTRUE)
    {
        if (s_log_file != NULL && chunk.data != NULL && chunk.len > 0)
        {
            fwrite(chunk.data, 1, chunk.len, s_log_file);
        }
        free(chunk.data);
    }

    /* Final flush and close */
    if (s_log_file != NULL)
    {
        fflush(s_log_file);
        fsync(fileno(s_log_file));
        fclose(s_log_file);
        s_log_file = NULL;
    }

    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------ */

esp_err_t logger_init(void)
{
    esp_vfs_fat_mount_config_t mount_cfg = {
        .format_if_mount_failed = true,
        .max_files              = 4,
        .allocation_unit_size   = 512,
    };

    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(
        LOGGER_MOUNT_POINT,
        CONFIG_LOGGER_PARTITION_LABEL,
        &mount_cfg,
        &s_wl_handle);

    if (err != ESP_OK)
    {
        send_message("LOGGER ERR: FAT mount: %s", esp_err_to_name(err));
        return err;
    }

    uint32_t highest = find_highest_counter();
    s_file_counter   = highest;

    send_message("LOGGER: mounted, next file %lu", (unsigned long)(highest + 1u));
    return ESP_OK;
}

esp_err_t logger_start(void)
{
    if (s_wl_handle == WL_INVALID_HANDLE)
    {
        send_message("LOGGER ERR: not mounted");
        return ESP_ERR_INVALID_STATE;
    }

    /* Create the inter-task queue */
    s_queue = xQueueCreate(CONFIG_LOGGER_QUEUE_DEPTH, sizeof(logger_chunk_t));
    if (s_queue == NULL)
    {
        send_message("LOGGER ERR: queue alloc failed");
        return ESP_ERR_NO_MEM;
    }

    /* Open the first log file for this boot */
    esp_err_t err = open_next_log_file();
    if (err != ESP_OK)
    {
        vQueueDelete(s_queue);
        s_queue = NULL;
        return err;
    }

    /* Spawn the logger task at slightly lower priority than gateway tasks */
    BaseType_t ret = xTaskCreatePinnedToCore(
        logger_task,
        "logger",
        4096,
        NULL,
        4,
        &s_task_handle,
        0);

    if (ret != pdPASS)
    {
        send_message("LOGGER ERR: task create failed");
        fclose(s_log_file);
        s_log_file = NULL;
        vQueueDelete(s_queue);
        s_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void logger_stop(void)
{
    if (!s_task_running)
    {
        return;
    }

    /* Signal the task to exit after draining */
    s_task_running = false;

    /* Send a zero-length sentinel to unblock xQueueReceive immediately */
    if (s_queue != NULL)
    {
        logger_chunk_t sentinel = { NULL, 0 };
        xQueueSend(s_queue, &sentinel, 0);
    }

    /* Give the task time to finish the drain + flush cycle and call vTaskDelete */
    vTaskDelay(pdMS_TO_TICKS(CONFIG_LOGGER_FLUSH_INTERVAL_MS + 200u));
    s_task_handle = NULL;

    if (s_queue != NULL)
    {
        vQueueDelete(s_queue);
        s_queue = NULL;
    }

    if (s_wl_handle != WL_INVALID_HANDLE)
    {
        esp_vfs_fat_spiflash_unmount_rw_wl(LOGGER_MOUNT_POINT, s_wl_handle);
        s_wl_handle = WL_INVALID_HANDLE;
    }
}

void logger_enqueue_data(const uint8_t *data, size_t len)
{
    if (s_queue == NULL || data == NULL || len == 0)
    {
        return;
    }

    /* Allocate a private copy so the caller's buffer can be reused immediately */
    uint8_t *copy = (uint8_t *)malloc(len);
    if (copy == NULL)
    {
        return;
    }

    memcpy(copy, data, len);

    logger_chunk_t chunk = { copy, len };

    /*
     * Non-blocking enqueue: if the queue is full the chunk is dropped rather
     * than stalling the UART RX path.
     */
    if (xQueueSend(s_queue, &chunk, 0) != pdTRUE)
    {
        free(copy);
    }
}

#endif /* CONFIG_LOGGER_ENABLED */
