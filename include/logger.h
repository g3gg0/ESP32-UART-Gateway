#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "sdkconfig.h"

#ifdef CONFIG_LOGGER_ENABLED

/* VFS mount point for the logger FAT partition */
#define LOGGER_MOUNT_POINT "/logger"

/*
 * logger_init  - Mount the FAT filesystem, format if needed, scan for the
 *                highest existing log file counter.  Must be called once
 *                before logger_start().
 */
esp_err_t logger_init(void);

/*
 * logger_start - Create the inter-task queue, open the next log file and
 *                launch the logger FreeRTOS task.
 */
esp_err_t logger_start(void);

/*
 * logger_stop  - Signal the logger task to drain and flush, wait for it to
 *                exit, then unmount the filesystem.
 */
void logger_stop(void);

/*
 * logger_enqueue_data - Copy len bytes from data into a heap buffer and post
 *                       it to the logger queue.  Non-blocking: data is silently
 *                       dropped if the queue is full.  Safe to call from any
 *                       task context.
 */
void logger_enqueue_data(const uint8_t *data, size_t len);

#else /* CONFIG_LOGGER_ENABLED not set */

/* Inline no-op stubs so call sites compile without #ifdef guards */
static inline esp_err_t logger_init(void)                                { return ESP_OK; }
static inline esp_err_t logger_start(void)                               { return ESP_OK; }
static inline void      logger_stop(void)                                { }
static inline void      logger_enqueue_data(const uint8_t *d, size_t l) { (void)d; (void)l; }

#endif /* CONFIG_LOGGER_ENABLED */
