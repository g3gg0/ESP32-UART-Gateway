#include "uart_gateway.h"
#include "led.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#define TAG "UART_GATEWAY"
#define NVS_NAMESPACE "uart_config"
#define NVS_KEY_BAUD "baud_rate"
#define NVS_KEY_TX_GPIO "tx_gpio"
#define NVS_KEY_RX_GPIO "rx_gpio"
#define NVS_KEY_RESET_GPIO "reset_gpio"
#define NVS_KEY_CONTROL_GPIO "control_gpio"
#define NVS_KEY_LED_GPIO "led_gpio"
#define NVS_KEY_EXTENDED_MODE "ext_mode"

/* Magic bytes: type 0x000A packet with 8-byte payload (length=12 includes header, type=0x000A in little-endian) */
const uint8_t uart_extmode_magic[UART_EXTMODE_MAGIC_SIZE] = {
    0x0C, 0x00,             /* length: 12 (4 bytes header + 8 bytes payload) */
    0x0A, 0x00,             /* type: 0x000A */
    0x55, 0x41, 0x52, 0x54, /* U A R T */
    0x47, 0x57, 0x45, 0x58  /* G W E X */
};

/* CDC buffer for data flow */
#define CDC_BUFFER_SIZE 64
#define UART_BUFFER_SIZE 2048

static TaskHandle_t cdc_read_task_handle = NULL;
static TaskHandle_t cdc_write_task_handle = NULL;
static TaskHandle_t uart_read_task_handle = NULL;
static TaskHandle_t uart_write_task_handle = NULL;

/* Task buffers (global to save stack space) */
static uint8_t cdc_read_buffer[CDC_BUFFER_SIZE];
static uint8_t uart_write_buffer[UART_BUFFER_SIZE];
static uint8_t uart_read_buffer[UART_BUFFER_SIZE];

typedef struct
{
    uart_port_t uart_num;
    uartgw_config_t current_config;
    bool is_configured;
    bool is_initializing;
    uint8_t config_buffer[UART_CONFIG_PACKET_SIZE];
    size_t config_buffer_pos;
    StreamBufferHandle_t cdc_to_uart_buffer;
    QueueHandle_t uart_to_cdc_buffer;
} uart_gateway_ctx_t;

static uart_gateway_ctx_t gateway_ctx = {
    .uart_num = UART_NUM_0,
    .is_configured = false,
    .is_initializing = false,
    .config_buffer_pos = 0,
    .cdc_to_uart_buffer = NULL,
    .uart_to_cdc_buffer = NULL,
    .current_config = {
        .baud_rate = UART_DEFAULT_BAUD,
        .tx_gpio = UART_DEFAULT_TX_GPIO,
        .rx_gpio = UART_DEFAULT_RX_GPIO,
        .reset_gpio = UART_DEFAULT_RESET_GPIO,
        .control_gpio = UART_DEFAULT_CONTROL_GPIO,
        .led_gpio = UART_DEFAULT_LED_GPIO,
        .extended_mode = 0,
    },
};

static esp_err_t reconfigure_uart(const uartgw_config_t *config);

static void send_current_config(void)
{
    /* Only send config responses in extended mode */
    if (gateway_ctx.current_config.extended_mode == 0)
    {
        return;
    }

    /* Allocate complete packet */
    uart_packet_header_t *packet = (uart_packet_header_t *)malloc(UART_PACKET_HEADER_SIZE + UART_CONFIG_PACKET_SIZE);
    if (packet == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate config response packet");
        return;
    }

    /* Build header */
    packet->length = (uint16_t)(UART_PACKET_HEADER_SIZE + UART_CONFIG_PACKET_SIZE); /* length includes header and payload */
    packet->type = UART_PACKET_TYPE_CONFIG;

    /* Build config payload directly in malloc'd memory */
    uart_gateway_build_response_packet((uart_config_packet_t *)PTR_BEHIND(packet));

    /* Send to queue */
    esp_err_t err = queue_packet(packet);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Config response queued successfully (%zu bytes)", packet->length);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to queue config response: %s", esp_err_to_name(err));
    }
}

void send_message(const char *fmt, ...)
{
    if (fmt == NULL)
    {
        ESP_LOGE(TAG, "send_message: fmt is NULL");
        return;
    }

    /* In extended_mode == 0, log messages are disabled */
    if (gateway_ctx.current_config.extended_mode == 0)
    {
        return;
    }

    char message[UART_LOGMSG_MAX_LEN];
    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(message, sizeof(message), fmt, args);
    va_end(args);

    if (written < 0)
    {
        ESP_LOGE(TAG, "send_message: vsnprintf failed");
        return;
    }

    size_t msg_len = (written >= (int)sizeof(message)) ? (sizeof(message) - 1) : (size_t)written;

    /* In extended_mode == 1, use packet header with type 0x03 for logs */
    size_t packet_size = UART_PACKET_HEADER_SIZE + msg_len;
    uart_packet_header_t *packet = (uart_packet_header_t *)malloc(packet_size);
    if (packet == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate log packet");
        return;
    }

    packet->length = (uint16_t)(sizeof(uart_packet_header_t) + msg_len);
    packet->type = UART_PACKET_TYPE_LOG;

    if (msg_len > 0)
    {
        memcpy(PTR_BEHIND(packet), message, msg_len);
    }

    esp_err_t err = queue_packet(packet);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "send_message: failed to queue log packet: %s", esp_err_to_name(err));
    }
}

/* Parse configuration packet */
static bool parse_config_packet(const uint8_t *packet_data, size_t bytes_read, uartgw_config_t *config)
{
    if (bytes_read < UART_CONFIG_PACKET_SIZE)
    {
        send_message("Config packet: invalid length %zu (expected %u)", bytes_read, UART_CONFIG_PACKET_SIZE);
        return false;
    }

    /* Cast to struct for easy access */
    const uart_config_packet_t *pkt = (const uart_config_packet_t *)packet_data;

    /* Extract from struct (automatic little-endian) */
    uint32_t baud = pkt->baud_rate;
    uint8_t tx_gpio = pkt->tx_gpio;
    uint8_t rx_gpio = pkt->rx_gpio;
    uint8_t reset_gpio = pkt->reset_gpio;
    uint8_t control_gpio = pkt->control_gpio;
    uint8_t led_gpio = pkt->led_gpio;
    uint8_t extended_mode = pkt->extended_mode;

    /* Validate parameters (baud_rate == 0 is query mode, allowed) */
    if (baud != 0)
    {
        if (baud < 300 || baud > 1000000)
        {
            send_message("Invalid baud rate: %lu", baud);
            return false;
        }

        if (tx_gpio > 43 || rx_gpio > 43)
        {
            send_message("Invalid GPIO pins: TX=%u, RX=%u", tx_gpio, rx_gpio);
            return false;
        }

        if ((reset_gpio > 43 && reset_gpio != 0xFF) || (control_gpio > 43 && control_gpio != 0xFF) || (led_gpio > 43 && led_gpio != 0xFF))
        {
            send_message("Invalid GPIO pins: RESET=%u, CONTROL=%u, LED=%u", reset_gpio, control_gpio, led_gpio);
            return false;
        }

        if (tx_gpio == rx_gpio)
        {
            send_message("TX and RX pins cannot be the same");
            return false;
        }
    }

    config->baud_rate = baud;
    config->tx_gpio = tx_gpio;
    config->rx_gpio = rx_gpio;
    config->reset_gpio = reset_gpio;
    config->control_gpio = control_gpio;
    config->led_gpio = led_gpio;
    config->extended_mode = extended_mode;

    return true;
}

/* Parse control packet and execute command */
static bool parse_control_command(const char *cmd, size_t cmd_len)
{
    /* Ensure null termination for safety */
    char cmd_buffer[UART_CONTROL_CMD_SIZE + 1];
    size_t copy_len = (cmd_len > UART_CONTROL_CMD_SIZE) ? UART_CONTROL_CMD_SIZE : cmd_len;
    memcpy(cmd_buffer, cmd, copy_len);
    cmd_buffer[copy_len] = '\0';

    /* Parse R:x (Reset control) */
    if (cmd_buffer[0] == 'R' && cmd_buffer[1] == ':')
    {
        /* Check if reset GPIO is configured (not 0xFF) */
        if (gateway_ctx.current_config.reset_gpio == 0xFF)
        {
            send_message("Reset GPIO not configured (0xFF), ignoring R: command");
            return false;
        }

        /* Set reset line state */
        gpio_set_level(gateway_ctx.current_config.reset_gpio, (cmd_buffer[2] == '0') ? 0 : 1);
        return true;
    }
    /* Parse C:x (Control pin) */
    else if (cmd_buffer[0] == 'C' && cmd_buffer[1] == ':')
    {
        /* Check if control GPIO is configured (not 0xFF) */
        if (gateway_ctx.current_config.control_gpio == 0xFF)
        {
            send_message("Control GPIO not configured (0xFF), ignoring C: command");
            return false;
        }

        gpio_set_level(gateway_ctx.current_config.control_gpio, (cmd_buffer[2] == '0') ? 0 : 1);
        return true;
    }
    /* Parse B:xx (Break condition) */
    else if (cmd_buffer[0] == 'B' && cmd_buffer[1] == ':')
    {
        int brk_len = atoi(&cmd_buffer[2]);

        if (!gateway_ctx.is_configured)
        {
            send_message("UART not configured, cannot send break");
            return false;
        }

        gpio_set_level(gateway_ctx.current_config.tx_gpio, 1);
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << gateway_ctx.current_config.tx_gpio),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };

        gpio_config(&io_conf);
        gpio_set_level(gateway_ctx.current_config.tx_gpio, 0);

        if (brk_len < 20)
        {
            esp_rom_delay_us(brk_len * 1000);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(brk_len));
        }

        gpio_set_level(gateway_ctx.current_config.tx_gpio, 1);

        reconfigure_uart(&gateway_ctx.current_config);

        /* Don't send config response for break commands */
        return true;
    }
    else
    {
        send_message("Unknown control command: '%s'", cmd_buffer);
        return false;
    }
}

/* Initialize all unused GPIO pins to GND with strong drive strength */
static void init_unused_gpios_to_gnd(uint8_t tx_gpio, uint8_t rx_gpio, uint8_t reset_gpio, uint8_t control_gpio, uint8_t led_gpio)
{
    /* ESP32-C3 has GPIO 0-21 usable */
    /* GPIO 18, 19 are used by USB (D- and D+), NEVER configure these */
    /* GPIO 11-17 may be used by embedded flash on some modules */
    /* GPIO 22 is strapping pin for downloading, skip it */
    /* GPIO 23-25 are reserved, skip them */
    /* Only configure safe GPIOs: 0-10, 20, 21 */
    static const uint8_t gpio_pins[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 21};
    static const size_t num_pins = sizeof(gpio_pins) / sizeof(gpio_pins[0]);

    ESP_LOGI(TAG, "Configuring unused GPIO pins to GND (excluding TX=%u, RX=%u, RESET=%u, CONTROL=%u, LED=%u)", tx_gpio, rx_gpio, reset_gpio, control_gpio, led_gpio);

    for (size_t i = 0; i < num_pins; i++)
    {
        uint8_t pin = gpio_pins[i];

        /* Skip TX, RX, RESET, CONTROL, and LED pins */
        if (pin == tx_gpio || pin == rx_gpio || pin == reset_gpio || pin == control_gpio || pin == led_gpio)
        {
            ESP_LOGD(TAG, "Skipping GPIO %u (in use)", pin);
            continue;
        }

        /* Configure as output, pull low to GND */
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };

        esp_err_t err = gpio_config(&io_conf);
        if (err == ESP_OK)
        {
            /* Set to LOW (GND) */
            gpio_set_level(pin, 0);

            /* Set to strong drive (I/O pad drive capability = 3 = max 40mA) */
            gpio_set_drive_capability(pin, GPIO_DRIVE_CAP_3);
            ESP_LOGD(TAG, "GPIO %u -> GND (strong drive)", pin);
        }
        else
        {
            ESP_LOGW(TAG, "Failed to configure GPIO %u to GND: %s", pin, esp_err_to_name(err));
        }
    }

    ESP_LOGI(TAG, "Unused GPIO pins configured to GND (USB pins 18,19 preserved, flash pins 11-17 preserved)");
}

static esp_err_t reconfigure_gpio(const uartgw_config_t *config)
{
    if (config->led_gpio != 0xFF)
    {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << config->led_gpio),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };

        gpio_config(&io_conf);
    }

    if (config->control_gpio != 0xFF)
    {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << config->control_gpio),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };

        gpio_config(&io_conf);
    }

    if (config->reset_gpio != 0xFF)
    {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << config->reset_gpio),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };

        gpio_config(&io_conf);
    }

    return ESP_OK;
}

/* Reconfigure UART with new pins and baud rate */
static esp_err_t reconfigure_uart(const uartgw_config_t *config)
{
    /* Deinitialize current UART if configured */
    if (gateway_ctx.is_configured)
    {
        uart_driver_delete(gateway_ctx.uart_num);
        gateway_ctx.is_configured = false;
    }

    /* Configure UART parameters */
    uart_config_t uart_hw_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    int intr_alloc_flags = 0;
#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(gateway_ctx.uart_num, 8192, 8192, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(gateway_ctx.uart_num, &uart_hw_config));
    ESP_ERROR_CHECK(uart_set_pin(gateway_ctx.uart_num, config->tx_gpio, config->rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    gateway_ctx.current_config = *config;
    gateway_ctx.is_configured = true;

    ESP_LOGI(TAG, "UART reconfigured: baud=%lu, TX=%u, RX=%u", config->baud_rate, config->tx_gpio, config->rx_gpio);

    return ESP_OK;
}

void uart_gateway_init(const uartgw_config_t *config)
{
    if (gateway_ctx.is_initializing)
    {
        return;
    }

    gateway_ctx.is_initializing = true;

    ESP_LOGI(TAG, "Creating stream buffers...");

    /* Create stream buffers if not already created */
    if (gateway_ctx.cdc_to_uart_buffer == NULL)
    {
        gateway_ctx.cdc_to_uart_buffer = xStreamBufferCreate(STREAM_BUFFER_SIZE, 1);
        if (gateway_ctx.cdc_to_uart_buffer == NULL)
        {
            ESP_LOGE(TAG, "Failed to create CDC->UART stream buffer");
        }
        else
        {
            ESP_LOGI(TAG, "CDC->UART stream buffer created (%u bytes)", STREAM_BUFFER_SIZE);
        }
    }

    if (gateway_ctx.uart_to_cdc_buffer == NULL)
    {
        gateway_ctx.uart_to_cdc_buffer = xQueueCreate(32, sizeof(uart_packet_header_t *));
        if (gateway_ctx.uart_to_cdc_buffer == NULL)
        {
            ESP_LOGE(TAG, "Failed to create UART->CDC packet queue");
        }
        else
        {
            ESP_LOGI(TAG, "UART->CDC packet queue created (32 items)");
        }
    }

    ESP_LOGI(TAG, "Initializing GPIOs...");
    reconfigure_gpio(config);
    ESP_LOGI(TAG, "Initializing UART gateway...");
    reconfigure_uart(config);

    /* Initialize all unused GPIO pins to GND with strong drive */
    init_unused_gpios_to_gnd(config->tx_gpio, config->rx_gpio, config->reset_gpio, config->control_gpio, config->led_gpio);

    gateway_ctx.is_initializing = false;

    /* Send startup message to unblock the USB CDC read on host side */
    send_message("ESP32C3 UART Gateway initialized - waiting for magic bytes");
}

esp_err_t uart_gateway_configure(const uartgw_config_t *config)
{
    if (config == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (gateway_ctx.is_initializing)
    {
        return ESP_ERR_INVALID_STATE;
    }

    reconfigure_gpio(config);
    reconfigure_uart(config);

    /* Initialize all unused GPIO pins to GND with strong drive */
    init_unused_gpios_to_gnd(config->tx_gpio, config->rx_gpio, config->reset_gpio, config->control_gpio, config->led_gpio);

    return ESP_OK;
}

void uart_gateway_get_config(uartgw_config_t *config)
{
    if (config != NULL)
    {
        *config = gateway_ctx.current_config;
    }
}

void uart_gateway_process_data(const uint8_t *data, size_t length)
{
    if (data == NULL || length == 0)
    {
        return;
    }

    /* Expecting exactly UART_CONFIG_PACKET_SIZE bytes as a single block */
    if (length != UART_CONFIG_PACKET_SIZE)
    {
        ESP_LOGW(TAG, "Config packet: invalid length %zu (expected %u)", length, UART_CONFIG_PACKET_SIZE);
        return;
    }
    else
    {
        ESP_LOGW(TAG, "✗ Config packet parse failed");
    }
}

esp_err_t uart_gateway_send(const uint8_t *data, size_t length)
{
    if (!gateway_ctx.is_configured)
    {
        ESP_LOGW(TAG, "UART send: UART not configured");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || length == 0)
    {
        ESP_LOGW(TAG, "UART send: invalid args");
        return ESP_ERR_INVALID_ARG;
    }

    int bytes_written = uart_write_bytes(gateway_ctx.uart_num, data, length);
    if (bytes_written < 0)
    {
        ESP_LOGE(TAG, "UART send: write failed, returned %d", bytes_written);
        return ESP_FAIL;
    }

    if (bytes_written != length)
    {
        ESP_LOGW(TAG, "UART send: partial write %d/%zu", bytes_written, length);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "UART send: wrote %d bytes", bytes_written);
    taskYIELD();
    return ESP_OK;
}

int uart_gateway_receive(uint8_t *buffer, size_t max_length)
{
    if (!gateway_ctx.is_configured || buffer == NULL || max_length == 0)
    {
        return 0;
    }

    int bytes_read = uart_read_bytes(gateway_ctx.uart_num, buffer, max_length, pdMS_TO_TICKS(10));

    return bytes_read;
}

bool uart_gateway_is_ready(void)
{
    return gateway_ctx.is_configured;
}

void uart_gateway_deinit(void)
{
    if (gateway_ctx.is_configured)
    {
        uart_driver_delete(gateway_ctx.uart_num);
        gateway_ctx.is_configured = false;
    }
}

esp_err_t uart_gateway_save_config(void)
{
    esp_err_t err;
    nvs_handle_t nvs_handle;

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Save baud rate */
    err = nvs_set_u32(nvs_handle, NVS_KEY_BAUD, gateway_ctx.current_config.baud_rate);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to save baud rate: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    /* Save TX GPIO */
    err = nvs_set_u8(nvs_handle, NVS_KEY_TX_GPIO, gateway_ctx.current_config.tx_gpio);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to save TX GPIO: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    /* Save RX GPIO */
    err = nvs_set_u8(nvs_handle, NVS_KEY_RX_GPIO, gateway_ctx.current_config.rx_gpio);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to save RX GPIO: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    /* Save Reset GPIO */
    err = nvs_set_u8(nvs_handle, NVS_KEY_RESET_GPIO, gateway_ctx.current_config.reset_gpio);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to save Reset GPIO: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    /* Save Control GPIO */
    err = nvs_set_u8(nvs_handle, NVS_KEY_CONTROL_GPIO, gateway_ctx.current_config.control_gpio);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to save Control GPIO: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    /* Save LED GPIO */
    err = nvs_set_u8(nvs_handle, NVS_KEY_LED_GPIO, gateway_ctx.current_config.led_gpio);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to save LED GPIO: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    /* Save extended mode */
    err = nvs_set_u8(nvs_handle, NVS_KEY_EXTENDED_MODE, gateway_ctx.current_config.extended_mode);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to save extended mode: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    /* Commit changes */
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Configuration saved to NVS: baud=%lu, TX=%u, RX=%u, RESET=%u, CONTROL=%u, LED=%u, EXT_MODE=%u",
                 gateway_ctx.current_config.baud_rate,
                 gateway_ctx.current_config.tx_gpio,
                 gateway_ctx.current_config.rx_gpio,
                 gateway_ctx.current_config.reset_gpio,
                 gateway_ctx.current_config.control_gpio,
                 gateway_ctx.current_config.led_gpio,
                 gateway_ctx.current_config.extended_mode);
    }
    else
    {
        ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));
    }

    return err;
}

esp_err_t uart_gateway_load_config(uartgw_config_t *loaded_config)
{
    esp_err_t err;
    nvs_handle_t nvs_handle;
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "No saved configuration in NVS");
        return ESP_ERR_NVS_NOT_FOUND;
    }
    else if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Load baud rate */
    err = nvs_get_u32(nvs_handle, NVS_KEY_BAUD, &loaded_config->baud_rate);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to load baud rate: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    /* Load TX GPIO */
    err = nvs_get_u8(nvs_handle, NVS_KEY_TX_GPIO, &loaded_config->tx_gpio);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to load TX GPIO: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    /* Load RX GPIO */
    err = nvs_get_u8(nvs_handle, NVS_KEY_RX_GPIO, &loaded_config->rx_gpio);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to load RX GPIO: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    /* Load Reset GPIO (optional, use default if not found) */
    err = nvs_get_u8(nvs_handle, NVS_KEY_RESET_GPIO, &loaded_config->reset_gpio);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        loaded_config->reset_gpio = UART_DEFAULT_RESET_GPIO;
        ESP_LOGI(TAG, "Reset GPIO not in NVS, using default: %u", UART_DEFAULT_RESET_GPIO);
        err = ESP_OK;
    }
    else if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to load Reset GPIO: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    /* Load Control GPIO (optional, use default if not found) */
    err = nvs_get_u8(nvs_handle, NVS_KEY_CONTROL_GPIO, &loaded_config->control_gpio);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        loaded_config->control_gpio = UART_DEFAULT_CONTROL_GPIO;
        ESP_LOGI(TAG, "Control GPIO not in NVS, using default: %u", UART_DEFAULT_CONTROL_GPIO);
        err = ESP_OK;
    }
    else if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to load Control GPIO: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    /* Load LED GPIO (optional, use default if not found) */
    err = nvs_get_u8(nvs_handle, NVS_KEY_LED_GPIO, &loaded_config->led_gpio);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        loaded_config->led_gpio = UART_DEFAULT_LED_GPIO;
        ESP_LOGI(TAG, "LED GPIO not in NVS, using default: %u", UART_DEFAULT_LED_GPIO);
        err = ESP_OK;
    }
    else if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to load LED GPIO: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    /* Load extended mode (optional, use default 0 if not found) */
    err = nvs_get_u8(nvs_handle, NVS_KEY_EXTENDED_MODE, &loaded_config->extended_mode);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        loaded_config->extended_mode = 0;
        ESP_LOGI(TAG, "Extended mode not in NVS, using default: 0");
        err = ESP_OK;
    }
    else if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to load extended mode: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    nvs_close(nvs_handle);

    return err;
}

void uart_gateway_build_response_packet(uart_config_packet_t *pkt)
{
    if (pkt == NULL)
    {
        return;
    }

    /* Fill config (automatic little-endian in struct) */
    pkt->baud_rate = gateway_ctx.current_config.baud_rate;
    pkt->tx_gpio = gateway_ctx.current_config.tx_gpio;
    pkt->rx_gpio = gateway_ctx.current_config.rx_gpio;
    pkt->reset_gpio = gateway_ctx.current_config.reset_gpio;
    pkt->control_gpio = gateway_ctx.current_config.control_gpio;
    pkt->led_gpio = gateway_ctx.current_config.led_gpio;
    pkt->extended_mode = gateway_ctx.current_config.extended_mode;

    ESP_LOGI(TAG, "Response packet built: baud=%lu, TX=%u, RX=%u, RESET=%u, CONTROL=%u, LED=%u",
             gateway_ctx.current_config.baud_rate,
             gateway_ctx.current_config.tx_gpio,
             gateway_ctx.current_config.rx_gpio,
             gateway_ctx.current_config.reset_gpio,
             gateway_ctx.current_config.control_gpio,
             gateway_ctx.current_config.led_gpio);
}

/* Queue data from CDC to UART */
esp_err_t uart_gateway_queue_cdc_data(const uint8_t *data, size_t length)
{
    if (gateway_ctx.cdc_to_uart_buffer == NULL || data == NULL || length == 0)
    {
        ESP_LOGE(TAG, "Queue CDC: invalid args - buffer=%p, data=%p, len=%zu",
                 gateway_ctx.cdc_to_uart_buffer, data, length);
        return ESP_ERR_INVALID_ARG;
    }

    size_t bytes_sent = xStreamBufferSend(gateway_ctx.cdc_to_uart_buffer, data, length, portMAX_DELAY);
    if (bytes_sent == length)
    {
        ESP_LOGD(TAG, "Buffered %zu bytes CDC->UART", length);
        taskYIELD();
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to buffer %zu bytes CDC->UART (sent %zu)", length, bytes_sent);
    return ESP_FAIL;
}

/* Receive data from CDC buffer (intended for UART) */
int uart_gateway_receive_cdc_queue(uint8_t *buffer, size_t max_length)
{
    if (gateway_ctx.cdc_to_uart_buffer == NULL || buffer == NULL || max_length == 0)
    {
        return 0;
    }

    size_t bytes_received = xStreamBufferReceive(gateway_ctx.cdc_to_uart_buffer, buffer, max_length, pdMS_TO_TICKS(20));

    return bytes_received;
}

/* Queue data from UART to CDC */
esp_err_t queue_packet(uart_packet_header_t *packet)
{
    if (packet == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (gateway_ctx.uart_to_cdc_buffer == NULL)
    {
        free(packet);
        return ESP_ERR_INVALID_ARG;
    }

    /* Queue the packet pointer (block indefinitely for back pressure) */
    if (xQueueSend(gateway_ctx.uart_to_cdc_buffer, &packet, portMAX_DELAY) == pdTRUE)
    {
        taskYIELD();
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to queue packet to UART->CDC");
    free(packet);
    return ESP_FAIL;
}

/* Receive data from UART buffer (intended for CDC) */
/* Returns pointer to packet header on success, NULL on timeout */
uart_packet_header_t *unqueue_packet(void)
{
    if (gateway_ctx.uart_to_cdc_buffer == NULL)
    {
        return NULL;
    }

    uart_packet_header_t *packet = NULL;
    if (xQueueReceive(gateway_ctx.uart_to_cdc_buffer, &packet, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        return packet;
    }

    return NULL;
}

/* Task: Read from USB CDC, queue to UART */
static void cdc_read_task(void *pvParameters)
{
    int bytes_read;
    uartgw_config_t new_config;

    /* Extended mode state machine */
    uint8_t header_buffer[UART_PACKET_HEADER_SIZE];
    uart_packet_header_t *header = (uart_packet_header_t *)header_buffer;
    size_t header_pos = 0;
    bool header_received = false;

    uint8_t *payload_buffer = NULL;
    size_t payload_needed = 0;
    size_t payload_pos = 0;

    /* Normal mode magic detection ring buffer */
    uint8_t magic_ring[UART_EXTMODE_MAGIC_SIZE] = {0};
    size_t magic_ring_pos = 0;

    /* Wait a bit for system to fully initialize */
    // vTaskDelay(200 / portTICK_PERIOD_MS);

    /* Send startup message to unblock host reader */
    send_message("ESP32C3 UART Gateway ready");

    while (1)
    {
        if (!uart_gateway_is_ready())
        {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            continue;
        }

        /* Read from USB CDC with timeout */
        bytes_read = usb_serial_jtag_read_bytes(cdc_read_buffer, CDC_BUFFER_SIZE, pdMS_TO_TICKS(10));

        if (bytes_read <= 0)
        {
            continue;
        }

        if (bytes_read > (int)CDC_BUFFER_SIZE)
        {
            bytes_read = (int)CDC_BUFFER_SIZE;
        }

        /* NORMAL MODE: search for magic bytes in incoming data */
        if (gateway_ctx.current_config.extended_mode == 0)
        {
            /* Accumulate bytes in ring buffer */
            int bytes_to_add = (bytes_read < (int)(UART_EXTMODE_MAGIC_SIZE - magic_ring_pos))
                                   ? bytes_read
                                   : (UART_EXTMODE_MAGIC_SIZE - magic_ring_pos);

            if (bytes_to_add > 0)
            {
                memcpy(&magic_ring[magic_ring_pos], cdc_read_buffer, bytes_to_add);
                magic_ring_pos += bytes_to_add;
            }

            /* Check if we have enough bytes and if they match magic */
            if (magic_ring_pos >= UART_EXTMODE_MAGIC_SIZE)
            {
                if (memcmp(magic_ring, uart_extmode_magic, UART_EXTMODE_MAGIC_SIZE) == 0)
                {
                    /* Magic found! Switch to extended mode */
                    gateway_ctx.current_config.extended_mode = 1;
                    header_pos = 0;
                    if (payload_buffer)
                    {
                        free(payload_buffer);
                        payload_buffer = NULL;
                    }
                    payload_needed = 0;
                    payload_pos = 0;
                    memset(magic_ring, 0, sizeof(magic_ring));
                    magic_ring_pos = 0;

                    send_message("Extended mode activated (temporary, until reboot)");

                    /* Process any remaining bytes in this read as extended mode */
                    if (bytes_to_add < bytes_read)
                    {
                        int remaining = bytes_read - bytes_to_add;
                        memmove(cdc_read_buffer, &cdc_read_buffer[bytes_to_add], remaining);
                        bytes_read = remaining;
                        /* Fall through to extended mode processing below */
                    }
                    else
                    {
                        continue;
                    }
                }
                else
                {
                    /* Not magic, shift buffer forward and queue first byte */
                    led_signal_uart_activity();
                    uart_gateway_queue_cdc_data(&magic_ring[0], 1);

                    /* Shift remaining bytes forward */
                    memmove(magic_ring, &magic_ring[1], UART_EXTMODE_MAGIC_SIZE - 1);
                    magic_ring_pos = UART_EXTMODE_MAGIC_SIZE - 1;

                    /* Add next byte from current read if available */
                    if (bytes_to_add < bytes_read)
                    {
                        magic_ring[magic_ring_pos] = cdc_read_buffer[bytes_to_add];
                        magic_ring_pos++;

                        /* Queue remaining bytes from this read */
                        int remaining = bytes_read - bytes_to_add - 1;
                        if (remaining > 0)
                        {
                            led_signal_uart_activity();
                            uart_gateway_queue_cdc_data(&cdc_read_buffer[bytes_to_add + 1], remaining);
                        }
                    }
                    continue;
                }
            }
            else
            {
                /* Not enough bytes yet, continue accumulating */
                continue;
            }
        }

        /* EXTENDED MODE: parse packets with headers */
        if (gateway_ctx.current_config.extended_mode == 1)
        {
            /* Process incoming bytes */
            for (int i = 0; i < bytes_read; i++)
            {
                uint8_t data_byte = cdc_read_buffer[i];

                if (!header_received)
                {
                    /* State 1: Reading header */
                    if (header_pos < UART_PACKET_HEADER_SIZE)
                    {
                        header_buffer[header_pos++] = data_byte;
                    }

                    /* Header complete, parse it */
                    if (header_pos == UART_PACKET_HEADER_SIZE)
                    {
                        header_pos = 0;
                        header_received = true;

                        /* Validate length (minimum 4 for header) */
                        if (header->length < sizeof(uart_packet_header_t))
                        {
                            ESP_LOGW(TAG, "Invalid packet length: %u (minimum %zu), resetting", header->length, sizeof(uart_packet_header_t));
                            header_received = false;
                            continue;
                        }

                        payload_needed = header->length - sizeof(uart_packet_header_t);
                        payload_pos = 0;

                        /* Allocate payload buffer if needed */
                        if (payload_needed > 0)
                        {
                            if (payload_buffer)
                            {
                                free(payload_buffer);
                            }
                            payload_buffer = (uint8_t *)malloc(payload_needed);
                            if (!payload_buffer)
                            {
                                ESP_LOGE(TAG, "Failed to allocate %zu bytes for packet payload", payload_needed);
                                header_received = false;
                                continue;
                            }
                        }
                    }
                }
                else
                {
                    /* State 2: Reading payload */
                    if (payload_pos < payload_needed)
                    {
                        payload_buffer[payload_pos++] = data_byte;
                    }

                    /* Payload complete, process packet */
                    if (payload_pos == payload_needed)
                    {
                        header_received = false;
                        payload_pos = 0;

                        switch (header->type)
                        {
                        case UART_PACKET_TYPE_DATA:
                            /* Type 0x00: Normal serial data - send to UART */
                            led_signal_uart_activity();
                            uart_gateway_queue_cdc_data(payload_buffer, payload_needed);
                            break;

                        case UART_PACKET_TYPE_CONFIG:
                            /* Type 0x01: Configuration packet */
                            if (parse_config_packet(payload_buffer, payload_needed, &new_config))
                            {
                                if (new_config.baud_rate == 0)
                                {
                                    send_message("CONFIG query received");
                                }
                                else
                                {
                                    send_message("CONFIG update");
                                    uart_gateway_configure(&new_config);
                                    uart_gateway_save_config();
                                }
                                send_current_config();
                            }
                            else
                            {
                                send_message("Failed to parse CONFIG packet");
                            }
                            break;

                        case UART_PACKET_TYPE_CONTROL:
                            /* Type 0x02: Control commands */
                            parse_control_command((const char *)payload_buffer, payload_needed);
                            break;

                        case UART_PACKET_TYPE_EXTMODE:
                            send_message("Extended mode activated");
                            break;

                        default:
                            send_message("Unknown packet type: 0x%04X", header->type);
                            break;
                        }

                        /* Free payload and reset for next packet */
                        if (payload_buffer)
                        {
                            free(payload_buffer);
                            payload_buffer = NULL;
                        }
                        header_pos = 0;
                        payload_needed = 0;
                        payload_pos = 0;
                    }
                }
            }
        }
    }
}

/* Task: Write queued CDC data to UART */
static void uart_write_task(void *pvParameters)
{
    int bytes_to_send;

    /* Wait a bit for system to fully initialize */
    // vTaskDelay(200 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "UART write task ready");

    while (1)
    {
        if (!uart_gateway_is_ready())
        {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            continue;
        }

        bytes_to_send = uart_gateway_receive_cdc_queue(uart_write_buffer, UART_BUFFER_SIZE);

        if (bytes_to_send <= 0)
        {
            continue;
        }

        esp_err_t ret = uart_gateway_send(uart_write_buffer, bytes_to_send);

        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "UART write: sent %d bytes successfully", bytes_to_send);
        }
        else
        {
            ESP_LOGE(TAG, "UART write: failed to send %d bytes", bytes_to_send);
        }
    }
}

/* Task: Read from UART, queue to CDC */
static void uart_read_task(void *pvParameters)
{
    int bytes_received;

    ESP_LOGI(TAG, "UART read task started");

    /* Wait a bit for system to fully initialize */
    // vTaskDelay(200 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "UART read task ready");

    while (1)
    {
        if (!uart_gateway_is_ready())
        {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            continue;
        }

        bytes_received = uart_gateway_receive(uart_read_buffer, UART_BUFFER_SIZE);

        if (bytes_received <= 0)
        {
            continue;
        }

        ESP_LOGI(TAG, "UART read: received %d bytes", bytes_received);

        led_signal_uart_activity();
        uart_packet_header_t *packet = (uart_packet_header_t *)malloc(sizeof(uart_packet_header_t) + bytes_received);
        if (packet == NULL)
        {
            ESP_LOGE(TAG, "UART read: failed to allocate packet");
            continue;
        }
        packet->type = UART_PACKET_TYPE_DATA;
        packet->length = sizeof(uart_packet_header_t) + bytes_received;

        memcpy(PTR_BEHIND(packet), uart_read_buffer, bytes_received);

        if (queue_packet(packet) != ESP_OK)
        {
            ESP_LOGE(TAG, "UART read: failed to queue %d bytes", bytes_received);
        }
    }
}

/* Task: Write queued UART data to CDC */
static void cdc_write_task(void *pvParameters)
{
    uart_packet_header_t *packet_header;
    uint16_t packet_type;
    uint16_t packet_length;
    uint8_t *payload;
    size_t payload_len;
    int written;

    ESP_LOGI(TAG, "CDC write task started");

    /* Wait a bit for system to fully initialize */
    vTaskDelay(200 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "CDC write task ready");

    while (1)
    {
        packet_header = unqueue_packet();

        if (packet_header == NULL)
        {
            continue;
        }

        /* Parse header */
        packet_length = packet_header->length;
        packet_type = packet_header->type;

        /* Validate packet length (minimum 2 to include type field) */
        if (packet_length < sizeof(uart_packet_header_t))
        {
            ESP_LOGW(TAG, "CDC write: invalid packet length %u (minimum %zu)", packet_length, sizeof(uart_packet_header_t));
            free(packet_header);
            continue;
        }

        /* Calculate payload length (packet_length includes the type field, so subtract header size) */
        payload_len = packet_length - sizeof(uart_packet_header_t);

        /* Extract payload pointer (after header) */
        payload = (uint8_t *)PTR_BEHIND(packet_header);

        ESP_LOGI(TAG, "CDC write: processing packet type=0x%04X, payload_len=%zu, extended_mode=%u",
                 packet_type, payload_len, gateway_ctx.current_config.extended_mode);

        /* Handle packet based on extended mode */
        if (gateway_ctx.current_config.extended_mode == 0)
        {
            /* Non-extended mode: discard all but UART_PACKET_TYPE_DATA, send only payload */
            if (packet_type == UART_PACKET_TYPE_DATA)
            {
                ESP_LOGI(TAG, "CDC write: sending DATA payload (%zu bytes)", payload_len);
                led_signal_uart_activity();
                size_t offset = 0;
                while (offset < payload_len)
                {
                    size_t chunk = payload_len - offset;
                    if (chunk > CDC_BUFFER_SIZE)
                    {
                        chunk = CDC_BUFFER_SIZE;
                    }
                    written = usb_serial_jtag_write_bytes(payload + offset, chunk, pdMS_TO_TICKS(1000));
                    if (written != (int)chunk)
                    {
                        ESP_LOGW(TAG, "CDC write: incomplete write %d/%zu", written, chunk);
                        break;
                    }
                    offset += chunk;
                }
            }
            else
            {
                /* Discard non-DATA packets in non-extended mode */
                ESP_LOGD(TAG, "CDC write: discarding packet type=0x%04X in non-extended mode", packet_type);
            }
        }
        else
        {
            /* Extended mode: send complete packet (header + payload) in fragments */
            const uint8_t *out = (const uint8_t *)packet_header;
            size_t out_len = packet_header->length;
            size_t offset = 0;
            while (offset < out_len)
            {
                size_t chunk = out_len - offset;
                if (chunk > CDC_BUFFER_SIZE)
                {
                    chunk = CDC_BUFFER_SIZE;
                }
                written = usb_serial_jtag_write_bytes(out + offset, chunk, pdMS_TO_TICKS(1000));
                if (written != (int)chunk)
                {
                    ESP_LOGW(TAG, "CDC write: incomplete write %d/%zu", written, chunk);
                    break;
                }
                offset += chunk;
            }
        }

        /* Free the allocated packet */
        free(packet_header);
        taskYIELD();
    }
}

void uart_gateway_start(void)
{
    /* Initialize USB CDC (enabled by default on ESP32-C3) */
    usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    esp_err_t usb_res = usb_serial_jtag_driver_install(&usb_cfg);
    if (usb_res != ESP_OK)
    {
        ESP_LOGE(TAG, "USB CDC init failed: %s", esp_err_to_name(usb_res));
        return;
    }
    ESP_LOGI(TAG, "USB CDC initialized");

    /* Create queue-based relay tasks */
    BaseType_t ret1 = xTaskCreatePinnedToCore(cdc_read_task, "cdc_read", 4096, NULL, 5, &cdc_read_task_handle, 0);
    BaseType_t ret2 = xTaskCreatePinnedToCore(uart_write_task, "uart_write", 4096, NULL, 5, &uart_write_task_handle, 0);
    BaseType_t ret3 = xTaskCreatePinnedToCore(uart_read_task, "uart_read", 4096, NULL, 5, &uart_read_task_handle, 0);
    BaseType_t ret4 = xTaskCreatePinnedToCore(cdc_write_task, "cdc_write", 4096, NULL, 5, &cdc_write_task_handle, 0);

    ESP_LOGI(TAG, "CDC read task creation: %s", ret1 == pdPASS ? "OK" : "FAILED");
    ESP_LOGI(TAG, "UART write task creation: %s", ret2 == pdPASS ? "OK" : "FAILED");
    ESP_LOGI(TAG, "UART read task creation: %s", ret3 == pdPASS ? "OK" : "FAILED");
    ESP_LOGI(TAG, "CDC write task creation: %s", ret4 == pdPASS ? "OK" : "FAILED");

    if (!(ret1 == pdPASS && ret2 == pdPASS && ret3 == pdPASS && ret4 == pdPASS))
    {
        ESP_LOGE(TAG, "Failed to create some relay tasks");
        return;
    }

    ESP_LOGI(TAG, "Gateway tasks started");
}

void uart_gateway_stop(void)
{
    /* Delete tasks if running */
    if (cdc_read_task_handle)
    {
        vTaskDelete(cdc_read_task_handle);
        cdc_read_task_handle = NULL;
    }
    if (uart_write_task_handle)
    {
        vTaskDelete(uart_write_task_handle);
        uart_write_task_handle = NULL;
    }
    if (uart_read_task_handle)
    {
        vTaskDelete(uart_read_task_handle);
        uart_read_task_handle = NULL;
    }
    if (cdc_write_task_handle)
    {
        vTaskDelete(cdc_write_task_handle);
        cdc_write_task_handle = NULL;
    }

    /* Uninstall USB CDC */
    usb_serial_jtag_driver_uninstall();
    ESP_LOGI(TAG, "Gateway tasks stopped and USB CDC uninstalled");
}