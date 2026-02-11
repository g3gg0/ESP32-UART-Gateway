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

/* Magic bytes: pattern for config packet identification */
const uint8_t uart_config_magic[UART_CONFIG_MAGIC_SIZE] = {
    0x55,
    0x41,
    0x52,
    0x54,
    0x47,
    0x57,
    0x01,
    0x00};

/* Magic bytes: pattern for control packet identification */
const uint8_t uart_control_magic[UART_CONTROL_MAGIC_SIZE] = {
    0x43,
    0x54,
    0x52,
    0x4C,
    0x47,
    0x57,
    0x01,
    0x00};

/* Magic bytes: pattern for log message packet identification */
const uint8_t uart_logmsg_magic[UART_LOGMSG_MAGIC_SIZE] = {
    0x4C,
    0x4F,
    0x47,
    0x4D,
    0x53,
    0x47,
    0x01,
    0x00};

/* CDC buffer for data flow */
#define CDC_BUFFER_SIZE 64
#define UART_BUFFER_SIZE 2048

static TaskHandle_t cdc_read_task_handle = NULL;
static TaskHandle_t cdc_write_task_handle = NULL;
static TaskHandle_t uart_read_task_handle = NULL;
static TaskHandle_t uart_write_task_handle = NULL;

/* Queue for config responses */
static QueueHandle_t config_response_queue = NULL;

/* Task buffers (global to save stack space) */
static uint8_t cdc_read_buffer[CDC_BUFFER_SIZE];
static uint8_t uart_write_buffer[UART_BUFFER_SIZE];
static uint8_t uart_read_buffer[UART_BUFFER_SIZE];
static uint8_t cdc_write_buffer[CDC_BUFFER_SIZE];

typedef struct
{
    uart_port_t uart_num;
    uartgw_config_t current_config;
    bool is_configured;
    bool is_initializing;
    uint8_t config_buffer[UART_CONFIG_PACKET_SIZE];
    size_t config_buffer_pos;
    uart_config_callback_t config_callback;
    StreamBufferHandle_t cdc_to_uart_buffer;
    StreamBufferHandle_t uart_to_cdc_buffer;
} uart_gateway_ctx_t;

static uart_gateway_ctx_t gateway_ctx = {
    .uart_num = UART_NUM_0,
    .is_configured = false,
    .is_initializing = false,
    .config_buffer_pos = 0,
    .config_callback = NULL,
    .cdc_to_uart_buffer = NULL,
    .uart_to_cdc_buffer = NULL,
    .current_config = {
        .baud_rate = UART_DEFAULT_BAUD,
        .tx_gpio = UART_DEFAULT_TX_GPIO,
        .rx_gpio = UART_DEFAULT_RX_GPIO,
        .reset_gpio = UART_DEFAULT_RESET_GPIO,
        .control_gpio = UART_DEFAULT_CONTROL_GPIO,
        .led_gpio = UART_DEFAULT_LED_GPIO,
    },
};


static esp_err_t reconfigure_uart(const uartgw_config_t *config);

static void IRAM_ATTR send_current_config()
{
    /* Queue the config response to be sent by cdc_write_task */
    if (!config_response_queue)
    {
        return;
    }

    uint8_t response_packet[UART_CONFIG_PACKET_SIZE];
    size_t response_len;
    uart_gateway_build_response_packet(response_packet, &response_len);

    /* Send to queue */
    BaseType_t yield_required = pdFALSE;
    if (xQueueSendFromISR(config_response_queue, response_packet, &yield_required) == pdPASS)
    {
        ESP_LOGI(TAG, "Config response queued (%zu bytes)", response_len);
        if (yield_required)
        {
            portYIELD_FROM_ISR();
        }
    }
    else
    {
        ESP_LOGW(TAG, "Failed to queue config response");
    }
}

/* Callback: Queue config response when it changes */
static void IRAM_ATTR on_config_changed(const uartgw_config_t *config)
{
    ESP_LOGI(TAG, "CONFIG CHANGE CALLBACK: baud=%lu, TX=%u, RX=%u", config->baud_rate, config->tx_gpio, config->rx_gpio);

    esp_err_t err = uart_gateway_save_config();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to save configuration: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "Configuration saved to NVS");
    }
    send_current_config();
}

void send_message(const char *fmt, ...)
{
    if (fmt == NULL)
    {
        ESP_LOGE(TAG, "send_message: fmt is NULL");
        return;
    }

    if (gateway_ctx.uart_to_cdc_buffer == NULL)
    {
        ESP_LOGE(TAG, "send_message: UART->CDC buffer not initialized");
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

    uint8_t packet[UART_LOGMSG_HEADER_SIZE + UART_LOGMSG_MAX_LEN];
    uart_logmsg_header_t header;
    memcpy(header.magic, uart_logmsg_magic, UART_LOGMSG_MAGIC_SIZE);
    header.length = (uint32_t)msg_len;
    for (size_t i = 0; i < UART_LOGMSG_MAGIC_SIZE; i++)
    {
        header.inv_magic[i] = uart_logmsg_magic[i] ^ 0xFF;
    }

    memcpy(packet, &header, UART_LOGMSG_HEADER_SIZE);
    if (msg_len > 0)
    {
        memcpy(packet + UART_LOGMSG_HEADER_SIZE, message, msg_len);
    }

    esp_err_t err = uart_gateway_queue_uart_data(packet, UART_LOGMSG_HEADER_SIZE + msg_len);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "send_message: failed to queue log packet: %s", esp_err_to_name(err));
    }
}

/* Check if magic bytes match (normal or inverted) */
static bool check_magic(const uint8_t *data, const uint8_t *magic, size_t magic_len, bool inverted)
{
    for (size_t i = 0; i < magic_len; i++)
    {
        if (data[i] != (magic[i] ^ ((inverted) ? 0xFF : 0x00)))
        {
            return false;
        }
    }
    return true;
}

/* Parse configuration packet */
static bool parse_config_packet(const uint8_t *packet_data, size_t bytes_read, uartgw_config_t *config)
{
    if (bytes_read < UART_CONFIG_PACKET_SIZE)
    {
        return false;
    }

    /* Cast to struct for easy access */
    const uart_config_packet_t *pkt = (const uart_config_packet_t *)packet_data;

    /* Check magic at start */
    if (!check_magic(pkt->magic, uart_config_magic, UART_CONFIG_MAGIC_SIZE, false))
    {
        ESP_LOGW(TAG, "Config packet: magic mismatch at start");
        return false;
    }

    /* Check inverted magic at end */
    if (!check_magic(pkt->inv_magic, uart_config_magic, UART_CONFIG_MAGIC_SIZE, true))
    {
        ESP_LOGW(TAG, "Config packet: inverted magic mismatch");
        return false;
    }

    /* Extract from struct (automatic little-endian) */
    uint32_t baud = pkt->baud_rate;
    uint8_t tx_gpio = pkt->tx_gpio;
    uint8_t rx_gpio = pkt->rx_gpio;
    uint8_t reset_gpio = pkt->reset_gpio;
    uint8_t control_gpio = pkt->control_gpio;
    uint8_t led_gpio = pkt->led_gpio;

    /* Validate parameters (baud_rate == 0 is query mode, allowed) */
    if (baud != 0)
    {
        if (baud < 300 || baud > 1000000)
        {
            ESP_LOGW(TAG, "Invalid baud rate: %lu", baud);
            return false;
        }

        if (tx_gpio > 43 || rx_gpio > 43)
        {
            ESP_LOGW(TAG, "Invalid GPIO pins: TX=%u, RX=%u", tx_gpio, rx_gpio);
            return false;
        }

        if ((reset_gpio > 43 && reset_gpio != 0xFF) || (control_gpio > 43 && control_gpio != 0xFF) || (led_gpio > 43 && led_gpio != 0xFF))
        {
            ESP_LOGW(TAG, "Invalid GPIO pins: RESET=%u, CONTROL=%u, LED=%u", reset_gpio, control_gpio, led_gpio);
            return false;
        }

        if (tx_gpio == rx_gpio)
        {
            ESP_LOGW(TAG, "TX and RX pins cannot be the same");
            return false;
        }
    }

    config->baud_rate = baud;
    config->tx_gpio = tx_gpio;
    config->rx_gpio = rx_gpio;
    config->reset_gpio = reset_gpio;
    config->control_gpio = control_gpio;
    config->led_gpio = led_gpio;

    return true;
}

/* Parse control packet and execute command */
static bool parse_control_packet(const uint8_t *packet_data, size_t bytes_read)
{
    if (bytes_read < UART_CONTROL_PACKET_SIZE)
    {
        return false;
    }

    /* Cast to struct for easy access */
    const uart_control_packet_t *pkt = (const uart_control_packet_t *)packet_data;

    /* Check magic at start */
    if (!check_magic(pkt->magic, uart_control_magic, UART_CONTROL_MAGIC_SIZE, false))
    {
        return false;
    }

    /* Check inverted magic at end */
    if (!check_magic(pkt->inv_magic, uart_control_magic, UART_CONTROL_MAGIC_SIZE, true))
    {
        return false;
    }

    /* Parse command string */
    char cmd[UART_CONTROL_CMD_SIZE + 1] = {0};
    memcpy(cmd, pkt->command, UART_CONTROL_CMD_SIZE);

    /* Ensure null termination */
    cmd[UART_CONTROL_CMD_SIZE] = '\0';

    /* Parse R:x (Reset control) */
    if (cmd[0] == 'R' && cmd[1] == ':')
    {
        /* Check if reset GPIO is configured (not 0xFF) */
        if (gateway_ctx.current_config.reset_gpio == 0xFF)
        {
            send_message("Reset GPIO not configured (0xFF), ignoring R: command");
            return false;
        }

        /* Set reset line state */
        gpio_set_level(gateway_ctx.current_config.reset_gpio, (cmd[2] == '0') ? 0 : 1);
        return true;
    }
    /* Parse C:x (Control pin) */
    else if (cmd[0] == 'C' && cmd[1] == ':')
    {
        /* Check if control GPIO is configured (not 0xFF) */
        if (gateway_ctx.current_config.control_gpio == 0xFF)
        {
            send_message("Control GPIO not configured (0xFF), ignoring C: command");
            return false;
        }

        gpio_set_level(gateway_ctx.current_config.control_gpio, (cmd[2] == '0') ? 0 : 1);
        return true;
    }
    /* Parse B:xx (Break condition) */
    else if (cmd[0] == 'B' && cmd[1] == ':')
    {
        int brk_len = atoi(&cmd[2]);

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

        if(brk_len < 20)
        {
            esp_rom_delay_us(brk_len * 1000);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(brk_len));            
        }

        gpio_set_level(gateway_ctx.current_config.tx_gpio, 1);

        reconfigure_uart(&gateway_ctx.current_config);

        return true;
    }
    else
    {
        send_message("Unknown control command: '%s'", cmd);
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

    /* Call callback if registered and not during initialization */
    if (gateway_ctx.config_callback != NULL && !gateway_ctx.is_initializing)
    {
        ESP_LOGI(TAG, "Calling config change callback");
        gateway_ctx.config_callback(config);
    }

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
        gateway_ctx.uart_to_cdc_buffer = xStreamBufferCreate(STREAM_BUFFER_SIZE, 1);
        if (gateway_ctx.uart_to_cdc_buffer == NULL)
        {
            ESP_LOGE(TAG, "Failed to create UART->CDC stream buffer");
        }
        else
        {
            ESP_LOGI(TAG, "UART->CDC stream buffer created (%u bytes)", STREAM_BUFFER_SIZE);
        }
    }

    ESP_LOGI(TAG, "Initializing GPIOs...");
    reconfigure_gpio(config);
    ESP_LOGI(TAG, "Initializing UART gateway...");
    reconfigure_uart(config);

    /* Initialize all unused GPIO pins to GND with strong drive */
    init_unused_gpios_to_gnd(config->tx_gpio, config->rx_gpio, config->reset_gpio, config->control_gpio, config->led_gpio);

    gateway_ctx.is_initializing = false;
}

void uart_gateway_register_callback(uart_config_callback_t callback)
{
    gateway_ctx.config_callback = callback;
    ESP_LOGI(TAG, "Configuration change callback registered");
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

    /* Commit changes */
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Configuration saved to NVS: baud=%lu, TX=%u, RX=%u, RESET=%u, CONTROL=%u, LED=%u",
                 gateway_ctx.current_config.baud_rate,
                 gateway_ctx.current_config.tx_gpio,
                 gateway_ctx.current_config.rx_gpio,
                 gateway_ctx.current_config.reset_gpio,
                 gateway_ctx.current_config.control_gpio,
                 gateway_ctx.current_config.led_gpio);
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

    nvs_close(nvs_handle);

    return err;
}

void uart_gateway_build_response_packet(uint8_t *packet, size_t *packet_len)
{
    if (packet == NULL || packet_len == NULL)
    {
        return;
    }

    /* Cast to struct and fill it */
    uart_config_packet_t *pkt = (uart_config_packet_t *)packet;

    /* Fill magic at start */
    memcpy(pkt->magic, uart_config_magic, UART_CONFIG_MAGIC_SIZE);

    /* Fill config (automatic little-endian in struct) */
    pkt->baud_rate = gateway_ctx.current_config.baud_rate;
    pkt->tx_gpio = gateway_ctx.current_config.tx_gpio;
    pkt->rx_gpio = gateway_ctx.current_config.rx_gpio;
    pkt->reset_gpio = gateway_ctx.current_config.reset_gpio;
    pkt->control_gpio = gateway_ctx.current_config.control_gpio;
    pkt->led_gpio = gateway_ctx.current_config.led_gpio;

    /* Fill inverted magic at end */
    for (int i = 0; i < UART_CONFIG_MAGIC_SIZE; i++)
    {
        pkt->inv_magic[i] = uart_config_magic[i] ^ 0xFF;
    }

    *packet_len = UART_CONFIG_PACKET_SIZE;

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
esp_err_t uart_gateway_queue_uart_data(const uint8_t *data, size_t length)
{
    if (gateway_ctx.uart_to_cdc_buffer == NULL || data == NULL || length == 0)
    {
        ESP_LOGE(TAG, "Queue UART: invalid args - buffer=%p, data=%p, len=%zu", gateway_ctx.uart_to_cdc_buffer, data, length);
        return ESP_ERR_INVALID_ARG;
    }

    /* Block indefinitely until all data fits in the buffer (back pressure) */
    size_t bytes_sent = xStreamBufferSend(gateway_ctx.uart_to_cdc_buffer, data, length, portMAX_DELAY);
    if (bytes_sent == length)
    {
        ESP_LOGD(TAG, "Buffered %zu bytes UART->CDC", length);
        taskYIELD();
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to buffer %zu bytes UART->CDC (sent %zu)", length, bytes_sent);
    return ESP_FAIL;
}

/* Receive data from UART buffer (intended for CDC) */
int uart_gateway_receive_uart_queue(uint8_t *buffer, size_t max_length)
{
    if (gateway_ctx.uart_to_cdc_buffer == NULL || buffer == NULL || max_length == 0)
    {
        return 0;
    }

    size_t bytes_received = xStreamBufferReceive(gateway_ctx.uart_to_cdc_buffer, buffer, max_length, pdMS_TO_TICKS(20));

    return bytes_received;
}

/* Task: Read from USB CDC, queue to UART */
static void cdc_read_task(void *pvParameters)
{
    int bytes_read;
    uartgw_config_t new_config;

    ESP_LOGI(TAG, "CDC read task started");

    /* Wait a bit for system to fully initialize */
    vTaskDelay(200 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "CDC read task ready");

    while (1)
    {
        if (!uart_gateway_is_ready())
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
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

        /* Ensure all available data is read in case we were interrupted.
         * Never read beyond the remaining space in cdc_read_buffer.
         */
        if (bytes_read < (int)CDC_BUFFER_SIZE)
        {
            const uint32_t remaining = (uint32_t)(CDC_BUFFER_SIZE - (size_t)bytes_read);
            int bytes_read_2 = usb_serial_jtag_read_bytes(&cdc_read_buffer[bytes_read], remaining, pdMS_TO_TICKS(10));
            if (bytes_read_2 > 0)
            {
                if (bytes_read + bytes_read_2 > (int)CDC_BUFFER_SIZE)
                {
                    bytes_read_2 = (int)CDC_BUFFER_SIZE - bytes_read;
                }
                bytes_read += bytes_read_2;
            }
        }

        /* Check if this is a config packet */
        if (bytes_read == 64)
        {
            if (parse_config_packet(cdc_read_buffer, bytes_read, &new_config))
            {
                if (new_config.baud_rate == 0)
                {
                    /* Query mode: send current config */
                    ESP_LOGI(TAG, "CDC: config query received, sending current config");
                    send_current_config();
                    continue;
                }
                ESP_LOGI(TAG, "CDC: VALID CONFIG: baud=%lu, TX=%u, RX=%u, RESET=%u, CONTROL=%u, LED=%u", new_config.baud_rate, new_config.tx_gpio, new_config.rx_gpio, new_config.reset_gpio, new_config.control_gpio, new_config.led_gpio);
                uart_gateway_configure(&new_config);
                continue;
            }

            if (parse_control_packet(cdc_read_buffer, bytes_read))
            {
                ESP_LOGI(TAG, "CDC: received control packet, processed accordingly");
                continue;
            }
        }

        led_signal_uart_activity();
        /* Non-config packets go to UART */
        uart_gateway_queue_cdc_data(cdc_read_buffer, bytes_read);
    }
}

/* Task: Write queued CDC data to UART */
static void uart_write_task(void *pvParameters)
{
    int bytes_to_send;

    ESP_LOGI(TAG, "UART write task started");

    /* Wait a bit for system to fully initialize */
    vTaskDelay(200 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "UART write task ready");

    while (1)
    {
        if (!uart_gateway_is_ready())
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        bytes_to_send = uart_gateway_receive_cdc_queue(uart_write_buffer, UART_BUFFER_SIZE);

        if (bytes_to_send <= 0)
        {
            continue;
        }

        ESP_LOGI(TAG, "UART write: sending %d bytes to UART", bytes_to_send);
        led_signal_uart_activity();
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
    vTaskDelay(200 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "UART read task ready");

    while (1)
    {
        if (!uart_gateway_is_ready())
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        bytes_received = uart_gateway_receive(uart_read_buffer, UART_BUFFER_SIZE);

        if (bytes_received <= 0)
        {
            continue;
        }
        ESP_LOGI(TAG, "UART read: received %d bytes", bytes_received);
        led_signal_uart_activity();

        esp_err_t ret = uart_gateway_queue_uart_data(uart_read_buffer, bytes_received);
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "UART read: queued %d bytes to CDC", bytes_received);
        }
        else
        {
            ESP_LOGE(TAG, "UART read: failed to queue %d bytes", bytes_received);
        }
    }
}

/* Task: Write queued UART data to CDC */
static void cdc_write_task(void *pvParameters)
{
    int bytes_to_send;

    ESP_LOGI(TAG, "CDC write task started");

    /* Wait a bit for system to fully initialize */
    vTaskDelay(200 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "CDC write task ready");

    while (1)
    {
        /* First, check for config responses */
        if (config_response_queue != NULL && xQueueReceive(config_response_queue, cdc_write_buffer, 0) == pdPASS)
        {
            ESP_LOGI(TAG, "CDC write: sending config response packet");
            int written = usb_serial_jtag_write_bytes(cdc_write_buffer, UART_CONFIG_PACKET_SIZE, pdMS_TO_TICKS(100));
            if (written == UART_CONFIG_PACKET_SIZE)
            {
                ESP_LOGI(TAG, "CDC write: config response sent successfully");
            }
            else
            {
                ESP_LOGW(TAG, "CDC write: config response incomplete %d/%u", written, UART_CONFIG_PACKET_SIZE);
            }
            taskYIELD();
            continue;
        }

        /* Then check for UART->CDC data */
        bytes_to_send = uart_gateway_receive_uart_queue(cdc_write_buffer, CDC_BUFFER_SIZE);

        if (bytes_to_send <= 0)
        {
            continue;
        }

        ESP_LOGI(TAG, "CDC write: sending %d bytes to CDC", bytes_to_send);
        led_signal_uart_activity();
        int written = usb_serial_jtag_write_bytes(cdc_write_buffer, bytes_to_send, pdMS_TO_TICKS(1000));
        if (written < bytes_to_send)
        {
            ESP_LOGW(TAG, "CDC write: incomplete write %d/%d", written, bytes_to_send);
        }
        else
        {
            ESP_LOGI(TAG, "CDC write: sent %d bytes successfully", bytes_to_send);
        }
        taskYIELD();
    }
}

void uart_gateway_start(void)
{
    /* Create config response queue */
    if (config_response_queue == NULL)
    {
        config_response_queue = xQueueCreate(4, UART_CONFIG_PACKET_SIZE);
        if (config_response_queue == NULL)
        {
            ESP_LOGE(TAG, "Failed to create config response queue");
            return;
        }
        ESP_LOGI(TAG, "Config response queue created");
    }

    /* Initialize USB CDC (enabled by default on ESP32-C3) */
    usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    esp_err_t usb_res = usb_serial_jtag_driver_install(&usb_cfg);
    if (usb_res != ESP_OK)
    {
        ESP_LOGE(TAG, "USB CDC init failed: %s", esp_err_to_name(usb_res));
        return;
    }
    ESP_LOGI(TAG, "USB CDC initialized");

    /* Register callback to persist config changes */
    uart_gateway_register_callback(on_config_changed);

    ESP_LOGI(TAG, "Creating relay tasks...");

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