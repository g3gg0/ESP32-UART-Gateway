#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "canonef.h"
#include "uart_gateway.h"

#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "soc/gpio_sig_map.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define CANONEF_RMT_RESOLUTION_HZ 5000000U
#define CANONEF_CLOCK_MIN_HZ 10000U
#define CANONEF_CLOCK_MAX_HZ 500000U
#define CANONEF_RX_IDLE_NS 3000000U
#define CANONEF_RX_GLITCH_NS 100U
#define CANONEF_TRANSFER_TIMEOUT_MS 20U
#define CANONEF_RMT_SYMBOLS 48U
#define CANONEF_CLOCK_PULSES_PER_BYTE 8U
#define CANONEF_ACK_START_TIMEOUT_US 50U
#define CANONEF_ACK_MIN_DURATION_US 4U
#define CANONEF_ACK_MAX_DURATION_US 3000U
#define CANONEF_MAX_XFER_BYTES 4096U
#define CANONEF_RESET_HOLD_US 800U

typedef struct
{
    bool active;
    bool spi_initialized;
    canonef_config_packet_t config;
    spi_device_handle_t spi;
    rmt_channel_handle_t lclk_rx;
    bool lclk_gpio_isr_installed;
    volatile bool lclk_ack_started;
    volatile bool lclk_ack_released;
    volatile uint32_t lclk_ack_duration_us;
    volatile int64_t lclk_ack_start_us;
    SemaphoreHandle_t rx_done;
    SemaphoreHandle_t mutex;
    volatile uint8_t lclk_pulse_count;
    volatile size_t lclk_symbol_count;
    rmt_symbol_word_t lclk_symbols[CANONEF_RMT_SYMBOLS];
} canonef_context_t;

static canonef_context_t canonef;

static bool IRAM_ATTR canonef_lclk_rx_done(rmt_channel_handle_t channel,
                                 const rmt_rx_done_event_data_t *event_data,
                                 void *user_data)
{
    (void)channel;
    canonef_context_t *context = (canonef_context_t *)user_data;
    BaseType_t task_woken = pdFALSE;

    context->lclk_symbol_count = event_data->num_symbols;
    xSemaphoreGiveFromISR(context->rx_done, &task_woken);
    return task_woken == pdTRUE;
}

static void IRAM_ATTR canonef_lclk_gpio_isr(void *arg)
{
    canonef_context_t *context = (canonef_context_t *)arg;
    BaseType_t task_woken = pdFALSE;
    bool normalized_low = gpio_get_level((gpio_num_t)context->config.lclk_in_gpio) !=
                          ((context->config.inversion_mask & CANONEF_INVERT_LCLK_IN) != 0);

    if (!context->lclk_ack_started && normalized_low)
    {
        if (context->lclk_pulse_count < CANONEF_CLOCK_PULSES_PER_BYTE + 1U)
        {
            context->lclk_pulse_count++;
        }
        if (context->lclk_pulse_count == CANONEF_CLOCK_PULSES_PER_BYTE + 1U)
        {
            context->lclk_ack_started = true;
            context->lclk_ack_start_us = esp_timer_get_time();
            xSemaphoreGiveFromISR(context->rx_done, &task_woken);
        }
    }
    else if (context->lclk_ack_started && !normalized_low && !context->lclk_ack_released)
    {
        int64_t duration_us = esp_timer_get_time() - context->lclk_ack_start_us;
        context->lclk_ack_duration_us = duration_us > UINT32_MAX ? UINT32_MAX : (uint32_t)duration_us;
        context->lclk_ack_released = true;
        xSemaphoreGiveFromISR(context->rx_done, &task_woken);
    }

    if (task_woken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t canonef_queue_packet(uint16_t packet_type, const void *payload, size_t payload_len)
{
    size_t packet_size = UART_PACKET_HEADER_SIZE + payload_len;
    if (packet_size > UINT16_MAX)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    uart_packet_header_t *packet = (uart_packet_header_t *)malloc(packet_size);
    if (packet == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    packet->length = (uint16_t)packet_size;
    packet->type = packet_type;
    if (payload_len > 0 && payload != NULL)
    {
        memcpy(PTR_BEHIND(packet), payload, payload_len);
    }

    return queue_packet(packet);
}

static esp_err_t canonef_queue_config_response(canonef_status_t status,
                                               const canonef_config_packet_t *config)
{
    canonef_config_response_t response = {
        .status = (uint8_t)status,
    };
    if (config != NULL)
    {
        response.config = *config;
    }

    return canonef_queue_packet(UART_PACKET_TYPE_LENS_CONFIG, &response, sizeof(response));
}

static bool canonef_gpio_config_valid(const canonef_config_packet_t *config)
{
    if (!GPIO_IS_VALID_OUTPUT_GPIO(config->dcl_gpio) ||
        !GPIO_IS_VALID_GPIO(config->dlc_gpio) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->lclk_out_gpio) ||
        !GPIO_IS_VALID_GPIO(config->lclk_in_gpio))
    {
        return false;
    }

    const uint8_t gpios[] = {
        config->dcl_gpio,
        config->dlc_gpio,
        config->lclk_out_gpio,
        config->lclk_in_gpio,
    };
    for (size_t left = 0; left < sizeof(gpios); left++)
    {
        for (size_t right = left + 1; right < sizeof(gpios); right++)
        {
            if (gpios[left] == gpios[right])
            {
                return false;
            }
        }
    }

    return true;
}

static void canonef_release_gpio(uint8_t gpio)
{
    if (GPIO_IS_VALID_GPIO(gpio))
    {
        (void)gpio_set_direction((gpio_num_t)gpio, GPIO_MODE_INPUT);
        (void)gpio_set_pull_mode((gpio_num_t)gpio, GPIO_FLOATING);
    }
}

void canonef_stop_session(void)
{
    canonef.active = false;

    if (canonef.lclk_gpio_isr_installed)
    {
        (void)gpio_isr_handler_remove((gpio_num_t)canonef.config.lclk_in_gpio);
        canonef.lclk_gpio_isr_installed = false;
    }

    if (canonef.lclk_rx != NULL)
    {
        (void)rmt_disable(canonef.lclk_rx);
        (void)rmt_del_channel(canonef.lclk_rx);
        canonef.lclk_rx = NULL;
    }
    if (canonef.spi_initialized)
    {
        if (canonef.spi != NULL)
        {
            (void)spi_bus_remove_device(canonef.spi);
            canonef.spi = NULL;
        }
        (void)spi_bus_free(SPI2_HOST);
        canonef.spi_initialized = false;
    }

    canonef_release_gpio(canonef.config.dcl_gpio);
    canonef_release_gpio(canonef.config.dlc_gpio);
    canonef_release_gpio(canonef.config.lclk_out_gpio);
    canonef_release_gpio(canonef.config.lclk_in_gpio);
    canonef.lclk_symbol_count = 0;
    canonef.lclk_pulse_count = 0;
    canonef.lclk_ack_started = false;
    canonef.lclk_ack_released = false;
    canonef.lclk_ack_duration_us = 0;
    canonef.lclk_ack_start_us = 0;
    memset(&canonef.config, 0, sizeof(canonef.config));
}

static esp_err_t canonef_start_session(const canonef_config_packet_t *config)
{
    if (config == NULL)
    {
        send_message("Canon EF config invalid: null config");
        return ESP_ERR_INVALID_ARG;
    }
    if (!canonef_gpio_config_valid(config))
    {
        send_message("Canon EF config invalid: GPIOs DCL=%u DLC=%u LCLKout=%u LCLKin=%u",
                     config->dcl_gpio, config->dlc_gpio, config->lclk_out_gpio, config->lclk_in_gpio);
        return ESP_ERR_INVALID_ARG;
    }
    if (config->clock_hz < CANONEF_CLOCK_MIN_HZ || config->clock_hz > CANONEF_CLOCK_MAX_HZ)
    {
        send_message("Canon EF config invalid: clock=%lu Hz", config->clock_hz);
        return ESP_ERR_INVALID_ARG;
    }
    if ((config->inversion_mask & 0xF0U) != 0 ||
        (config->flags & ~(CANONEF_CONFIG_FLAG_ACK_DURATION | CANONEF_CONFIG_FLAG_IGNORE_ACK)) != 0)
    {
        send_message("Canon EF config invalid: inversion=0x%02X flags=0x%02X",
                     config->inversion_mask, config->flags);
        return ESP_ERR_INVALID_ARG;
    }

    canonef_stop_session();
    canonef.config = *config;

    if (canonef.rx_done == NULL)
    {
        canonef.rx_done = xSemaphoreCreateBinary();
    }
    if (canonef.mutex == NULL)
    {
        canonef.mutex = xSemaphoreCreateMutex();
    }
    if (canonef.rx_done == NULL || canonef.mutex == NULL)
    {
        canonef_stop_session();
        return ESP_ERR_NO_MEM;
    }

    esp_err_t error;
    bool capture_ack = (config->flags & CANONEF_CONFIG_FLAG_ACK_DURATION) != 0;
    if (capture_ack)
    {
        rmt_rx_channel_config_t lclk_rx_config = {
            .gpio_num = (gpio_num_t)config->lclk_in_gpio,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = CANONEF_RMT_RESOLUTION_HZ,
            .mem_block_symbols = CANONEF_RMT_SYMBOLS,
            .flags.invert_in = (config->inversion_mask & CANONEF_INVERT_LCLK_IN) != 0,
        };
        error = rmt_new_rx_channel(&lclk_rx_config, &canonef.lclk_rx);
        if (error != ESP_OK)
        {
            send_message("Canon EF RMT RX setup failed: %s", esp_err_to_name(error));
            canonef_stop_session();
            return error;
        }

        rmt_rx_event_callbacks_t rx_callbacks = {
            .on_recv_done = canonef_lclk_rx_done,
        };
        error = rmt_rx_register_event_callbacks(canonef.lclk_rx, &rx_callbacks, &canonef);
        if (error == ESP_OK)
        {
            error = rmt_enable(canonef.lclk_rx);
        }
        if (error != ESP_OK)
        {
            send_message("Canon EF RMT callback/enable failed: %s", esp_err_to_name(error));
            canonef_stop_session();
            return error;
        }
    }
    else
    {
        gpio_config_t lclk_gpio_config = {
            .pin_bit_mask = 1ULL << config->lclk_in_gpio,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE,
        };
        error = gpio_config(&lclk_gpio_config);
        if (error == ESP_OK)
        {
            error = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
            if (error == ESP_ERR_INVALID_STATE)
            {
                error = ESP_OK;
            }
        }
        if (error == ESP_OK)
        {
            error = gpio_isr_handler_add((gpio_num_t)config->lclk_in_gpio,
                                         canonef_lclk_gpio_isr, &canonef);
            canonef.lclk_gpio_isr_installed = (error == ESP_OK);
        }
        if (error != ESP_OK)
        {
            send_message("Canon EF GPIO ISR setup failed: %s", esp_err_to_name(error));
            canonef_stop_session();
            return error;
        }
    }

    spi_bus_config_t spi_bus_config = {
        .mosi_io_num = config->dcl_gpio,
        .miso_io_num = config->dlc_gpio,
        .sclk_io_num = config->lclk_out_gpio,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = CANONEF_MAX_XFER_BYTES,
    };
    gpio_config_t dlc_gpio_config = {
        .pin_bit_mask = 1ULL << config->dlc_gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    error = gpio_config(&dlc_gpio_config);
    if (error != ESP_OK)
    {
        send_message("Canon EF DLC input setup failed: %s", esp_err_to_name(error));
        canonef_stop_session();
        return error;
    }
    error = spi_bus_initialize(SPI2_HOST, &spi_bus_config, SPI_DMA_DISABLED);
    if (error != ESP_OK)
    {
        send_message("Canon EF SPI bus setup failed: %s", esp_err_to_name(error));
        canonef_stop_session();
        return error;
    }
    canonef.spi_initialized = true;

    spi_device_interface_config_t spi_config = {
        .clock_speed_hz = (int)config->clock_hz,
        /* Inverted LCLK uses CPOL=0/CPHA=1; direct LCLK uses CPOL=1/CPHA=1. */
        .mode = (config->inversion_mask & CANONEF_INVERT_LCLK_OUT) ? 1 : 3,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    error = spi_bus_add_device(SPI2_HOST, &spi_config, &canonef.spi);
    if (error != ESP_OK)
    {
        send_message("Canon EF SPI device setup failed: %s", esp_err_to_name(error));
        canonef_stop_session();
        return error;
    }
    esp_rom_gpio_connect_out_signal(config->dcl_gpio, FSPID_OUT_IDX,
                                    (config->inversion_mask & CANONEF_INVERT_DCL) != 0, 0);
    esp_rom_gpio_connect_out_signal(config->lclk_out_gpio, FSPICLK_OUT_IDX,
                                    false, 0);
    canonef.active = true;
    return ESP_OK;
}

static bool canonef_find_ack_duration(uint16_t *ack_us)
{
    size_t low_pulse_count = 0;
    size_t symbol_count = canonef.lclk_symbol_count;
    if (symbol_count > CANONEF_RMT_SYMBOLS)
    {
        symbol_count = CANONEF_RMT_SYMBOLS;
    }

    for (size_t symbol_index = 0; symbol_index < symbol_count; symbol_index++)
    {
        const rmt_symbol_word_t *symbol = &canonef.lclk_symbols[symbol_index];
        const uint32_t levels[] = {symbol->level0, symbol->level1};
        const uint32_t durations[] = {symbol->duration0, symbol->duration1};
        for (size_t part = 0; part < 2; part++)
        {
            if (durations[part] == 0 || levels[part] != 0)
            {
                continue;
            }

            low_pulse_count++;
            if (low_pulse_count == CANONEF_CLOCK_PULSES_PER_BYTE + 1U)
            {
                uint32_t duration_us = (durations[part] * 1000000U +
                                        CANONEF_RMT_RESOLUTION_HZ - 1U) /
                                       CANONEF_RMT_RESOLUTION_HZ;
                *ack_us = (duration_us > UINT16_MAX) ? UINT16_MAX : (uint16_t)duration_us;
                return true;
            }
        }
    }

    return false;
}

static esp_err_t canonef_reset_lclk_rx(void)
{
    if (canonef.lclk_rx == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t error = rmt_disable(canonef.lclk_rx);
    if (error != ESP_OK)
    {
        return error;
    }

    error = rmt_enable(canonef.lclk_rx);
    while (xSemaphoreTake(canonef.rx_done, 0) == pdTRUE)
    {
    }
    return error;
}

static esp_err_t canonef_transfer_byte(uint8_t tx_byte, uint8_t *rx_byte, uint16_t *ack_us)
{
    uint8_t spi_tx = tx_byte;
    uint8_t spi_rx = 0xFF;
    spi_transaction_t spi_transaction = {
        .length = 8,
        .tx_buffer = &spi_tx,
        .rx_buffer = &spi_rx,
    };
    while (xSemaphoreTake(canonef.rx_done, 0) == pdTRUE)
    {
    }
    bool capture_ack = (canonef.config.flags & CANONEF_CONFIG_FLAG_ACK_DURATION) != 0;
    esp_err_t error = ESP_OK;
    if (capture_ack)
    {
        canonef.lclk_symbol_count = 0;
        memset(canonef.lclk_symbols, 0, sizeof(canonef.lclk_symbols));

        rmt_receive_config_t receive_config = {
            .signal_range_min_ns = CANONEF_RX_GLITCH_NS,
            .signal_range_max_ns = CANONEF_RX_IDLE_NS,
        };
        error = rmt_receive(canonef.lclk_rx, canonef.lclk_symbols,
                            sizeof(canonef.lclk_symbols), &receive_config);
        if (error == ESP_ERR_INVALID_STATE)
        {
            error = canonef_reset_lclk_rx();
            if (error == ESP_OK)
            {
                error = rmt_receive(canonef.lclk_rx, canonef.lclk_symbols,
                                    sizeof(canonef.lclk_symbols), &receive_config);
            }
        }
    }
    else
    {
        canonef.lclk_pulse_count = 0;
        canonef.lclk_ack_started = false;
        canonef.lclk_ack_released = false;
        canonef.lclk_ack_duration_us = 0;
        canonef.lclk_ack_start_us = 0;
    }
    if (error != ESP_OK)
    {
        return error;
    }

    error = spi_device_transmit(canonef.spi, &spi_transaction);
    if (error != ESP_OK)
    {
        if (capture_ack)
        {
            (void)canonef_reset_lclk_rx();
        }
        return error;
    }

    if (canonef.config.inversion_mask & CANONEF_INVERT_DLC)
    {
        spi_rx ^= 0xFFU;
    }
    *rx_byte = spi_rx;

    if (!capture_ack)
    {
        esp_rom_delay_us(CANONEF_ACK_START_TIMEOUT_US);
        if (!canonef.lclk_ack_started)
        {
            *ack_us = 0;
            canonef.lclk_pulse_count = 0;
            if ((canonef.config.flags & CANONEF_CONFIG_FLAG_IGNORE_ACK) != 0)
            {
                return ESP_OK;
            }
            return ESP_ERR_TIMEOUT;
        }

        esp_rom_delay_us(CANONEF_ACK_MAX_DURATION_US);
        if (!canonef.lclk_ack_released)
        {
            int64_t elapsed_us = esp_timer_get_time() - canonef.lclk_ack_start_us;
            *ack_us = elapsed_us > UINT16_MAX ? UINT16_MAX : (uint16_t)elapsed_us;
            canonef.lclk_pulse_count = 0;
            if ((canonef.config.flags & CANONEF_CONFIG_FLAG_IGNORE_ACK) != 0)
            {
                return ESP_OK;
            }
            return ESP_ERR_INVALID_STATE;
        }

        if (canonef.lclk_ack_duration_us < CANONEF_ACK_MIN_DURATION_US ||
            canonef.lclk_ack_duration_us > CANONEF_ACK_MAX_DURATION_US)
        {
            if ((canonef.config.flags & CANONEF_CONFIG_FLAG_IGNORE_ACK) != 0)
            {
                return ESP_OK;
            }
            return ESP_ERR_INVALID_STATE;
        }
        return ESP_OK;
    }

    if (xSemaphoreTake(canonef.rx_done, pdMS_TO_TICKS(CANONEF_TRANSFER_TIMEOUT_MS)) != pdTRUE)
    {
        (void)canonef_reset_lclk_rx();
        if ((canonef.config.flags & CANONEF_CONFIG_FLAG_IGNORE_ACK) != 0)
        {
            return ESP_OK;
        }
        return ESP_ERR_TIMEOUT;
    }
    if (!capture_ack)
    {
        return ESP_OK;
    }
    if (!canonef_find_ack_duration(ack_us))
    {
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

esp_err_t canonef_handle_config_packet(const uint8_t *payload, size_t payload_len)
{
    if (payload == NULL || payload_len != sizeof(canonef_config_packet_t))
    {
        (void)canonef_queue_config_response(CANONEF_STATUS_BAD_LEN, NULL);
        return ESP_ERR_INVALID_SIZE;
    }

    canonef_config_packet_t config;
    memcpy(&config, payload, sizeof(config));
    esp_err_t error = canonef_start_session(&config);
    if (error == ESP_OK)
    {
        return canonef_queue_config_response(CANONEF_STATUS_OK, &canonef.config);
    }

    canonef_status_t status = CANONEF_STATUS_INTERNAL;
    if (error == ESP_ERR_INVALID_ARG)
    {
        status = CANONEF_STATUS_INVALID_ARG;
    }
    else if (error == ESP_ERR_NO_MEM)
    {
        status = CANONEF_STATUS_NO_MEMORY;
    }
    (void)canonef_queue_config_response(status, &config);
    return error;
}

esp_err_t canonef_handle_xfer_packet(const uint8_t *payload, size_t payload_len)
{
    if (!canonef.active)
    {
        uint8_t status = CANONEF_STATUS_NOT_INITIALIZED;
        (void)canonef_queue_packet(UART_PACKET_TYPE_LENS_XFER, &status, sizeof(status));
        return ESP_ERR_INVALID_STATE;
    }
    if (payload == NULL || payload_len == 0 || payload_len > CANONEF_MAX_XFER_BYTES)
    {
        uint8_t status = CANONEF_STATUS_BAD_LEN;
        (void)canonef_queue_packet(UART_PACKET_TYPE_LENS_XFER, &status, sizeof(status));
        return ESP_ERR_INVALID_SIZE;
    }

    bool include_ack = (canonef.config.flags & CANONEF_CONFIG_FLAG_ACK_DURATION) != 0;
    bool ignore_ack = (canonef.config.flags & CANONEF_CONFIG_FLAG_IGNORE_ACK) != 0;
    size_t record_size = include_ack ? 3U : 1U;
    size_t response_len = 1U + payload_len * record_size;
    size_t response_records = 0;
    uint8_t *response = (uint8_t *)malloc(response_len);
    if (response == NULL)
    {
        uint8_t status = CANONEF_STATUS_NO_MEMORY;
        (void)canonef_queue_packet(UART_PACKET_TYPE_LENS_XFER, &status, sizeof(status));
        return ESP_ERR_NO_MEM;
    }

    memset(response, 0xFF, response_len);
    response[0] = CANONEF_STATUS_OK;
    esp_err_t result = ESP_OK;

    if (xSemaphoreTake(canonef.mutex, pdMS_TO_TICKS(CANONEF_TRANSFER_TIMEOUT_MS)) != pdTRUE)
    {
        response[0] = CANONEF_STATUS_TRANSFER_FAILED;
        result = ESP_ERR_TIMEOUT;
    }
    else
    {
        for (size_t byte_index = 0; byte_index < payload_len; byte_index++)
        {
            uint8_t rx_byte = 0xFF;
            uint16_t ack_us = UINT16_MAX;
            esp_err_t error = canonef_transfer_byte(payload[byte_index], &rx_byte, &ack_us);
            size_t output_index = 1U + byte_index * record_size;
            if (include_ack)
            {
                response[output_index] = (uint8_t)(ack_us & 0xFFU);
                response[output_index + 1U] = (uint8_t)(ack_us >> 8);
                response[output_index + 2U] = rx_byte;
            }
            else
            {
                response[output_index] = rx_byte;
            }
            response_records++;

            if (error != ESP_OK)
            {
                if (ignore_ack && (error == ESP_ERR_NOT_FOUND ||
                                   error == ESP_ERR_TIMEOUT ||
                                   error == ESP_ERR_INVALID_STATE))
                {
                    continue;
                }
                  response[0] = (error == ESP_ERR_NOT_FOUND || error == ESP_ERR_TIMEOUT)
                              ? CANONEF_STATUS_ACK_TIMEOUT
                              : (error == ESP_ERR_INVALID_STATE
                                  ? CANONEF_STATUS_ACK_TOO_LONG
                                  : CANONEF_STATUS_TRANSFER_FAILED);
                result = error;
                break;
            }
        }
        xSemaphoreGive(canonef.mutex);
    }

    response_len = 1U + response_records * record_size;

    esp_err_t queue_error = canonef_queue_packet(UART_PACKET_TYPE_LENS_XFER,
                                                  response, response_len);
    free(response);
    return queue_error == ESP_OK ? result : queue_error;
}

esp_err_t canonef_handle_reset_packet(const uint8_t *payload, size_t payload_len)
{
    if ((payload != NULL && payload_len != 0) || (payload == NULL && payload_len != 0))
    {
        (void)canonef_queue_packet(UART_PACKET_TYPE_LENS_RESET,
                                    &(uint8_t){CANONEF_STATUS_BAD_LEN}, 1);
        return ESP_ERR_INVALID_SIZE;
    }
    if (!canonef.active || canonef.spi == NULL)
    {
        (void)canonef_queue_packet(UART_PACKET_TYPE_LENS_RESET,
                                    &(uint8_t){CANONEF_STATUS_NOT_INITIALIZED}, 1);
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(canonef.mutex, pdMS_TO_TICKS(CANONEF_TRANSFER_TIMEOUT_MS)) != pdTRUE)
    {
        (void)canonef_queue_packet(UART_PACKET_TYPE_LENS_RESET,
                                    &(uint8_t){CANONEF_STATUS_TRANSFER_FAILED}, 1);
        return ESP_ERR_TIMEOUT;
    }

    const uint8_t lclk_gpio = canonef.config.lclk_out_gpio;
    const bool inverted = (canonef.config.inversion_mask & CANONEF_INVERT_LCLK_OUT) != 0;
    esp_err_t error = gpio_intr_disable((gpio_num_t)canonef.config.lclk_in_gpio);
    if (error == ESP_OK)
    {
        /* Disconnect SPI and route the pin to the GPIO output path first. */
        esp_rom_gpio_connect_out_signal(lclk_gpio, SIG_GPIO_OUT_IDX, false, 0);
        error = gpio_set_direction((gpio_num_t)lclk_gpio, GPIO_MODE_OUTPUT);
    }
    if (error == ESP_OK)
    {
        error = gpio_set_level((gpio_num_t)lclk_gpio, inverted ? 1 : 0);
    }
    if (error == ESP_OK)
    {
        esp_rom_delay_us(CANONEF_RESET_HOLD_US);
        /* Passive output releases the physical active-low LCLK line. */
        error = gpio_set_level((gpio_num_t)lclk_gpio, inverted ? 0 : 1);
    }
    if (error == ESP_OK)
    {
        /* Return ownership to SPI only after the GPIO is passive. */
        esp_rom_gpio_connect_out_signal(lclk_gpio, FSPICLK_OUT_IDX, false, 0);
        (void)gpio_intr_enable((gpio_num_t)canonef.config.lclk_in_gpio);
    }

    xSemaphoreGive(canonef.mutex);
    uint8_t status = (error == ESP_OK) ? CANONEF_STATUS_OK : CANONEF_STATUS_TRANSFER_FAILED;
    send_message("Canon EF lens reset: LCLK GPIO=%u status=%s", lclk_gpio, esp_err_to_name(error));
    (void)canonef_queue_packet(UART_PACKET_TYPE_LENS_RESET, &status, sizeof(status));
    return error;
}