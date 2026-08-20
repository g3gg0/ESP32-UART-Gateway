#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "canonef.h"
#include "uart_gateway.h"

#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_types.h"   // Include RMT driver types
#include "../src/rmt_private.h" // Include private RMT definitions
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
#define CANONEF_RX_IDLE_NS_ACK_START 50000U
#define CANONEF_RX_IDLE_NS_ACK_DURATION 3000000U
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
    SemaphoreHandle_t rx_done;
    SemaphoreHandle_t mutex;

    canonef_config_packet_t config;
    spi_device_handle_t spi;
    rmt_channel_handle_t rmt_rx_channel;
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

    xSemaphoreGiveFromISR(context->rx_done, &task_woken);
    return task_woken == pdTRUE;
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

    if (canonef.rmt_rx_channel != NULL)
    {
        (void)rmt_disable(canonef.rmt_rx_channel);
        (void)rmt_del_channel(canonef.rmt_rx_channel);
        canonef.rmt_rx_channel = NULL;
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

    if (canonef.rx_done)
    {
        vSemaphoreDelete(canonef.rx_done);
        canonef.rx_done = NULL;
    }
    if (canonef.mutex)
    {
        vSemaphoreDelete(canonef.mutex);
        canonef.mutex = NULL;
    }

    canonef_release_gpio(canonef.config.dcl_gpio);
    canonef_release_gpio(canonef.config.dlc_gpio);
    canonef_release_gpio(canonef.config.lclk_out_gpio);
    canonef_release_gpio(canonef.config.lclk_in_gpio);

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
        send_message("Canon EF config invalid: GPIOs DCL=%u DLC=%u LCLKout=%u LCLKin=%u", config->dcl_gpio, config->dlc_gpio, config->lclk_out_gpio, config->lclk_in_gpio);
        return ESP_ERR_INVALID_ARG;
    }
    if (config->clock_hz < CANONEF_CLOCK_MIN_HZ || config->clock_hz > CANONEF_CLOCK_MAX_HZ)
    {
        send_message("Canon EF config invalid: clock=%lu Hz", config->clock_hz);
        return ESP_ERR_INVALID_ARG;
    }
    if ((config->inversion_mask & 0xF0U) != 0 || (config->flags & ~(CANONEF_CONFIG_FLAG_ACK_DURATION | CANONEF_CONFIG_FLAG_IGNORE_ACK)) != 0)
    {
        send_message("Canon EF config invalid: inversion=0x%02X flags=0x%02X", config->inversion_mask, config->flags);
        return ESP_ERR_INVALID_ARG;
    }

    canonef_stop_session();
    canonef.config = *config;

    canonef.rx_done = xSemaphoreCreateBinary();
    canonef.mutex = xSemaphoreCreateMutex();

    if (canonef.rx_done == NULL || canonef.mutex == NULL)
    {
        canonef_stop_session();
        return ESP_ERR_NO_MEM;
    }

    esp_err_t error;

    /* RMT init */
    rmt_rx_channel_config_t lclk_rx_config = {
        .gpio_num = (gpio_num_t)config->lclk_in_gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = CANONEF_RMT_RESOLUTION_HZ,
        .mem_block_symbols = CANONEF_RMT_SYMBOLS,
        .flags.invert_in = (config->inversion_mask & CANONEF_INVERT_LCLK_IN) != 0,
    };
    error = rmt_new_rx_channel(&lclk_rx_config, &canonef.rmt_rx_channel);
    if (error != ESP_OK)
    {
        send_message("Canon EF RMT RX setup failed: %s", esp_err_to_name(error));
        canonef_stop_session();
        return error;
    }

    rmt_rx_event_callbacks_t rx_callbacks = {
        .on_recv_done = canonef_lclk_rx_done,
    };
    error = rmt_rx_register_event_callbacks(canonef.rmt_rx_channel, &rx_callbacks, &canonef);
    if (error == ESP_OK)
    {
        error = rmt_enable(canonef.rmt_rx_channel);
    }
    if (error != ESP_OK)
    {
        send_message("Canon EF RMT callback/enable failed: %s", esp_err_to_name(error));
        canonef_stop_session();
        return error;
    }

    /* SPI init */
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
        .mode = 3,
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

    /* configure inversion at a single place */
    esp_rom_gpio_connect_in_signal(config->dlc_gpio, FSPID_IN_IDX, (config->inversion_mask & CANONEF_INVERT_DLC) != 0);
    esp_rom_gpio_connect_out_signal(config->dcl_gpio, FSPID_OUT_IDX, (config->inversion_mask & CANONEF_INVERT_DCL) != 0, 0);
    esp_rom_gpio_connect_out_signal(config->lclk_out_gpio, FSPICLK_OUT_IDX, (config->inversion_mask & CANONEF_INVERT_LCLK_OUT) != 0, 0);

    canonef.active = true;
    send_message("Canon EF: Initialized");

    return ESP_OK;
}

static bool canonef_find_ack_duration(uint8_t *ack_delay_us, uint16_t *ack_duration_us)
{
    const volatile rmt_symbol_word_t *symbol_before = &canonef.lclk_symbols[7];
    const volatile rmt_symbol_word_t *symbol_after = &canonef.lclk_symbols[8];
    uint32_t ack_delay = symbol_before->duration1;
    uint32_t ack_duration = symbol_after->duration0;

    *ack_delay_us = 0;
    *ack_duration_us = 0;

    /* if the ack came, the inactive phase of the clock (high) will have some duration, we call that ack delay, the
    time it took until the ack came */
    if (ack_duration > 0)
    {
        uint32_t delay_us = (ack_delay * 1000000U + CANONEF_RMT_RESOLUTION_HZ - 1U) / CANONEF_RMT_RESOLUTION_HZ;
        *ack_delay_us = (delay_us > UINT8_MAX) ? UINT8_MAX : (uint8_t)delay_us;

        uint32_t duration_us = (ack_duration * 1000000U + CANONEF_RMT_RESOLUTION_HZ - 1U) / CANONEF_RMT_RESOLUTION_HZ;
        *ack_duration_us = (duration_us > UINT16_MAX) ? UINT16_MAX : (uint16_t)duration_us;
        return true;
    }

    return false;
}

static esp_err_t canonef_reset_rmt(void)
{
    if (canonef.rmt_rx_channel == NULL)
    {
        send_message("Canon EF lens: Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* may fail if never enabled, so thats okay to ignore */
    rmt_disable(canonef.rmt_rx_channel);

    esp_err_t error = rmt_enable(canonef.rmt_rx_channel);
    if (error != ESP_OK)
    {
        send_message("Canon EF lens: rmt_enable() failed with error=%s", esp_err_to_name(error));
        return error;
    }

    return error;
}

static esp_err_t canonef_transfer_byte(uint8_t tx_byte, uint8_t *rx_byte, uint8_t *ack_delay_us, uint16_t *ack_duration_us)
{
    uint8_t spi_tx = tx_byte;
    uint8_t spi_rx = 0xFF;
    spi_transaction_t spi_transaction = {
        .length = 8,
        .tx_buffer = &spi_tx,
        .rx_buffer = &spi_rx,
    };

    bool capture_ack = (canonef.config.flags & CANONEF_CONFIG_FLAG_ACK_DURATION) != 0;
    bool lclk_in_inv = (canonef.config.inversion_mask & CANONEF_INVERT_LCLK_IN) != 0;
    esp_err_t error = ESP_OK;

    /* take it if we can */
    xSemaphoreTake(canonef.rx_done, 0);

    /* two possible modes
        normal mode: times out after an ACK has been seen (no matter how long it is) or the duration we want to wait for an ACK to come
        measurement mode: times out after the ack was there AND the maximum time we want to wait for the ACK in any case.

        Problem is, the RMT only has a single timeout, no matter at which phase. even when we see an ACK, we still
        have to wait the maximum time that an ACK could be held active by the lens.
        In normal mode so just wait for ACK to be seen which should happen after 14µs.
        In measurement mode we wait for up to 3ms. This means in measurement mode EVERY byte will take 3ms, even if the ACK was only 14µs long.
        We need to wait for the full duration to measure it.
     */
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = CANONEF_RX_GLITCH_NS,
        .signal_range_max_ns = capture_ack ? CANONEF_RX_IDLE_NS_ACK_DURATION : CANONEF_RX_IDLE_NS_ACK_START,
    };

    /* first reset RMT */
    canonef_reset_rmt();

    memset(canonef.lclk_symbols, 0, sizeof(canonef.lclk_symbols));
    error = rmt_receive(canonef.rmt_rx_channel, canonef.lclk_symbols, sizeof(canonef.lclk_symbols), &receive_config);
    if (error == ESP_ERR_INVALID_STATE)
    {
        send_message("Canon EF lens: rmt_receive() failed with error=%s", esp_err_to_name(error));
    }

    if (error != ESP_OK)
    {
        return error;
    }

    error = spi_device_transmit(canonef.spi, &spi_transaction);
    if (error != ESP_OK)
    {
        canonef_reset_rmt();
        return error;
    }

    *rx_byte = spi_rx;

    if (xSemaphoreTake(canonef.rx_done, pdMS_TO_TICKS(CANONEF_TRANSFER_TIMEOUT_MS)) != pdTRUE)
    {
        send_message("Canon EF lens: Unexpected RMT Timeout after SPI transfer");
        canonef_reset_rmt();
        return ESP_ERR_TIMEOUT;
    }

    /* wait until the LCLK line is high again, otherwise we might have a glitch on the next transfer */
    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level((gpio_num_t)canonef.config.lclk_in_gpio) == (lclk_in_inv ? 1 : 0))
    {
        if (esp_timer_get_time() > (CANONEF_RX_IDLE_NS_ACK_DURATION / 1000) + start_time)
        {
            /* if the LCLK line is still low after the maximum time we expect it to be high, something is wrong */
            send_message("Canon EF lens: LCLK line did not go high after SPI transfer");
            canonef_reset_rmt();
            return ESP_ERR_INVALID_STATE;
        }
    }

    /* depending on the mode we might find the full ack or just the delay until it came */
    if (!canonef_find_ack_duration(ack_delay_us, ack_duration_us))
    {
        send_message("Canon EF lens: ACK: ack_delay_us=%u us, ack_duration_us=%u us", *ack_delay_us, *ack_duration_us);
        return ESP_ERR_NOT_FOUND;
    }
    if (capture_ack && *ack_duration_us == UINT16_MAX)
    {
        return ESP_ERR_INVALID_STATE;
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

    if (xSemaphoreTake(canonef.mutex, pdMS_TO_TICKS(CANONEF_TRANSFER_TIMEOUT_MS)) != pdTRUE)
    {
        uint8_t status = CANONEF_STATUS_NOT_INITIALIZED;
        (void)canonef_queue_packet(UART_PACKET_TYPE_LENS_XFER, &status, sizeof(status));
        return ESP_ERR_TIMEOUT;
    }

    bool include_measurement = (canonef.config.flags & CANONEF_CONFIG_FLAG_ACK_DURATION) != 0;
    bool ignore_ack = (canonef.config.flags & CANONEF_CONFIG_FLAG_IGNORE_ACK) != 0;
    size_t record_size = include_measurement ? 4U : 1U;
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
    size_t output_index = 1;

    for (size_t byte_index = 0; byte_index < payload_len; byte_index++)
    {
        uint8_t rx_byte = 0xFF;
        uint8_t ack_delay_us = UINT8_MAX;
        uint16_t ack_duration_us = UINT16_MAX;

        esp_err_t error = canonef_transfer_byte(payload[byte_index], &rx_byte, &ack_delay_us, &ack_duration_us);
        bool stop_after_record = false;

        if (error != ESP_OK)
        {
            if (ignore_ack && (error == ESP_ERR_NOT_FOUND ||
                               error == ESP_ERR_TIMEOUT ||
                               error == ESP_ERR_INVALID_STATE))
            {
                /* ACK failed, but still use that data */
            }
            else
            {
                response[0] = (error == ESP_ERR_NOT_FOUND || error == ESP_ERR_TIMEOUT) ? CANONEF_STATUS_ACK_TIMEOUT
                                                                                       : (error == ESP_ERR_INVALID_STATE ? CANONEF_STATUS_ACK_TOO_LONG : CANONEF_STATUS_TRANSFER_FAILED);
                result = error;
                stop_after_record = true;
            }
        }

        if (include_measurement)
        {
            response[output_index++] = (uint8_t)(ack_delay_us & 0xFFU);
            response[output_index++] = (uint8_t)(ack_duration_us & 0xFFU);
            response[output_index++] = (uint8_t)(ack_duration_us >> 8);
        }
        response[output_index++] = rx_byte;
        response_records++;
        if (stop_after_record)
        {
            break;
        }
    }
    xSemaphoreGive(canonef.mutex);

    esp_err_t queue_error = canonef_queue_packet(UART_PACKET_TYPE_LENS_XFER, response, output_index);
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


    gpio_set_direction((gpio_num_t)lclk_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)lclk_gpio, 0);
    esp_rom_gpio_connect_out_signal(lclk_gpio, SIG_GPIO_OUT_IDX, (canonef.config.inversion_mask & CANONEF_INVERT_DCL) != 0, 0);
    esp_rom_delay_us(CANONEF_RESET_HOLD_US);
    gpio_set_level((gpio_num_t)lclk_gpio, 1);
    esp_rom_gpio_connect_out_signal(lclk_gpio, FSPICLK_OUT_IDX, (canonef.config.inversion_mask & CANONEF_INVERT_DCL) != 0, 0);

    xSemaphoreGive(canonef.mutex);
    uint8_t status = CANONEF_STATUS_OK;
    (void)canonef_queue_packet(UART_PACKET_TYPE_LENS_RESET, &status, sizeof(status));

    return ESP_OK;
}