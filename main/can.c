#include <string.h>
#include <stdlib.h>

#include "can.h"
#include "uart_gateway.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "CAN_MODULE"

static bool can_session_active = false;
static bool can_driver_installed = false;
static uint8_t can_rx_gpio_active = 0xFF;
static uint8_t can_tx_gpio_active = 0xFF;
static uint32_t can_baud_active = CAN_DEFAULT_BAUD;
static uint16_t can_event_flags_sticky = 0;

static uint32_t read_u32_le(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static esp_err_t can_queue_response(uint8_t req_op, uint16_t seq, can_uart_status_t status,
                                    const uint8_t *data, uint16_t data_len, bool include_pins)
{
    uint8_t can_rx = 0xFF;
    uint8_t can_tx = 0xFF;
    if (include_pins && can_driver_installed)
    {
        can_rx = can_rx_gpio_active;
        can_tx = can_tx_gpio_active;
    }

    size_t packet_size = UART_PACKET_HEADER_SIZE + sizeof(can_uart_rsp_hdr_t) + data_len;
    uart_packet_header_t *packet = (uart_packet_header_t *)malloc(packet_size);
    if (!packet)
    {
        return ESP_ERR_NO_MEM;
    }

    packet->length = (uint16_t)packet_size;
    packet->type = UART_PACKET_TYPE_CAN;

    can_uart_rsp_hdr_t *rsp = (can_uart_rsp_hdr_t *)PTR_BEHIND(packet);
    rsp->magic = CAN_UART_MAGIC;
    rsp->op = (uint8_t)(req_op | 0x80);
    rsp->status = (uint8_t)status;
    rsp->seq = seq;
    rsp->can_rx_gpio = can_rx;
    rsp->can_tx_gpio = can_tx;
    rsp->reserved = 0;
    rsp->data_len = data_len;

    if (data_len && data)
    {
        memcpy(((uint8_t *)rsp) + sizeof(*rsp), data, data_len);
    }

    return queue_packet(packet);
}

void can_stop_session(void)
{
    if (!can_driver_installed)
    {
        can_session_active = false;
        can_rx_gpio_active = 0xFF;
        can_tx_gpio_active = 0xFF;
        can_baud_active = CAN_DEFAULT_BAUD;
        can_event_flags_sticky = 0;
        return;
    }

    (void)twai_stop();
    (void)twai_driver_uninstall();

    can_driver_installed = false;
    can_session_active = false;
    can_rx_gpio_active = 0xFF;
    can_tx_gpio_active = 0xFF;
    can_baud_active = CAN_DEFAULT_BAUD;
    can_event_flags_sticky = 0;
}

static esp_err_t can_get_timing_config(uint32_t can_baud, twai_timing_config_t *out)
{
    if (!out)
    {
        return ESP_ERR_INVALID_ARG;
    }

    switch (can_baud)
    {
    case 25000:
        *out = (twai_timing_config_t)TWAI_TIMING_CONFIG_25KBITS();
        break;
    case 50000:
        *out = (twai_timing_config_t)TWAI_TIMING_CONFIG_50KBITS();
        break;
    case 100000:
        *out = (twai_timing_config_t)TWAI_TIMING_CONFIG_100KBITS();
        break;
    case 125000:
        *out = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS();
        break;
    case 250000:
        *out = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS();
        break;
    case 500000:
        *out = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS();
        break;
    case 1000000:
        *out = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS();
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t can_start_session(uint8_t can_rx_gpio, uint8_t can_tx_gpio, uint32_t can_baud)
{
    if (can_rx_gpio > 43 || can_tx_gpio > 43)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (can_rx_gpio == can_tx_gpio)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (can_baud == 0)
    {
        can_baud = CAN_DEFAULT_BAUD;
    }

    twai_timing_config_t t_config;
    if (can_get_timing_config(can_baud, &t_config) != ESP_OK)
    {
        return ESP_ERR_INVALID_ARG;
    }

    can_stop_session();

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(can_tx_gpio, can_rx_gpio, TWAI_MODE_NORMAL);
    g_config.alerts_enabled = TWAI_ALERT_RX_QUEUE_FULL |
                              TWAI_ALERT_RX_FIFO_OVERRUN |
                              TWAI_ALERT_ABOVE_ERR_WARN |
                              TWAI_ALERT_BELOW_ERR_WARN |
                              TWAI_ALERT_ERR_PASS |
                              TWAI_ALERT_ARB_LOST |
                              TWAI_ALERT_BUS_ERROR |
                              TWAI_ALERT_BUS_OFF |
                              TWAI_ALERT_BUS_RECOVERED;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK)
    {
        return err;
    }
    can_driver_installed = true;

    err = twai_start();
    if (err != ESP_OK)
    {
        (void)twai_driver_uninstall();
        can_driver_installed = false;
        return err;
    }

    can_rx_gpio_active = can_rx_gpio;
    can_tx_gpio_active = can_tx_gpio;
    can_baud_active = can_baud;
    can_session_active = false;

    return ESP_OK;
}

esp_err_t can_handle_packet(const uint8_t *payload, size_t payload_len)
{
    if (!payload)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (payload_len < sizeof(can_uart_req_hdr_t))
    {
        (void)can_queue_response(0x00, 0, CAN_UART_STATUS_BAD_LEN, NULL, 0, false);
        return ESP_ERR_INVALID_SIZE;
    }

    const can_uart_req_hdr_t *req = (const can_uart_req_hdr_t *)payload;
    if (req->magic != CAN_UART_MAGIC)
    {
        (void)can_queue_response(req->op, req->seq, CAN_UART_STATUS_BAD_LEN, NULL, 0, false);
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint8_t *args = payload + sizeof(*req);
    size_t arg_len = payload_len - sizeof(*req);
    uint8_t op = req->op;
    uint16_t seq = req->seq;

    switch (op)
    {
    case CAN_UART_OP_START:
    {
        if (arg_len < 4)
        {
            (void)can_queue_response(op, seq, CAN_UART_STATUS_BAD_LEN, NULL, 0, true);
            return ESP_ERR_INVALID_SIZE;
        }

        uint8_t can_rx_gpio = args[0];
        uint8_t can_tx_gpio = args[1];
        uint32_t can_baud = CAN_DEFAULT_BAUD;
        if (arg_len >= 8)
        {
            can_baud = read_u32_le(&args[4]);
        }

        esp_err_t err = can_start_session(can_rx_gpio, can_tx_gpio, can_baud);
        if (err == ESP_OK)
        {
            esp_err_t qerr = can_queue_response(op, seq, CAN_UART_STATUS_OK, NULL, 0, true);
            if (qerr == ESP_OK)
            {
                can_session_active = true;
                send_message("CAN started: RX=%u TX=%u baud=%lu", can_rx_gpio_active, can_tx_gpio_active, can_baud_active);
                return ESP_OK;
            }

            can_stop_session();
            return qerr;
        }

        if (err == ESP_ERR_INVALID_ARG)
        {
            (void)can_queue_response(op, seq, CAN_UART_STATUS_INVALID_ARG, NULL, 0, false);
        }
        else
        {
            (void)can_queue_response(op, seq, CAN_UART_STATUS_INTERNAL, NULL, 0, false);
        }
        return err;
    }

    case CAN_UART_OP_STOP:
    {
        if (!can_session_active)
        {
            (void)can_queue_response(op, seq, CAN_UART_STATUS_NOT_INITIALIZED, NULL, 0, false);
            return ESP_ERR_INVALID_STATE;
        }

        can_stop_session();
        (void)can_queue_response(op, seq, CAN_UART_STATUS_OK, NULL, 0, false);
        send_message("CAN stopped");
        return ESP_OK;
    }

    default:
        (void)can_queue_response(op, seq, CAN_UART_STATUS_BAD_OP, NULL, 0, true);
        return ESP_ERR_NOT_SUPPORTED;
    }
}

void can_read_task(void *pvParameters)
{
    (void)pvParameters;

    twai_message_t message;
    twai_status_info_t status_info;

    ESP_LOGI(TAG, "CAN read task started");

    while (1)
    {
        if (!can_session_active)
        {
            vTaskDelay(20 / portTICK_PERIOD_MS);
            continue;
        }

        uint32_t alerts = 0;
        if (twai_read_alerts(&alerts, 0) == ESP_OK && alerts)
        {
            if (alerts & TWAI_ALERT_RX_QUEUE_FULL)
            {
                can_event_flags_sticky |= CAN_UART_EVENT_FLAG_RX_QUEUE_FULL;
            }
            if (alerts & TWAI_ALERT_RX_FIFO_OVERRUN)
            {
                can_event_flags_sticky |= CAN_UART_EVENT_FLAG_RX_FIFO_OVERRUN;
            }
            if (alerts & TWAI_ALERT_ABOVE_ERR_WARN)
            {
                can_event_flags_sticky |= CAN_UART_EVENT_FLAG_ABOVE_ERR_WARN;
            }
            if (alerts & TWAI_ALERT_BELOW_ERR_WARN)
            {
                can_event_flags_sticky |= CAN_UART_EVENT_FLAG_BELOW_ERR_WARN;
            }
            if (alerts & TWAI_ALERT_ERR_PASS)
            {
                can_event_flags_sticky |= CAN_UART_EVENT_FLAG_ERR_PASS;
            }
            if (alerts & TWAI_ALERT_ARB_LOST)
            {
                can_event_flags_sticky |= CAN_UART_EVENT_FLAG_ARB_LOST;
            }
            if (alerts & TWAI_ALERT_BUS_ERROR)
            {
                can_event_flags_sticky |= CAN_UART_EVENT_FLAG_BUS_ERROR;
            }
            if (alerts & TWAI_ALERT_BUS_OFF)
            {
                can_event_flags_sticky |= CAN_UART_EVENT_FLAG_BUS_OFF;
            }
            if (alerts & TWAI_ALERT_BUS_RECOVERED)
            {
                can_event_flags_sticky |= CAN_UART_EVENT_FLAG_BUS_RECOVERED;
            }
        }

        esp_err_t err = twai_receive(&message, pdMS_TO_TICKS(100));
        if (err == ESP_ERR_TIMEOUT)
        {
            continue;
        }
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "CAN receive error: %s", esp_err_to_name(err));
            continue;
        }

        can_uart_frame_t frame = {
            .id = message.identifier,
            .dlc = message.data_length_code,
            .flags = 0,
            .event_flags = can_event_flags_sticky,
            .timestamp_us = (uint64_t)esp_timer_get_time(),
            .rx_missed_count = 0,
            .rx_overrun_count = 0,
            .data = {0},
        };
        can_event_flags_sticky = 0;

        if (message.extd)
        {
            frame.flags |= CAN_UART_FRAME_FLAG_EXTD;
        }
        if (message.rtr)
        {
            frame.flags |= CAN_UART_FRAME_FLAG_RTR;
        }
        if (message.ss)
        {
            frame.flags |= CAN_UART_FRAME_FLAG_SS;
        }
        if (message.self)
        {
            frame.flags |= CAN_UART_FRAME_FLAG_SELF;
        }
        if (message.dlc_non_comp)
        {
            frame.flags |= CAN_UART_FRAME_FLAG_DLC_NON_COMP;
        }

        if (!message.rtr && frame.dlc > 0)
        {
            size_t copy_len = frame.dlc;
            if (copy_len > sizeof(frame.data))
            {
                copy_len = sizeof(frame.data);
            }
            memcpy(frame.data, message.data, copy_len);
        }

        if (twai_get_status_info(&status_info) == ESP_OK)
        {
            frame.rx_missed_count = status_info.rx_missed_count;
            frame.rx_overrun_count = status_info.rx_overrun_count;
        }

        esp_err_t qerr = can_queue_response(CAN_UART_OP_RX_FRAME, 0, CAN_UART_STATUS_OK,
                                            (const uint8_t *)&frame, (uint16_t)sizeof(frame), true);
        if (qerr != ESP_OK)
        {
            ESP_LOGW(TAG, "CAN queue RX frame failed: %s", esp_err_to_name(qerr));
        }
    }
}
