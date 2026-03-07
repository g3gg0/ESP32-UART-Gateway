#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* Deinitialize CAN driver/session state. Safe to call repeatedly. */
void can_stop_session(void);

/* Initialize/start CAN session with selected pins and bitrate (bit/s). */
esp_err_t can_start_session(uint8_t can_rx_gpio, uint8_t can_tx_gpio, uint32_t can_baud);

/* Handle UART_PACKET_TYPE_CAN payload (request header + args). */
esp_err_t can_handle_packet(const uint8_t *payload, size_t payload_len);

/* FreeRTOS task to read CAN frames and forward them to host. */
void can_read_task(void *pvParameters);
