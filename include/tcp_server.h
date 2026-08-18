#pragma once

#include "esp_err.h"
#include "uart_gateway.h"

esp_err_t tcp_server_start(void);
void tcp_server_stop(void);

void tcp_server_fanout_packet(const uart_packet_header_t *packet);
