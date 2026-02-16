#pragma once


#include "esp_err.h"
#include "uart_gateway.h"

/* Initialize LED task using configured GPIO */
esp_err_t led_init(const uartgw_config_t *config);

/* Stop LED task and deinitialize GPIO */
void led_stop(void);

/* Signal UART activity - triggers active blink for a duration */
void led_signal_uart_activity(void);

