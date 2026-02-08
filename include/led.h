#ifndef LED_H
#define LED_H

#include "esp_err.h"

/* Initialize LED on specified GPIO */
esp_err_t led_init();

/* Signal UART activity - triggers 20 Hz blink for a duration */
void led_signal_uart_activity(void);

#endif /* LED_H */
