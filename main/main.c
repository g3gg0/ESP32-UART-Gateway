
#include <string.h>

#include "esp_system.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "uart_gateway.h"
#include "led.h"
#include "logger.h"
#include "config_manager.h"
#include "wifi_manager.h"


static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void app_main()
{
    uartgw_config_t saved_config;

    /* Initialize NVS */
    initialize_nvs();

    /* Load all data-driven configuration fields */
    config_manager_init();

    const device_config_t *cfg = config_manager_get();
    saved_config.baud_rate = cfg->uart_baud_rate;
    saved_config.tx_gpio = cfg->uart_tx_gpio;
    saved_config.rx_gpio = cfg->uart_rx_gpio;
    saved_config.reset_gpio = cfg->uart_reset_gpio;
    saved_config.control_gpio = cfg->uart_control_gpio;
    saved_config.led_gpio = cfg->uart_led_gpio;
    saved_config.extended_mode = 0;

    /* Start gateway tasks and USB CDC inside uart_gateway */
    uart_gateway_start();

    /* Mount FAT filesystem and open first log file for this boot */
    logger_init();
    logger_start();

    /* Initialize UART gateway (creates stream buffers) */
    uart_gateway_init(&saved_config);

    /* Initialize LED on configured GPIO */
    led_init(&saved_config);

    /* Start WiFi STA/AP and TCP protocol bridge */
    wifi_manager_start();

    /* Give USB CDC time to stabilize before tasks start processing */
    vTaskDelay(100 / portTICK_PERIOD_MS);

    /* Keep logs at ERROR level to avoid corrupting traffic */
    esp_log_level_set("*", ESP_LOG_NONE);

    /* Main loop - just monitor */
    while (true)
    {
        /* Monitoring disabled to prevent log spam */
        vTaskDelay(30000 / portTICK_PERIOD_MS);
    }
}