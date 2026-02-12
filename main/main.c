
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
#include <string.h>

#define TAG "ESP32-UART"

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
    uartgw_config_t saved_config = {
        .baud_rate = UART_DEFAULT_BAUD,
        .tx_gpio = UART_DEFAULT_TX_GPIO,
        .rx_gpio = UART_DEFAULT_RX_GPIO,
        .reset_gpio = UART_DEFAULT_RESET_GPIO,
        .control_gpio = UART_DEFAULT_CONTROL_GPIO,
        .led_gpio = UART_DEFAULT_LED_GPIO,
        .extended_mode = 0,
    };

    ESP_LOGI(TAG, "Starting UART Gateway");

    /* Initialize NVS */
    initialize_nvs();

    /* Start gateway tasks and USB CDC inside uart_gateway */
    uart_gateway_start();

    /* Try to load saved configuration from NVS */
    esp_err_t load_result = uart_gateway_load_config(&saved_config);
    if (load_result == ESP_OK)
    {
        ESP_LOGI(TAG, "Loaded persisted UART configuration from NVS");
    }
    else if (load_result != ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGW(TAG, "Failed to load configuration from NVS: %s", esp_err_to_name(load_result));
    }

    /* Initialize UART gateway (creates stream buffers) */
    uart_gateway_init(&saved_config);

    /* Initialize LED on configured GPIO */
    led_init(&saved_config);

    /* Give USB CDC time to stabilize before tasks start processing */
    ESP_LOGI(TAG, "Waiting for USB CDC to stabilize...");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "System ready, entering main loop");

    /* Keep logs at ERROR level to avoid corrupting traffic */
    esp_log_level_set("*", ESP_LOG_NONE);

    /* Main loop - just monitor */
    while (true)
    {
        /* Monitoring disabled to prevent log spam */
        vTaskDelay(30000 / portTICK_PERIOD_MS);
    }
}