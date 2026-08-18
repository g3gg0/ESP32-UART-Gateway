# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

idf_component_register(
    SRCS "main.c" "led.c" "uart_gateway.c" "swd.c" "can.c" "logger.c" "config_manager.c" "wifi_manager.c" "tcp_server.c"
    INCLUDE_DIRS "../include"
    PRIV_REQUIRES nvs_flash driver esp_timer fatfs wear_levelling esp_wifi esp_netif esp_event esp_http_server lwip mdns
    EMBED_TXTFILES "web/config.html" "web/files.html"
)
