set(component_sources
    "main.c"
    "led.c"
    "uart_gateway.c"
    "logger.c"
    "config_manager.c"
    "wifi_manager.c"
    "tcp_server.c"
)

if(CONFIG_GW_SWD_ENABLED)
    list(APPEND component_sources "swd.c")
endif()

if(CONFIG_GW_CAN_ENABLED)
    list(APPEND component_sources "can.c")
endif()

if(CONFIG_GW_LENS_ENABLED)
    list(APPEND component_sources "canonef.c")
endif()

idf_component_register(
    SRCS ${component_sources}
    INCLUDE_DIRS "../include"
    PRIV_REQUIRES nvs_flash driver esp_timer fatfs wear_levelling esp_wifi esp_netif esp_event esp_http_server lwip mdns
    EMBED_TXTFILES "web/config.html" "web/files.html"
)
