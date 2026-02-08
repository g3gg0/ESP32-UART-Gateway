# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

idf_component_register(
    SRCS "main.c" "led.c" "uart_gateway.c"
    INCLUDE_DIRS "../include"
    PRIV_REQUIRES nvs_flash driver
)
