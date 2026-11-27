idf_component_register(SRCS "main.c" "ui.c" "link.c" "hc12.c" "ota.c" "tlog.c"
                       INCLUDE_DIRS "."
                       REQUIRES freertos esp_driver_i2c esp_driver_gpio esp_driver_uart esp_driver_tsens esp_adc
                                esp_timer esp_hw_support esp_system log
                                esp_wifi esp_netif esp_event nvs_flash esp_http_server app_update esp_app_format)
