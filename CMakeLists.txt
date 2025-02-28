idf_component_register(SRCS "src/esp32wake.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_gpio esp_driver_uart wake_protocol)
