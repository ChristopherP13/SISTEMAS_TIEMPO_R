idf_component_register(SRCS "rgb_led.c" "ledc_basic_example_main.c"
                    PRIV_REQUIRES esp_driver_ledc esp_driver_gpio esp_timer
                    INCLUDE_DIRS ".")
