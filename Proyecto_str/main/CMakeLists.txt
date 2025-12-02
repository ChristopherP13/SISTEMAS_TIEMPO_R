idf_component_register(SRCS "fan_control.c" "main.c" "rgb_led.c" "wifi_app.c" "http_server.c" 
                    PRIV_REQUIRES esp_wifi nvs_flash driver esp_http_server esp_timer app_update json mqtt esp_adc

                    INCLUDE_DIRS "."
                    EMBED_FILES webpage/app.css webpage/app.js webpage/favicon.ico webpage/index.html webpage/jquery-3.3.1.min.js
                    )
