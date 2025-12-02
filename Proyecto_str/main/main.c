/**
 * Application entry point.
 */

#include "nvs_flash.h"
#include "wifi_app.h"
#include "driver/gpio.h"
#include "fan_control.h"

#define BLINK_GPIO               2

static void configure_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Start Wifi and time sync helper
    init_obtain_time();
    configure_led();
    wifi_app_start();

    // Start fan controller
    ESP_ERROR_CHECK(fan_control_init());
}
