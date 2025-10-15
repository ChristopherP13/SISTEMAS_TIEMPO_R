#include "rgb_led.h"

// Definimos los canales LEDC que usaremos para cada color
#define LEDC_CHANNEL_R LEDC_CHANNEL_0
#define LEDC_CHANNEL_G LEDC_CHANNEL_1
#define LEDC_CHANNEL_B LEDC_CHANNEL_2

// Usaremos una resolución de 8 bits para el ciclo de trabajo (0-255)
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT

void rgb_led_init(int red_gpio, int green_gpio, int blue_gpio) {
    // 1. Configuración del temporizador LEDC
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = 5000, // Frecuencia de 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // 2. Configuración del canal para el color ROJO
    ledc_channel_config_t ledc_channel_r = {
        .channel    = LEDC_CHANNEL_R,
        .duty       = 0,
        .gpio_num   = red_gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel_r);

    // 3. Configuración del canal para el color VERDE
    ledc_channel_config_t ledc_channel_g = {
        .channel    = LEDC_CHANNEL_G,
        .duty       = 0,
        .gpio_num   = green_gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel_g);

    // 4. Configuración del canal para el color AZUL
    ledc_channel_config_t ledc_channel_b = {
        .channel    = LEDC_CHANNEL_B,
        .duty       = 0,
        .gpio_num   = blue_gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel_b);
}

void rgb_led_set_color(uint8_t red, uint8_t green, uint8_t blue) {
    // Actualizar el ciclo de trabajo para cada canal
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_R, red);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_R);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_G, green);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_G);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_B, blue);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_B);
}