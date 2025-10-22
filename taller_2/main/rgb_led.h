#ifndef RGB_LED_H
#define RGB_LED_H

#include "driver/ledc.h"

/**
 * @brief Configura e inicializa los pines y el controlador PWM (LEDC) para el LED RGB.
 *
 * @param red_gpio GPIO para el canal Rojo.
 * @param green_gpio GPIO para el canal Verde.
 * @param blue_gpio GPIO para el canal Azul.
 */
void rgb_led_init(int red_gpio, int green_gpio, int blue_gpio);

/**
 * @brief Establece el color del LED RGB.
 *
 * @param red Valor de 0 a 255 para la intensidad del rojo.
 * @param green Valor de 0 a 255 para la intensidad del verde.
 * @param blue Valor de 0 a 255 para la intensidad del azul.
 */
void rgb_led_set_color(uint8_t red, uint8_t green, uint8_t blue);

#endif // RGB_LED_H