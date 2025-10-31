#ifndef RGB_LED_H
#define RGB_LED_H

#include "driver/ledc.h"

// =======================================================
// --- NUEVAS ESTRUCTURAS ---
// =======================================================

/**
 * @brief Estructura para definir los pines GPIO de un LED RGB.
 */
typedef struct {
    int red_pin;
    int green_pin;
    int blue_pin;
} rgb_led_pins_t;

/**
 * @brief Estructura para representar un color RGB.
 */
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_color_t;


// =======================================================
// --- PROTOTIPOS DE FUNCIONES MODIFICADOS ---
// =======================================================

/**
 * @brief Configura e inicializa los pines y el controlador PWM (LEDC) para el LED RGB.
 *
 * @param pins Puntero a una estructura que contiene los GPIOs para cada canal.
 */
void rgb_led_init(const rgb_led_pins_t *pins);

/**
 * @brief Establece el color del LED RGB.
 *
 * @param color Una estructura que contiene los valores de 0 a 255 para cada color.
 */
void rgb_led_set_color(rgb_color_t color);

#endif // RGB_LED_H