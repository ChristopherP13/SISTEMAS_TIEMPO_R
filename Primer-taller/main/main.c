#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdbool.h> // Necesario para usar el tipo de dato 'bool' (true/false)

// Define los pines GPIO para el LED y el Botón
#define LED_PIN     GPIO_NUM_2
#define BUTTON_PIN  GPIO_NUM_0

void app_main(void)
{
    // --- Configuración del LED (GPIO 2) como SALIDA ---
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    // --- Configuración del Botón (GPIO 0) como ENTRADA con PULL-UP ---
    gpio_config_t io_conf_button = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_button);

    // --- Variables para gestionar el estado ---
    bool is_blinking = false;       // Estado actual del sistema (parpadeando o no)
    int last_button_state = 1;      // Último estado leído del botón (1 = no presionado)

    printf("¡Programa iniciado! Presiona el botón BOOT para iniciar/detener el parpadeo.\n");

    // Bucle infinito
    while (1) {
        // --- Lógica para detectar el pulso del botón (evitar rebotes) ---
        int current_button_state = gpio_get_level(BUTTON_PIN);

        // Verificamos si el botón FUE presionado (antes estaba en 1, ahora en 0)
        if (last_button_state == 1 && current_button_state == 0) {
            // Un pequeño retardo para el rebote del botón
            vTaskDelay(pdMS_TO_TICKS(20)); 
            
            // Cambiamos el estado de parpadeo
            is_blinking = !is_blinking; 
            
            if (is_blinking) {
                printf("Modo parpadeo: ACTIVADO\n");
            } else {
                printf("Modo parpadeo: DESACTIVADO\n");
            }
        }
        // Actualizamos el último estado del botón para la siguiente iteración
        last_button_state = current_button_state;


        // --- Lógica principal basada en el estado ---
        if (is_blinking) {
            // Si el modo parpadeo está activo, hacemos que el LED parpadee
            // 500ms encendido + 500ms apagado = 1 segundo de frecuencia
            gpio_set_level(LED_PIN, 1); // Encender LED
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(LED_PIN, 0); // Apagar LED
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            // Si el modo parpadeo está inactivo, nos aseguramos de que el LED esté apagado
            gpio_set_level(LED_PIN, 0);
            // Pequeña pausa para no sobrecargar el procesador cuando no hace nada
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}