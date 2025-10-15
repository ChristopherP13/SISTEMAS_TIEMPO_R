#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// --- Incluimos nuestra nueva librería ---
#include "rgb_led.h"

// --- Pines para el LED RGB ---
#define RGB_LED_RED_GPIO    12
#define RGB_LED_GREEN_GPIO  14
#define RGB_LED_BLUE_GPIO   13

// --- Pin del botón BOOT ---
// El botón BOOT en la mayoría de las placas ESP32 está conectado al GPIO 0
#define BOOT_BUTTON_GPIO    0

// Semáforo para notificar desde la interrupción a la tarea principal
SemaphoreHandle_t buttonSemaphore = NULL;

// Variable para gestionar el antirrebote (debounce) del botón
volatile int64_t last_interrupt_time = 0;

/**
 * @brief Rutina de Servicio de Interrupción (ISR) para el botón.
 *
 * Esta función se ejecuta CADA VEZ que se presiona el botón.
 * ¡Debe ser muy rápida! Por eso usamos un semáforo para avisar a otra tarea.
 */
static void IRAM_ATTR button_isr_handler(void* arg) {
    // Lógica antirrebote: solo reaccionar si han pasado más de 200ms
    int64_t now = esp_timer_get_time();
    if (now - last_interrupt_time > 500 * 1000) { // 500ms en microsegundos
        last_interrupt_time = now;
        // "Da" el semáforo para despertar a la tarea principal.
        xSemaphoreGiveFromISR(buttonSemaphore, NULL);
    }
}

/**
 * @brief Configura el GPIO del botón y la interrupción.
 */
void button_init(void) {
    // Crear el semáforo binario
    buttonSemaphore = xSemaphoreCreateBinary();

    // Configurar el GPIO
    gpio_config_t io_conf;
    io_conf.pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE; // El botón BOOT necesita pull-up
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // Interrupción por flanco de bajada (al presionar)
    gpio_config(&io_conf);

    // Instalar el servicio de interrupciones
    gpio_install_isr_service(0);
    // Añadir el manejador para nuestro pin específico
    gpio_isr_handler_add(BOOT_BUTTON_GPIO, button_isr_handler, NULL);
}

void app_main(void)
{
    // 1. Inicializa nuestra librería de LED RGB con los pines especificados
    rgb_led_init(RGB_LED_RED_GPIO, RGB_LED_GREEN_GPIO, RGB_LED_BLUE_GPIO);
    printf("Librería RGB LED inicializada.\n");

    // 2. Inicializa el botón y su interrupción
    button_init();
    printf("Botón BOOT (GPIO 0) configurado con interrupción.\n");

    // 3. Variable para llevar la cuenta del color actual
    int color_index = 0;

    // 4. Establece un color inicial (Rojo)
    printf("Color inicial: ROJO\n");
    rgb_led_set_color(255, 0, 0);

    // 5. Bucle principal
    while (1) {
        // Espera "para siempre" hasta que la interrupción del botón libere el semáforo
        if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE) {
            
            // Incrementa el índice de color y vuelve a 0 si llega al final
            color_index = (color_index + 1) % 7; // 7 colores en total

            switch (color_index) {
                case 0: // Rojo
                    printf("Botón presionado. Cambiando a ROJO\n");
                    rgb_led_set_color(255, 0, 0);
                    break;
                case 1: // Verde
                    printf("Botón presionado. Cambiando a VERDE\n");
                    rgb_led_set_color(0, 255, 0);
                    break;
                case 2: // Azul
                    printf("Botón presionado. Cambiando a AZUL\n");
                    rgb_led_set_color(0, 0, 255);
                    break;
                case 3: // Amarillo (Rojo + Verde)
                    printf("Botón presionado. Cambiando a AMARILLO\n");
                    rgb_led_set_color(255, 255, 0);
                    break;
                case 4: // Magenta (Rojo + Azul)
                    printf("Botón presionado. Cambiando a MAGENTA\n");
                    rgb_led_set_color(255, 0, 255);
                    break;
                case 5: // Cian (Verde + Azul)
                    printf("Botón presionado. Cambiando a CIAN\n");
                    rgb_led_set_color(0, 255, 255);
                    break;
                case 6: // Cian (Verde + Azul)
                    printf("Botón presionado. Cambiando a BLANCO\n");
                    rgb_led_set_color(255, 255, 255);
                    break;
            }
        }
    }
}