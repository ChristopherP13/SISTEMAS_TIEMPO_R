#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

// Incluimos la librería personalizada para el LED RGB
#include "rgb_led.h"

//================== Pines y canales ==================//
// Pines para el LED RGB
#define RGB_RED_PIN         25
#define RGB_GREEN_PIN       26
#define RGB_BLUE_PIN        27

// Sensores ADC
#define POT_ADC_CHANNEL       ADC_CHANNEL_6      // GPIO34 (ADC1_CH6)
#define NTC_ADC_CHANNEL       ADC_CHANNEL_4      // GPIO32 (ADC1_CH4)

// Botón BOOT
#define BOOT_BUTTON_PIN     GPIO_NUM_0

//================== NTC parámetros ===================//
#define R_FIJA                100000.0f
#define NTC_R_NOMINAL         100000.0f
#define NTC_TEMP_NOMINAL      298.15f           // 25°C en K
#define NTC_BETA              4190.0f
#define VOLTAJE_ENTRADA       3.3f

//================== UART parámetros ==================//
#define UART_NUM              UART_NUM_0
#define UART_BUF_SIZE         256

//================== Periodos tareas =================//
#define SENSE_PERIOD_MS       2000      // Muestreo de sensores cada 2 segundos

static const char *TAG = "RGB_NTC_CONTROL";

// Estructura para definir el rango de temperatura de un color
typedef struct {
    float min_temp;
    float max_temp;
} color_range_t;

// Estructura del mensaje que se envía entre tareas
typedef struct {
    float   temp_c;         // Temperatura calculada en Celsius
    float   v_ntc;          // Voltaje medido en el divisor del NTC
    uint8_t pot_intensity;  // Intensidad del 0-255 calculada del potenciómetro
} sensor_data_t;

//===== Colas y Mutex =====//
static QueueHandle_t qLedControl = NULL;
static QueueHandle_t qLogger = NULL;
static SemaphoreHandle_t g_ranges_mutex = NULL; // Mutex para proteger el acceso a los rangos

//===== Variables Globales =====//
// Rangos de temperatura para cada color. Protegidos por el Mutex.
static color_range_t g_red_range   = { .min_temp = 1.0f, .max_temp = 0.0f }; // Inicia deshabilitado
static color_range_t g_green_range = { .min_temp = 1.0f, .max_temp = 0.0f }; // Inicia deshabilitado
static color_range_t g_blue_range  = { .min_temp = 1.0f, .max_temp = 0.0f }; // Inicia deshabilitado

// Flag para habilitar/deshabilitar el logging. 'volatile' es crucial para ISRs.
static volatile bool g_log_enabled = false;

// ADC handle
static adc_oneshot_unit_handle_t g_adc1_handle = NULL;


//================== Rutina de Interrupción (ISR) para el Botón =================//
static void IRAM_ATTR boot_button_isr_handler(void* arg) {
    g_log_enabled = !g_log_enabled;
}


//================== Inicialización Periféricos =======================//
static void init_adc(void) {
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &g_adc1_handle));

    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc1_handle, POT_ADC_CHANNEL, &channel_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc1_handle, NTC_ADC_CHANNEL, &channel_config));
}

static void init_button_isr(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .intr_type = GPIO_INTR_NEGEDGE // Interrupción en flanco de bajada (al presionar)
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOOT_BUTTON_PIN, boot_button_isr_handler, NULL);
}

static void init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}


//================== Task: Sensor (Productor) =========================//
static void TaskSensor(void *arg) {
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        sensor_data_t msg = {0};
        int pot_raw, ntc_raw;

        // 1. Leer ADCs
        ESP_ERROR_CHECK(adc_oneshot_read(g_adc1_handle, POT_ADC_CHANNEL, &pot_raw));
        ESP_ERROR_CHECK(adc_oneshot_read(g_adc1_handle, NTC_ADC_CHANNEL, &ntc_raw));

        // 2. Procesar datos
        // Intensidad del potenciómetro (0-4095 -> 0-255)
        msg.pot_intensity = (uint8_t)(pot_raw / 16); // 4096 / 16 = 256

        // Temperatura del NTC
        msg.v_ntc = ntc_raw * (VOLTAJE_ENTRADA / 4095.0f);
        float r_ntc = (VOLTAJE_ENTRADA - msg.v_ntc > 1e-6f) ? (R_FIJA * msg.v_ntc / (VOLTAJE_ENTRADA - msg.v_ntc)) : 1e9f;
        float invT = (1.0f / NTC_TEMP_NOMINAL) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R_NOMINAL);
        msg.temp_c = (1.0f / invT) - 273.15f;

        // 3. Enviar datos a las colas de los consumidores
        xQueueSend(qLedControl, &msg, 0); // Sobrescribe si la cola está llena
        xQueueSend(qLogger, &msg, 0);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SENSE_PERIOD_MS));
    }
}


//================== Task: LED Control (Consumidor) ===================//
static void TaskLedControl(void *arg) {
    sensor_data_t data;
    while (1) {
        // Espera a recibir nuevos datos del sensor
        if (xQueueReceive(qLedControl, &data, portMAX_DELAY) == pdTRUE) {
            uint8_t base_r = 0, base_g = 0, base_b = 0;

            // Bloquea el mutex antes de leer los rangos compartidos
            xSemaphoreTake(g_ranges_mutex, portMAX_DELAY);

            // Comprueba si la temperatura actual está dentro del rango de cada color
            if (data.temp_c >= g_red_range.min_temp && data.temp_c <= g_red_range.max_temp) {
                base_r = 255;
            }
            if (data.temp_c >= g_green_range.min_temp && data.temp_c <= g_green_range.max_temp) {
                base_g = 255;
            }
            if (data.temp_c >= g_blue_range.min_temp && data.temp_c <= g_blue_range.max_temp) {
                base_b = 255;
            }

            // Libera el mutex después de leer
            xSemaphoreGive(g_ranges_mutex);

            // Aplica la intensidad del potenciómetro
            uint8_t final_r = (uint8_t)((base_r * data.pot_intensity) / 255);
            uint8_t final_g = (uint8_t)((base_g * data.pot_intensity) / 255);
            uint8_t final_b = (uint8_t)((base_b * data.pot_intensity) / 255);

            // Establece el color final del LED usando la librería
            rgb_led_set_color(final_r, final_g, final_b);
        }
    }
}


//================== Task: Logger (Consumidor) ========================//
static void TaskLogger(void *arg) {
    sensor_data_t data;
    while (1) {
        // Espera a recibir nuevos datos
        if (xQueueReceive(qLogger, &data, portMAX_DELAY) == pdTRUE) {
            // Solo imprime si el flag de logging está activado
            if (g_log_enabled) {
                printf("Temp: %.2f C | Potenciometro (PWM): %d/255 | V_NTC: %.2f V\n",
                       data.temp_c, data.pot_intensity, data.v_ntc);
            }
        }
    }
}


//================== Task: UART Control ===============================//
static void print_uart_help() {
    printf("\n--- Comandos Disponibles ---\n");
    printf("Uso: COLOR VALOR_MIN VALOR_MAX\n");
    printf("NOTA: El rango de temperatura debe estar entre 0.0 y 50.0 grados.\n");
    printf("Ejemplo: R 0 15.5  (Enciende el LED Rojo entre 0 y 15.5 grados)\n");
    printf("Ejemplo: G 16 30   (Enciende el LED Verde entre 16 y 30 grados)\n");
    printf("Ejemplo: B 30.1 50 (Enciende el LED Azul entre 30.1 y 50 grados)\n");
    printf("Para deshabilitar un color, pon un rango invalido (ej: R 1 0)\n");
    printf("----------------------------\n");
}

static void TaskUart(void *arg) {
    uint8_t *data = (uint8_t *) malloc(UART_BUF_SIZE);

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, (UART_BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        if (len) {
            data[len] = '\0'; // Asegura que el string termine
            char cmd;
            float min_val, max_val;

            // Intenta parsear el comando
            if (sscanf((char*)data, "%c %f %f", &cmd, &min_val, &max_val) == 3) {
                
                // --- NUEVA VALIDACIÓN ---
                if (min_val < 0.0f || max_val > 50.0f) {
                    printf("Error: Los valores de temperatura deben estar entre 0.0 y 50.0.\n");
                } else {
                    bool updated = false;
                    // Bloquea el mutex para escribir en los rangos
                    xSemaphoreTake(g_ranges_mutex, portMAX_DELAY);
                    switch(cmd) {
                        case 'R': case 'r':
                            g_red_range.min_temp = min_val;
                            g_red_range.max_temp = max_val;
                            updated = true;
                            break;
                        case 'G': case 'g':
                            g_green_range.min_temp = min_val;
                            g_green_range.max_temp = max_val;
                            updated = true;
                            break;
                        case 'B': case 'b':
                            g_blue_range.min_temp = min_val;
                            g_blue_range.max_temp = max_val;
                            updated = true;
                            break;
                        default:
                            printf("Error: Color '%c' no reconocido.\n", cmd);
                    }
                    xSemaphoreGive(g_ranges_mutex);

                    if (updated) {
                         printf("OK: Rango para el color %c actualizado a [%.2f, %.2f]\n", cmd, min_val, max_val);
                    }
                }

            } else if (strchr((char*)data, '?') != NULL) {
                print_uart_help();
            } else {
                printf("Error: Comando no valido. Envie '?' para ayuda.\n");
            }
        }
    }
    free(data);
}


//================== app_main: Crea recursos y tareas =================//
void app_main(void) {
    // 1. Inicialización de periféricos
    init_adc();
    init_button_isr();
    init_uart();
    rgb_led_init(RGB_RED_PIN, RGB_GREEN_PIN, RGB_BLUE_PIN);
    
    // 2. Creación de recursos de FreeRTOS
    qLedControl = xQueueCreate(1, sizeof(sensor_data_t)); // Profundidad 1, siempre queremos el último dato
    qLogger = xQueueCreate(1, sizeof(sensor_data_t));
    g_ranges_mutex = xSemaphoreCreateMutex();

    configASSERT(qLedControl && qLogger && g_ranges_mutex);

    // 3. Creación de tareas
    xTaskCreate(TaskSensor,     "Sensor",     2048, NULL, 5, NULL);
    xTaskCreate(TaskLedControl, "LedControl", 2048, NULL, 4, NULL);
    xTaskCreate(TaskLogger,     "Logger",     2048, NULL, 3, NULL);
    xTaskCreate(TaskUart,       "Uart",       2560, NULL, 4, NULL); // Aumentado stack por el uso de printf/sscanf

    // 4. Mensajes de inicio
    printf("\nPrograma iniciado\n");
    printf("Presione el boton BOOT para activar/desactivar el log de datos.\n");
    printf("Envie por el terminal el signo '?' para mostrar los comandos de configuracion.\n");
}