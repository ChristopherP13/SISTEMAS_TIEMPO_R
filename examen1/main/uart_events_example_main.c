/********************************************************************************
 * SEBASTIAN TORRES GAMBOA 1122530252
 * CHRISTOPHER ANDRES PIEDRAHITA PAVAS 1054479153.
 *******************************************************************************/

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
#include "rgb_led.h"

/*================== Pines, canales y constantes ==================*/
#define RGB_RED_PIN         25
#define RGB_GREEN_PIN       26
#define RGB_BLUE_PIN        27
#define POT_ADC_CHANNEL     ADC_CHANNEL_6
#define NTC_ADC_CHANNEL     ADC_CHANNEL_4
#define BOOT_BUTTON_PIN     GPIO_NUM_0
#define R_FIJA              100000.0f
#define NTC_R_NOMINAL       100000.0f
#define NTC_TEMP_NOMINAL    298.15f
#define NTC_BETA            4190.0f
#define VOLTAJE_ENTRADA     3.3f
#define UART_NUM            UART_NUM_0
#define UART_BUF_SIZE       256
#define SENSE_PERIOD_MS_DEFAULT 2000
static const char *TAG = "RGB_NTC_CONTROL";

/*================== Tipos de datos ==================*/
typedef struct {
    float min_temp;
    float max_temp;
} color_range_t;

typedef struct {
    float   temp_c;
    float   v_ntc;
    uint8_t pot_intensity;
} sensor_data_t;

// Estructura de Contexto de la Aplicación para agrupar todo el estado
typedef struct {
    QueueHandle_t       qLedControl;
    QueueHandle_t       qLogger;
    SemaphoreHandle_t   ranges_mutex;
    adc_oneshot_unit_handle_t adc1_handle;
    color_range_t   red_range;
    color_range_t   green_range;
    color_range_t   blue_range;
    volatile bool       log_enabled;
    volatile bool       led_enabled;
    volatile uint32_t   sense_period_ms;
} AppContext_t;


/*================== Rutina de Interrupción (ISR) del Botón ===================*/
// La ISR recibe el contexto y alterna el estado del flag 'led_enabled'
static void IRAM_ATTR boot_button_isr_handler(void* arg) {
    AppContext_t *context = (AppContext_t *) arg;
    context->led_enabled = !context->led_enabled;
}


/*================== Inicialización de periféricos ===================*/
static void init_adc(AppContext_t *context) {
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &context->adc1_handle));
    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(context->adc1_handle, POT_ADC_CHANNEL, &channel_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(context->adc1_handle, NTC_ADC_CHANNEL, &channel_config));
}

static void init_button_isr(AppContext_t *context) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOOT_BUTTON_PIN, boot_button_isr_handler, context);
}

static void init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate  = 115200, .data_bits  = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1, .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}


/*================== Tareas del sistema ===================*/
static void TaskSensor(void *arg) {
    AppContext_t *context = (AppContext_t *) arg;
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1) {
        sensor_data_t msg = {0};
        int pot_raw, ntc_raw;
        ESP_ERROR_CHECK(adc_oneshot_read(context->adc1_handle, POT_ADC_CHANNEL, &pot_raw));
        ESP_ERROR_CHECK(adc_oneshot_read(context->adc1_handle, NTC_ADC_CHANNEL, &ntc_raw));
        msg.pot_intensity = (uint8_t)(pot_raw / 16);
        msg.v_ntc = ntc_raw * (VOLTAJE_ENTRADA / 4095.0f);
        float r_ntc = (VOLTAJE_ENTRADA - msg.v_ntc > 1e-6f)
                    ? (R_FIJA * msg.v_ntc / (VOLTAJE_ENTRADA - msg.v_ntc)) : 1e9f;
        float invT = (1.0f / NTC_TEMP_NOMINAL) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R_NOMINAL);
        msg.temp_c = (1.0f / invT) - 273.15f;
        xQueueSend(context->qLedControl, &msg, 0);
        xQueueSend(context->qLogger,     &msg, 0);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(context->sense_period_ms));
    }
}

static void TaskLedControl(void *arg) {
    AppContext_t *context = (AppContext_t *) arg;
    sensor_data_t data;
    while (1) {
        if (xQueueReceive(context->qLedControl, &data, portMAX_DELAY) == pdTRUE) {
            // Comprueba el flag a través del contexto. Si es 'false', apaga el LED.
            if (!context->led_enabled) {
                rgb_led_set_color((rgb_color_t){ .r = 0, .g = 0, .b = 0 });
                continue;
            }
            uint8_t base_r = 0, base_g = 0, base_b = 0;
            xSemaphoreTake(context->ranges_mutex, portMAX_DELAY);
            if (data.temp_c >= context->red_range.min_temp && data.temp_c <= context->red_range.max_temp) base_r = 255;
            if (data.temp_c >= context->green_range.min_temp && data.temp_c <= context->green_range.max_temp) base_g = 255;
            if (data.temp_c >= context->blue_range.min_temp && data.temp_c <= context->blue_range.max_temp) base_b = 255;
            xSemaphoreGive(context->ranges_mutex);
            rgb_color_t final_color = {
                .r = (uint8_t)((base_r * data.pot_intensity) / 255),
                .g = (uint8_t)((base_g * data.pot_intensity) / 255),
                .b = (uint8_t)((base_b * data.pot_intensity) / 255)
            };
            rgb_led_set_color(final_color);
        }
    }
}

static void TaskLogger(void *arg) {
    AppContext_t *context = (AppContext_t *) arg;
    sensor_data_t data;
    while (1) {
        if (xQueueReceive(context->qLogger, &data, portMAX_DELAY) == pdTRUE) {
            if (context->log_enabled) {
                printf("Temp: %.2f C | Pot (PWM): %d/255 | V_NTC: %.2f V\n",
                       data.temp_c, data.pot_intensity, data.v_ntc);
            }
        }
    }
}

static void print_uart_help() {
    printf("\n--- Comandos Disponibles ---\n"
           "  ?               : Muestra esta ayuda.\n"
           "  R min max       : Fija el rango de temperatura para el ROJO.   Ej: R 0 15.5\n"
           "  G min max       : Fija el rango de temperatura para el VERDE.  Ej: G 16 30\n"
           "  B min max       : Fija el rango de temperatura para el AZUL.   Ej: B 30.1 50\n"
           "  T <miliseg>     : Fija el periodo de muestreo de sensores.   Ej: T 1000\n"
           "  V               : Muestra el voltaje actual del potenciometro.\n"
           "  L               : Activa/desactiva el log de datos por consola.\n"
           "------------------------------------------------------------------\n"
           "NOTA: El rango de temperatura debe estar entre 0.0 y 50.0 grados.\n"
           "Para deshabilitar un color, pon un rango invalido (ej: R 1 0)\n"
           "------------------------------------------------------------------\n");
}

static void TaskUart(void *arg) {
    AppContext_t *context = (AppContext_t *) arg;
    uint8_t *data = (uint8_t *) malloc(UART_BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, (UART_BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';
            char cmd = data[0];
            float min_val, max_val;
            uint32_t period;
            switch(cmd) {
                case 'R': case 'r': case 'G': case 'g': case 'B': case 'b':
                    if (sscanf((char*)data, "%c %f %f", &cmd, &min_val, &max_val) == 3) {
                        if (min_val < 0.0f || max_val > 50.0f) { printf("Error: Temp debe estar entre 0.0 y 50.0.\n"); }
                        else {
                            xSemaphoreTake(context->ranges_mutex, portMAX_DELAY);
                            if (cmd == 'R' || cmd == 'r') { context->red_range = (color_range_t){min_val, max_val}; }
                            if (cmd == 'G' || cmd == 'g') { context->green_range = (color_range_t){min_val, max_val}; }
                            if (cmd == 'B' || cmd == 'b') { context->blue_range = (color_range_t){min_val, max_val}; }
                            xSemaphoreGive(context->ranges_mutex);
                            printf("OK: Rango para %c actualizado a [%.2f, %.2f]\n", cmd, min_val, max_val);
                        }
                    } else { printf("Error: Formato incorrecto. Uso: COLOR min max\n"); }
                    break;
                case 'T': case 't':
                    if (sscanf((char*)data, "%c %lu", &cmd, &period) == 2) {
                        if (period >= 100) {
                            context->sense_period_ms = period;
                            printf("OK: Periodo de muestreo actualizado a %lu ms\n", context->sense_period_ms);
                        } else { printf("Error: El periodo debe ser >= 100 ms.\n"); }
                    } else { printf("Error: Formato incorrecto. Uso: T <milisegundos>\n"); }
                    break;
                case 'V': case 'v': {
                    int pot_raw;
                    ESP_ERROR_CHECK(adc_oneshot_read(context->adc1_handle, POT_ADC_CHANNEL, &pot_raw));
                    printf("Voltaje del potenciometro: %.2f V\n", pot_raw * (VOLTAJE_ENTRADA / 4095.0f));
                    } break;
                case 'L': case 'l':
                    context->log_enabled = !context->log_enabled;
                    printf("OK: Log de datos %s.\n", context->log_enabled ? "habilitado" : "deshabilitado");
                    break;
                case '?': print_uart_help(); break;
                default:
                    if (strchr((char*)data, '?') != NULL) { print_uart_help(); }
                    else { printf("Error: Comando no valido. Envie '?' para ayuda.\n"); }
                    break;
            }
        }
    }
    free(data);
}


/*================== app_main: Bootstrap del sistema ===================*/
void app_main(void) {
    AppContext_t *app_context = (AppContext_t *) calloc(1, sizeof(AppContext_t));
    if (!app_context) {
        printf("Error critico: No se pudo alocar memoria para el contexto.\n");
        return;
    }
    
    // Inicialización del estado en el contexto
    app_context->red_range = (color_range_t){ .min_temp = 1.0f, .max_temp = 0.0f };
    app_context->green_range = (color_range_t){ .min_temp = 1.0f, .max_temp = 0.0f };
    app_context->blue_range = (color_range_t){ .min_temp = 1.0f, .max_temp = 0.0f };
    app_context->log_enabled = false;
    app_context->led_enabled = false; 
    app_context->sense_period_ms = SENSE_PERIOD_MS_DEFAULT;

    // Inicialización de periféricos y recursos RTOS
    init_adc(app_context);
    init_button_isr(app_context);
    init_uart();
    rgb_led_init(&(rgb_led_pins_t){ .red_pin = RGB_RED_PIN, .green_pin = RGB_GREEN_PIN, .blue_pin = RGB_BLUE_PIN });
    app_context->qLedControl = xQueueCreate(1, sizeof(sensor_data_t));
    app_context->qLogger     = xQueueCreate(1, sizeof(sensor_data_t));
    app_context->ranges_mutex  = xSemaphoreCreateMutex();
    configASSERT(app_context->qLedControl && app_context->qLogger && app_context->ranges_mutex);

    // Creación de tareas, pasando el contexto a cada una
    xTaskCreate(TaskSensor,     "Sensor",     2048, app_context, 5, NULL);
    xTaskCreate(TaskLedControl, "LedControl", 2048, app_context, 4, NULL);
    xTaskCreate(TaskLogger,     "Logger",     2048, app_context, 3, NULL);
    xTaskCreate(TaskUart,       "Uart",       2560, app_context, 4, NULL);

    printf("\nPrograma iniciado (version sin variables globales)\n");
    printf("Presione el boton BOOT para encender/apagar el LED.\n");
    printf("Envie por el terminal el signo '?' para mostrar los comandos de configuracion.\n");
}

/* ===================== FIN DEL PROGRAMA ===================== */