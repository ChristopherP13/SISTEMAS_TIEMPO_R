/********************************************************************************
 * Archivo principal (app_main.c) — versión con comentarios exhaustivos
 * -----------------------------------------------------------------------------
 * - Mantiene la MISMA funcionalidad que tu código (no se cambia lógica).
 * - Añade comentarios con "lujo de detalle" para estudio de RT/ESP-IDF.
 * - Destaca explícitamente las zonas de “CAMBIO REALIZADO” respecto a la versión
 *   previa (uso de rgb_led_pins_t y rgb_color_t de tu nueva librería).
 *******************************************************************************/

#include <stdio.h>              // printf, sscanf: E/S estándar (redirigida a UART0/monitor)
#include <string.h>             // strchr: útil para detectar '?' en el buffer de UART
#include <math.h>               // logf: requerido por ecuación Beta del NTC
#include "freertos/FreeRTOS.h"  // Tipos RTOS (TickType_t), macros (pdMS_TO_TICKS), configASSERT
#include "freertos/task.h"      // API de tareas: xTaskCreate, vTaskDelayUntil, xTaskGetTickCount
#include "freertos/queue.h"     // API de colas: xQueueCreate, xQueueSend, xQueueReceive
#include "freertos/semphr.h"    // API de semáforos/mutex: xSemaphoreCreateMutex, Take/Give
#include "driver/gpio.h"        // GPIO: gpio_config, gpio_install_isr_service, gpio_isr_handler_add
#include "driver/uart.h"        // UART: uart_driver_install, uart_param_config, uart_set_pin, uart_read_bytes
#include "esp_log.h"            // Logging estructurado (TAG + niveles); aquí sólo declaramos TAG
#include "esp_adc/adc_oneshot.h"// ADC en modo "oneshot": config unidad/canales y lectura puntual

// Incluimos la librería personalizada para el LED RGB (tu header con structs y API)
#include "rgb_led.h"

/*================== Pines y canales ==================*/
// Pines de salida para el LED RGB (se usarán con LEDC PWM a través de la librería)
#define RGB_RED_PIN         25   // GPIO del canal ROJO
#define RGB_GREEN_PIN       26   // GPIO del canal VERDE
#define RGB_BLUE_PIN        27   // GPIO del canal AZUL

// Sensores ADC (mapeo lógico → canal ADC1)
#define POT_ADC_CHANNEL     ADC_CHANNEL_6      // GPIO34 ↔ ADC1_CH6 (entrada analógica: sólo input)
#define NTC_ADC_CHANNEL     ADC_CHANNEL_4      // GPIO32 ↔ ADC1_CH4

// Botón BOOT (GPIO0) con pull-up interno; al presionar → nivel bajo (flanco NEGEDGE)
#define BOOT_BUTTON_PIN     GPIO_NUM_0

/*================== NTC parámetros ===================*/
// Parámetros eléctricos y termoeléctricos del divisor con NTC (modelo Beta)
#define R_FIJA              100000.0f  // Resistencia fija del divisor (Ohm) — 100kΩ
#define NTC_R_NOMINAL       100000.0f  // Resistencia del NTC a 25°C (Ohm) — 100kΩ
#define NTC_TEMP_NOMINAL    298.15f    // T0 en Kelvin (25°C = 298.15 K)
#define NTC_BETA            4190.0f    // Constante Beta del NTC (K)
#define VOLTAJE_ENTRADA     3.3f       // Vin del divisor / Vref efectiva del ADC (V)

/*================== UART parámetros ==================*/
// UART de consola y comandos (la 0 suele estar enlazada a USB/UART del DevKit)
#define UART_NUM            UART_NUM_0
#define UART_BUF_SIZE       256        // Buffer de lectura (bytes)

/*================== Periodos tareas =================*/
// Periodicidad de la tarea de sensado (usamos vTaskDelayUntil para exactitud temporal)
#define SENSE_PERIOD_MS     2000       // 2000 ms → 2 s entre lecturas

// Identificador de módulo para ESP_LOGx (útil si usas logging estructurado en vez de printf)
static const char *TAG = "RGB_NTC_CONTROL";

/*================== Tipos de datos para intercambio ==================*/
// Rango de temperaturas (°C) asociado a un color; si min>max, rango "deshabilitado"
typedef struct {
    float min_temp;
    float max_temp;
} color_range_t;

// Mensaje que publica TaskSensor (y que consumen LED/Logger)
typedef struct {
    float   temp_c;         // Temperatura estimada (°C) vía ecuación Beta
    float   v_ntc;          // Voltaje en el nodo del NTC (V)
    uint8_t pot_intensity;  // Intensidad 0..255 (POT 12 bits → 8 bits por división)
} sensor_data_t;

/*================== Recursos RTOS globales ==================*/
// Colas de comunicación entre tareas (productor/consumidor)
// - qLedControl: últimos datos hacia el controlador del LED
// - qLogger:     últimos datos hacia el logger (impresión condicional)
static QueueHandle_t qLedControl = NULL;
static QueueHandle_t qLogger     = NULL;

// Mutex para proteger el acceso concurrente a los rangos de color globales
// (Escritura desde TaskUart, lectura desde TaskLedControl)
static SemaphoreHandle_t g_ranges_mutex = NULL;

/*================== Estado global compartido ==================*/
// Rangos por color (se inician deshabilitados con min>max)
static color_range_t g_red_range   = { .min_temp = 1.0f, .max_temp = 0.0f };
static color_range_t g_green_range = { .min_temp = 1.0f, .max_temp = 0.0f };
static color_range_t g_blue_range  = { .min_temp = 1.0f, .max_temp = 0.0f };

// Bandera toggled por ISR para habilitar/deshabilitar logging; 'volatile' evita que
// el compilador asuma que no cambia “fuera” del flujo de la tarea (acceso desde ISR).
static volatile bool g_log_enabled = false;

// Handle de la unidad ADC1 para lecturas oneshot (lo devuelve el driver)
static adc_oneshot_unit_handle_t g_adc1_handle = NULL;


/*================== Rutina de Interrupción (ISR) del Botón ===================*/
// - IRAM_ATTR: la ISR se ubica en RAM interna (reduce latencia / seguro ante cache misses).
// - Acción mínima en ISR: Togglear un bool (operación no bloqueante, atómica en este caso).
static void IRAM_ATTR boot_button_isr_handler(void* arg) {
    g_log_enabled = !g_log_enabled;   // Alterna ON/OFF del logging (se lee en TaskLogger)
}


/*================== Inicialización de periféricos ===================*/
// ADC1 en modo oneshot: crea unidad y registra canales POT y NTC con misma config
static void init_adc(void) {
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1 };     // Selección de ADC1
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &g_adc1_handle));     // Crea unidad y entrega handle

    // Config común para ambos canales: 12 bits y 12 dB (≈0..3.3V)
    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,   // 12 bits (0..4095)
        .atten    = ADC_ATTEN_DB_12         // Extiende rango a ~3.3V
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc1_handle, POT_ADC_CHANNEL, &channel_config)); // POT
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc1_handle, NTC_ADC_CHANNEL, &channel_config)); // NTC
}

// Configura GPIO0 como entrada con pull-up e interrupción por flanco de bajada (al presionar)
static void init_button_isr(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_PIN), // Máscara de pin (bit 0 → GPIO0)
        .mode = GPIO_MODE_INPUT,                    // Sólo lectura
        .pull_up_en = 1,                            // Pull-up interno habilitado
        .intr_type = GPIO_INTR_NEGEDGE              // Interrupción por flanco descendente
    };
    gpio_config(&io_conf);                          // Aplica configuración
    gpio_install_isr_service(0);                    // Instala servicio ISR global (prioridad por defecto)
    gpio_isr_handler_add(BOOT_BUTTON_PIN,           // Registra nuestra ISR para el pin BOOT
                         boot_button_isr_handler,
                         NULL);                     // 'arg' opcional (no usado)
}

// Configura UART0 (115200-8N1, sin flow control) y deja pines por defecto (TX/RX del DevKit)
static void init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate  = 115200,                // Velocidad en baudios
        .data_bits  = UART_DATA_8_BITS,      // 8 bits de dato
        .parity     = UART_PARITY_DISABLE,   // Sin paridad
        .stop_bits  = UART_STOP_BITS_1,      // 1 bit stop
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE, // Sin RTS/CTS
        .source_clk = UART_SCLK_DEFAULT,     // Reloj fuente por defecto
    };
    // Instala driver con buffer RX (doble del solicitado); sin buffer TX dedicado ni cola de eventos
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));  // Aplica parámetros
    // Mantiene pines por defecto (TX=GPIO1, RX=GPIO3 en muchas placas; RTS/CTS no usados)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}


/*================== Task: Sensor (Productor periódico) ===================*/
// - Dispara lecturas ADC cada SENSE_PERIOD_MS con vTaskDelayUntil (periodicidad absoluta).
// - Publica un sensor_data_t a dos colas (LED y Logger). Patrón "last value wins" (colas de profundidad 1).
static void TaskSensor(void *arg) {
    TickType_t last_wake_time = xTaskGetTickCount();  // Marca de tiempo base para mantener período exacto

    while (1) {
        sensor_data_t msg = {0};                      // Inicializa a cero todos los campos
        int pot_raw, ntc_raw;                         // Lecturas crudas (12 bits: 0..4095)

        // 1) Lectura ADC (bloquean durante la conversión; latencia µs)
        ESP_ERROR_CHECK(adc_oneshot_read(g_adc1_handle, POT_ADC_CHANNEL, &pot_raw));
        ESP_ERROR_CHECK(adc_oneshot_read(g_adc1_handle, NTC_ADC_CHANNEL, &ntc_raw));

        // 2) Procesamiento:
        //    a) Potenciómetro: 12b → 8b (escala simple por división)
        msg.pot_intensity = (uint8_t)(pot_raw / 16);  // 4096/16 = 256 → valores exactos 0..255

        //    b) NTC: cuentas → voltaje → resistencia → temperatura (modelo Beta)
        msg.v_ntc = ntc_raw * (VOLTAJE_ENTRADA / 4095.0f);   // Vntc = raw * (Vref / 4095)
        // Evitar división por casi cero si Vntc ≈ Vin → r_ntc → muy grande (saturación)
        float r_ntc = (VOLTAJE_ENTRADA - msg.v_ntc > 1e-6f)
                    ? (R_FIJA * msg.v_ntc / (VOLTAJE_ENTRADA - msg.v_ntc))
                    : 1e9f;
        // Ecuación Beta: 1/T = 1/T0 + (1/β)*ln(R/R0)
        float invT = (1.0f / NTC_TEMP_NOMINAL) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R_NOMINAL);
        msg.temp_c = (1.0f / invT) - 273.15f;                   // Kelvin → °C

        // 3) Publicación a consumidores (colas tamaño 1: si están llenas, xQueueSend con 0 ticks descarta el nuevo)
        xQueueSend(qLedControl, &msg, 0);
        xQueueSend(qLogger,     &msg, 0);

        // 4) Espera hasta el siguiente “tick de período” (evita deriva temporal acumulada)
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SENSE_PERIOD_MS));
    }
}


/*================== Task: LED Control (Consumidor) ===================*/
// - Consume el último sensor_data_t de qLedControl.
// - Lee rangos de color bajo mutex para coherencia (TaskUart puede estar escribiendo).
// - Calcula base R/G/B (255 si la T cae en el rango) y escala por la intensidad del potenciómetro.
// - CAMBIO REALIZADO: arma un rgb_color_t y llama a rgb_led_set_color(final_color).
static void TaskLedControl(void *arg) {
    sensor_data_t data;

    while (1) {
        // Espera bloqueante a nueva muestra (cola profundidad 1 ⇒ siempre el último estado)
        if (xQueueReceive(qLedControl, &data, portMAX_DELAY) == pdTRUE) {
            uint8_t base_r = 0, base_g = 0, base_b = 0;  // Canales base antes de escalar por intensidad

            // Sección crítica breve: lectura coherente de rangos bajo protección
            xSemaphoreTake(g_ranges_mutex, portMAX_DELAY);
            if (data.temp_c >= g_red_range .min_temp && data.temp_c <= g_red_range .max_temp) base_r = 255;
            if (data.temp_c >= g_green_range.min_temp && data.temp_c <= g_green_range.max_temp) base_g = 255;
            if (data.temp_c >= g_blue_range .min_temp && data.temp_c <= g_blue_range .max_temp) base_b = 255;
            xSemaphoreGive(g_ranges_mutex);

            // Escalado por la intensidad del potenciómetro (0..255)
            uint8_t final_r = (uint8_t)((base_r * data.pot_intensity) / 255);
            uint8_t final_g = (uint8_t)((base_g * data.pot_intensity) / 255);
            uint8_t final_b = (uint8_t)((base_b * data.pot_intensity) / 255);

            // ================== CAMBIO REALIZADO ================== //
            // Construimos la estructura rgb_color_t esperada por la NUEVA API de la librería:
            rgb_color_t final_color = {
                .r = final_r,
                .g = final_g,
                .b = final_b
            };
            // Llamada atómica (semánticamente) para fijar el color (LEDC duty por canal):
            rgb_led_set_color(final_color);
            // ====================================================== //
        }
    }
}


/*================== Task: Logger (Consumidor) ===================*/
// - Consume la última muestra de qLogger y la imprime SI g_log_enabled==true.
// - La bandera se alterna desde la ISR; por eso es 'volatile' y se consulta en cada iteración.
static void TaskLogger(void *arg) {
    sensor_data_t data;

    while (1) {
        if (xQueueReceive(qLogger, &data, portMAX_DELAY) == pdTRUE) {
            if (g_log_enabled) {
                // Nota: printf puede ser relativamente costoso; Logger tiene prioridad menor.
                printf("Temp: %.2f C | Potenciometro (PWM): %d/255 | V_NTC: %.2f V\n",
                       data.temp_c, data.pot_intensity, data.v_ntc);
            }
        }
    }
}


/*================== Task: UART Control (Parser de comandos) ===================*/
// - Imprime ayuda de comandos cuando el usuario escribe '?'.
// - Parsea "R/G/B min max" y actualiza rangos protegidos por mutex.
// - Se validan límites [0,50] °C (NEW validation).
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
    uint8_t *data = (uint8_t *) malloc(UART_BUF_SIZE);  // Buffer de recepción UART (se libera al final del while(1))

    while (1) {
        // Lectura no bloqueante (timeout 20 ms) para poder responder con fluidez
        int len = uart_read_bytes(UART_NUM, data, (UART_BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        if (len) {
            data[len] = '\0';  // Garantiza terminación NUL para tratarlo como string C
            char  cmd;         // 'R'/'G'/'B' (case-insensitive)
            float min_val, max_val;

            // Intenta parsear: 1 char + 2 floats (con separadores en blanco)
            if (sscanf((char*)data, "%c %f %f", &cmd, &min_val, &max_val) == 3) {

                // --- NUEVA VALIDACIÓN --- (limita a rango razonable de trabajo)
                if (min_val < 0.0f || max_val > 50.0f) {
                    printf("Error: Los valores de temperatura deben estar entre 0.0 y 50.0.\n");
                } else {
                    bool updated = false;

                    // Sección crítica: se actualizan los rangos globales bajo mutex
                    xSemaphoreTake(g_ranges_mutex, portMAX_DELAY);
                    switch(cmd) {
                        case 'R': case 'r':
                            g_red_range.min_temp   = min_val;
                            g_red_range.max_temp   = max_val;
                            updated = true; break;
                        case 'G': case 'g':
                            g_green_range.min_temp = min_val;
                            g_green_range.max_temp = max_val;
                            updated = true; break;
                        case 'B': case 'b':
                            g_blue_range.min_temp  = min_val;
                            g_blue_range.max_temp  = max_val;
                            updated = true; break;
                        default:
                            printf("Error: Color '%c' no reconocido.\n", cmd);
                            break;
                    }
                    xSemaphoreGive(g_ranges_mutex);

                    if (updated) {
                        // Notifica la configuración aplicada; si min>max, se considera deshabilitado por diseño.
                        printf("OK: Rango para el color %c actualizado a [%.2f, %.2f]\n", cmd, min_val, max_val);
                    }
                }

            } else if (strchr((char*)data, '?') != NULL) {
                // Cualquier aparición de '?' en la entrada dispara la ayuda
                print_uart_help();
            } else {
                // Entrada no válida (ni formato, ni ayuda)
                printf("Error: Comando no valido. Envie '?' para ayuda.\n");
            }
        }
    }
    free(data); // Higiene — en la práctica no se ejecuta por el while(1).
}


/*================== app_main: Bootstrap del sistema ===================*/
// - Inicializa periféricos (ADC, GPIO/ISR, UART) y la librería del RGB.
// - Crea colas/mutex (recursos RTOS) y luego las tareas con sus prioridades.
// - Imprime mensajes de bienvenida/ayuda.
void app_main(void) {
    // 1) Inicialización de periféricos (drivers antes de arrancar tareas que dependan de ellos)
    init_adc();
    init_button_isr();
    init_uart();

    // ================== CAMBIO REALIZADO ================== //
    // Se crea una estructura con el mapeo de pines del LED y se pasa por puntero
    // a la nueva API rgb_led_init(const rgb_led_pins_t*).
    rgb_led_pins_t led_pins = {
        .red_pin   = RGB_RED_PIN,
        .green_pin = RGB_GREEN_PIN,
        .blue_pin  = RGB_BLUE_PIN
    };
    rgb_led_init(&led_pins);   // Inicializa LEDC timer + canales R/G/B con duty=0
    // ====================================================== //

    // 2) Creación de recursos RTOS (colas de profundidad 1 → “last value wins”, y mutex de rangos)
    qLedControl     = xQueueCreate(1, sizeof(sensor_data_t));
    qLogger         = xQueueCreate(1, sizeof(sensor_data_t));
    g_ranges_mutex  = xSemaphoreCreateMutex();

    // Verificación defensiva (si algún recurso falló por falta de heap, aborta)
    configASSERT(qLedControl && qLogger && g_ranges_mutex);

    // 3) Creación de tareas (función, nombre, stack bytes, parámetro, prioridad, handle opcional)
    xTaskCreate(TaskSensor,     "Sensor",     2048, NULL, 5, NULL); // Periódica → prioridad mayor
    xTaskCreate(TaskLedControl, "LedControl", 2048, NULL, 4, NULL); // Control (usa lectura + cálculo corto)
    xTaskCreate(TaskLogger,     "Logger",     2048, NULL, 3, NULL); // I/O (printf) → prioridad menor
    xTaskCreate(TaskUart,       "Uart",       2560, NULL, 4, NULL); // Parsing puede usar sscanf → stack mayor

    // 4) Mensajes UX (consola)
    printf("\nPrograma iniciado\n");
    printf("Presione el boton BOOT para activar/desactivar el log de datos.\n");
    printf("Envie por el terminal el signo '?' para mostrar los comandos de configuracion.\n");
}

/* ===================== FIN DEL PROGRAMA ===================== */