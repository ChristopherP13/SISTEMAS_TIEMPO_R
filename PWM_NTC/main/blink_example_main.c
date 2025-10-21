#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

//================== Pines y canales ==================//
#define POT_PWM_OUTPUT_PIN    26
#define POT_ADC_CHANNEL       ADC_CHANNEL_6      // GPIO34 (ADC1_CH6)
#define POT_PWM_CHANNEL       LEDC_CHANNEL_0

#define NTC_PWM_OUTPUT_PIN    27
#define NTC_ADC_CHANNEL       ADC_CHANNEL_4      // GPIO32 (ADC1_CH4)
#define NTC_PWM_CHANNEL       LEDC_CHANNEL_1

//================== NTC parámetros ===================//
#define R_FIJA                100000.0f
#define NTC_R_NOMINAL         100000.0f
#define NTC_TEMP_NOMINAL      298.15f           // 25°C en K
#define NTC_BETA              4190.0f
#define VOLTAJE_ENTRADA       3.3f

//================== LEDC (PWM) ======================//
#define PWM_TIMER             LEDC_TIMER_0
#define PWM_MODE              LEDC_HIGH_SPEED_MODE
#define PWM_DUTY_RES          LEDC_TIMER_8_BIT
#define PWM_FREQUENCY         (5000)            // 5 kHz

//================== Periodos tareas =================//
#define SENSE_PERIOD_MS       1000     // 50 Hz muestreo
#define LOG_TIMEOUT_MS        1500   // Logger imprime al menos cada 1 s si no llegan datos

static const char *TAG = "PWM del potenciometro";

//================== Mensaje que viaja por colas =====//
typedef struct {
    int     pot_raw;
    int     ntc_raw;
    float   v_ntc;
    float   temp_c;
    uint32_t pot_pwm;  // 0..255
    uint32_t ntc_pwm;  // 0..255
} sample_msg_t;

//===== Colas (una por consumidor) y semáforos binarios =====//
static QueueHandle_t qAct = NULL;  // Sensor -> Actuador
static QueueHandle_t qLog = NULL;  // Sensor -> Logger
static SemaphoreHandle_t semAct = NULL; // “hay nuevo dato” para Actuador
static SemaphoreHandle_t semLog = NULL; // “hay nuevo dato” para Logger

// ADC handle (sólo lo usa TaskSensor)
static adc_oneshot_unit_handle_t g_adc1 = NULL;

//================== Utilidades =======================//
static inline uint32_t clip_u32(uint32_t x, uint32_t lo, uint32_t hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

//================== Inicialización periféricos =======//
static void init_pwm(void)
{
    ledc_timer_config_t tcfg = {
        .speed_mode      = PWM_MODE,
        .timer_num       = PWM_TIMER,
        .duty_resolution = PWM_DUTY_RES,
        .freq_hz         = PWM_FREQUENCY,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    ledc_channel_config_t cpot = {
        .speed_mode = PWM_MODE,
        .channel    = POT_PWM_CHANNEL,
        .timer_sel  = PWM_TIMER,
        .gpio_num   = POT_PWM_OUTPUT_PIN,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&cpot));

    ledc_channel_config_t cntc = {
        .speed_mode = PWM_MODE,
        .channel    = NTC_PWM_CHANNEL,
        .timer_sel  = PWM_TIMER,
        .gpio_num   = NTC_PWM_OUTPUT_PIN,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&cntc));
}

static void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &g_adc1));

    adc_oneshot_chan_cfg_t chcfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc1, POT_ADC_CHANNEL, &chcfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc1, NTC_ADC_CHANNEL, &chcfg));
}

//================== Task: Sensor (productor) =========//
static void TaskSensor(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();

    while (1) {
        sample_msg_t m = {0};

        // Leer ADC
        (void)adc_oneshot_read(g_adc1, POT_ADC_CHANNEL, &m.pot_raw);
        (void)adc_oneshot_read(g_adc1, NTC_ADC_CHANNEL, &m.ntc_raw);

        // Derivados
        m.pot_pwm = (uint32_t)((m.pot_raw * 255UL) / 4095UL);

        m.v_ntc = m.ntc_raw * (VOLTAJE_ENTRADA / 4095.0f);
        float denom = VOLTAJE_ENTRADA - m.v_ntc;
        float r_ntc = (denom > 1e-6f) ? (R_FIJA * m.v_ntc / denom) : 1e9f;

        float invT   = (1.0f / NTC_TEMP_NOMINAL) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R_NOMINAL);
        float temp_k = 1.0f / invT;
        m.temp_c = temp_k - 273.15f;

        if (m.temp_c <= 0.0f)       m.ntc_pwm = 0;
        else if (m.temp_c >= 50.0f) m.ntc_pwm = 255;
        else                        m.ntc_pwm = (uint32_t)(m.temp_c * 255.0f / 50.0f);

        // Enviar a colas (no bloquear: si están llenas, pisa el más viejo)
        // Profundidad >1 permite "buffer" cuando consumidores están ocupados.
        (void)xQueueSend(qAct, &m, 0);
        (void)xQueueSend(qLog, &m, 0);

        // Despertar consumidores inmediatamente
        xSemaphoreGive(semAct);
        xSemaphoreGive(semLog);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(SENSE_PERIOD_MS));
    }
}

//================== Task: Actuador (consumidor) ======//
static void TaskActuador(void *arg)
{
    (void)arg;

    while (1) {
        // Espera notificación de nueva muestra
        xSemaphoreTake(semAct, portMAX_DELAY);

        // Drena la cola para quedarnos con la última muestra disponible
        sample_msg_t m, latest = {0};
        BaseType_t got_any = pdFALSE;
        while (xQueueReceive(qAct, &m, 0) == pdTRUE) {
            latest = m;
            got_any = pdTRUE;
        }
        if (!got_any) continue; // nada que hacer

        uint32_t pot = clip_u32(latest.pot_pwm, 0, 255);
        uint32_t ntc = clip_u32(latest.ntc_pwm, 0, 255);

        ledc_set_duty(PWM_MODE, POT_PWM_CHANNEL, pot);
        ledc_update_duty(PWM_MODE, POT_PWM_CHANNEL);

        ledc_set_duty(PWM_MODE, NTC_PWM_CHANNEL, ntc);
        ledc_update_duty(PWM_MODE, NTC_PWM_CHANNEL);
    }
}

//================== Task: Logger (consumidor) ========//
static void TaskLogger(void *arg)
{
    (void)arg;

    while (1) {
        // Despierta en evento o cada LOG_TIMEOUT_MS para imprimir algo
        if (xSemaphoreTake(semLog, pdMS_TO_TICKS(LOG_TIMEOUT_MS)) != pdTRUE) {
            // timeout: seguiremos con lo último que haya en la cola (si hay)
        }

        sample_msg_t m, latest = {0};
        BaseType_t got_any = pdFALSE;
        while (xQueueReceive(qLog, &m, 0) == pdTRUE) {
            latest = m;
            got_any = pdTRUE;
        }
        if (!got_any) continue;

        if (latest.temp_c < 0.0f) {
            ESP_LOGI(TAG, " %" PRIu32 "/255 | Temp: < 0 C -> PWM Temp: %" PRIu32 "/255 | V_NTC=%.3f V",
                     latest.pot_pwm, latest.ntc_pwm, latest.v_ntc);
        } else if (latest.temp_c > 50.0f) {
            ESP_LOGI(TAG, " %" PRIu32 "/255 | Temp: > 50 C -> PWM Temp: %" PRIu32 "/255 | V_NTC=%.3f V",
                     latest.pot_pwm, latest.ntc_pwm, latest.v_ntc);
        } else {
            ESP_LOGI(TAG, " %" PRIu32 "/255 | Temp: %.2f C -> PWM Temp: %" PRIu32 "/255 | V_NTC=%.3f V",
                     latest.pot_pwm, latest.temp_c, latest.ntc_pwm, latest.v_ntc);
        }
    }
}

//================== app_main: crea recursos y tareas ==//
void app_main(void)
{
    ESP_LOGI(TAG, "Init peripherals…");
    init_pwm();
    init_adc();

    // Colas con profundidad >1 (pequeño buffer)
    qAct = xQueueCreate(4, sizeof(sample_msg_t));
    qLog = xQueueCreate(4, sizeof(sample_msg_t));
    configASSERT(qAct && qLog);

    // Semáforos binarios (nacen "vacíos")
    semAct = xSemaphoreCreateBinary();
    semLog = xSemaphoreCreateBinary();
    configASSERT(semAct && semLog);

    // Crear tareas (prioridades relativas: Sensor > Actuador > Logger)
    configASSERT(xTaskCreate(TaskSensor,   "SENSE",  2048, NULL, 3, NULL) == pdPASS);
    configASSERT(xTaskCreate(TaskActuador, "ACT",    2048, NULL, 2, NULL) == pdPASS);
    configASSERT(xTaskCreate(TaskLogger,   "LOGGER", 2048, NULL, 1, NULL) == pdPASS);

    // app_main puede terminar aquí
    // vTaskDelete(NULL);
}