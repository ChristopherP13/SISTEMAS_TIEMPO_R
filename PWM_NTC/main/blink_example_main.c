#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"
#include <inttypes.h>

// Se incluye la librería moderna para el ADC
#include "esp_adc/adc_oneshot.h"

//===== Configuración de Pines y Canales =====//
// Pin de salida para la señal PWM.
#define PWM_OUTPUT_PIN    26
// Unidad de ADC a usar (en ESP32, GPIO34 está en ADC1)
#define POT_ADC_UNIT      ADC_UNIT_1
// Canal de ADC correspondiente al GPIO34
#define POT_ADC_CHANNEL   ADC_CHANNEL_6


//===== Configuración de PWM (LEDC) =====//
#define PWM_TIMER         LEDC_TIMER_0
#define PWM_MODE          LEDC_HIGH_SPEED_MODE
#define PWM_CHANNEL       LEDC_CHANNEL_0
#define PWM_DUTY_RES      LEDC_TIMER_8_BIT // Resolución de 8 bits (0-255)
#define PWM_FREQUENCY     (5000) // Frecuencia en Hz


//===== Configuración de Monitoreo =====//
#define MONITOR_INTERVAL_MS 500 // Intervalo en milisegundos (2 segundos)

// Tag para los mensajes de logging
static const char *TAG = "PWM_CONTROL";

void app_main(void)
{
    //------------------- CONFIGURACIÓN DEL PWM (LEDC) -------------------//
    // PASO 1: Configurar el timer del periférico LEDC
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = PWM_MODE,
        .timer_num        = PWM_TIMER,
        .duty_resolution  = PWM_DUTY_RES,
        .freq_hz          = PWM_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // PASO 2: Configurar el canal del periférico LEDC
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = PWM_MODE,
        .channel        = PWM_CHANNEL,
        .timer_sel      = PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PWM_OUTPUT_PIN,
        .duty           = 0, // Iniciar con ciclo de trabajo 0
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));


    //------------------- CONFIGURACIÓN DEL ADC (API Moderna Oneshot) -------------------//
    // PASO 3: Inicializar la unidad del ADC
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = POT_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // PASO 4: Configurar el canal del ADC
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // Ancho de bits por defecto (usualmente 12 bits -> 0-4095)
        .atten = ADC_ATTEN_DB_12,         // Atenuación de 12dB para poder leer el rango completo (0-3.3V)
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, POT_ADC_CHANNEL, &config));

    ESP_LOGI(TAG, "Configuración completada. Iniciando bucle principal.");

    //------------------- BUCLE PRINCIPAL -------------------//
    while (1) {
        int pot_raw_value;
        // Leer el valor crudo del ADC usando la nueva API
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, POT_ADC_CHANNEL, &pot_raw_value));

        // Mapear el valor del ADC (0-4095) al rango del PWM (0-255)
        uint32_t duty_cycle = (pot_raw_value * 255) / 4095;

        // Establecer el nuevo ciclo de trabajo
        ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty_cycle));
        // Actualizar la salida para que el cambio tenga efecto
        ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL));

        // Imprimir el ciclo de trabajo actual en el monitor
        ESP_LOGI(TAG, "Valor ADC: %d -> Ciclo de trabajo PWM: %" PRIu32, pot_raw_value, duty_cycle);

        // Esperar el intervalo de tiempo definido para el monitoreo
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}