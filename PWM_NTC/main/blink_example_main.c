#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"
#include <inttypes.h>
#include "esp_adc/adc_oneshot.h"
#include <math.h> // Necesario para el NTC

//================================================================//
//==              CONFIGURACIÓN DEL POTENCIÓMETRO               ==//
//================================================================//
#define POT_PWM_OUTPUT_PIN    26
#define POT_ADC_UNIT          ADC_UNIT_1
#define POT_ADC_CHANNEL       ADC_CHANNEL_6     // Conectado a GPIO34
#define POT_PWM_CHANNEL       LEDC_CHANNEL_0    // Usará el canal PWM 0

//================================================================//
//==               CONFIGURACIÓN DEL TERMISTOR NTC              ==//
//================================================================//
#define NTC_PWM_OUTPUT_PIN    27                // Pin de salida para el LED de temperatura
#define NTC_ADC_UNIT          ADC_UNIT_1        // Comparte la unidad ADC con el potenciómetro
#define NTC_ADC_CHANNEL       ADC_CHANNEL_4     // Conectado a GPIO32
#define NTC_PWM_CHANNEL       LEDC_CHANNEL_1    // Usará el canal PWM 1
#define R_FIJA                100000.0          // Resistencia fija de 100k Ohms
#define NTC_R_NOMINAL         100000.0          // Resistencia nominal del NTC a 25°C
#define NTC_TEMP_NOMINAL      298.15            // Temperatura nominal en Kelvin (25 + 273.15)
#define NTC_BETA              4190.0            // Coeficiente Beta del NTC
#define VOLTAJE_ENTRADA       3.3               // Voltaje de alimentación del divisor

//================================================================//
//==             CONFIGURACIÓN GENERAL DE PWM Y MONITOREO       ==//
//================================================================//
#define PWM_TIMER             LEDC_TIMER_0      // Ambos PWM compartirán el mismo timer
#define PWM_MODE              LEDC_HIGH_SPEED_MODE
#define PWM_DUTY_RES          LEDC_TIMER_8_BIT
#define PWM_FREQUENCY         (5000)
#define MONITOR_INTERVAL_MS   1000              // Intervalo de 3 segundos
static const char *TAG = "CONTROL_DUAL";


void app_main(void)
{
    //------------------- CONFIGURACIÓN DEL HARDWARE -------------------//
    // 1. Configurar el TIMER del PWM (común para ambos LEDs)
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = PWM_MODE,
        .timer_num        = PWM_TIMER,
        .duty_resolution  = PWM_DUTY_RES,
        .freq_hz          = PWM_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 2. Configurar el CANAL PWM para el Potenciómetro
    ledc_channel_config_t pot_ledc_channel = {
        .speed_mode     = PWM_MODE,
        .channel        = POT_PWM_CHANNEL,
        .timer_sel      = PWM_TIMER,
        .gpio_num       = POT_PWM_OUTPUT_PIN,
        .duty           = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pot_ledc_channel));
    
    // 3. Configurar el CANAL PWM para el NTC
    ledc_channel_config_t ntc_ledc_channel = {
        .speed_mode     = PWM_MODE,
        .channel        = NTC_PWM_CHANNEL,
        .timer_sel      = PWM_TIMER,
        .gpio_num       = NTC_PWM_OUTPUT_PIN,
        .duty           = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ntc_ledc_channel));

    // 4. Inicializar la UNIDAD ADC (común para ambos sensores)
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // 5. Configurar el CANAL ADC para el Potenciómetro
    adc_oneshot_chan_cfg_t adc_config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12, };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, POT_ADC_CHANNEL, &adc_config));
    
    // 6. Configurar el CANAL ADC para el NTC
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, NTC_ADC_CHANNEL, &adc_config));

    ESP_LOGI(TAG, "Configuración completada. Iniciando bucle principal.");

    //------------------- BUCLE PRINCIPAL -------------------//
    while (1) {
        
        // --- TAREA 1: Lógica del Potenciómetro ---
        int pot_raw_value;
        uint32_t pot_duty_cycle;

        adc_oneshot_read(adc1_handle, POT_ADC_CHANNEL, &pot_raw_value);
        pot_duty_cycle = (pot_raw_value * 255) / 4095;
        ledc_set_duty(PWM_MODE, POT_PWM_CHANNEL, pot_duty_cycle);
        ledc_update_duty(PWM_MODE, POT_PWM_CHANNEL);


        // --- TAREA 2: Lógica del Termistor NTC ---
        int ntc_raw_value;
        float ntc_voltaje, ntc_resistencia, temp_kelvin, temp_celsius;
        uint32_t ntc_duty_cycle;

        adc_oneshot_read(adc1_handle, NTC_ADC_CHANNEL, &ntc_raw_value);
        ntc_voltaje = ntc_raw_value * (VOLTAJE_ENTRADA / 4095.0);
        ntc_resistencia = R_FIJA * ntc_voltaje / (VOLTAJE_ENTRADA - ntc_voltaje);
        temp_kelvin = 1.0 / ( (1.0 / NTC_TEMP_NOMINAL) + (1.0 / NTC_BETA) * log(ntc_resistencia / NTC_R_NOMINAL) );
        temp_celsius = temp_kelvin - 273.15;
        
        if (temp_celsius <= 0) { ntc_duty_cycle = 0; } 
        else if (temp_celsius >= 50) { ntc_duty_cycle = 255; } 
        else { ntc_duty_cycle = (uint32_t)(temp_celsius * 255.0 / 50.0); }
        
        ledc_set_duty(PWM_MODE, NTC_PWM_CHANNEL, ntc_duty_cycle);
        ledc_update_duty(PWM_MODE, NTC_PWM_CHANNEL);

        
        // --- TAREA 3: Reporte en el Monitor Serie ---
        if (temp_celsius < 0) {
            ESP_LOGI(TAG, "Pot: %" PRIu32 "/255 | Temp: < 0 C -> PWM Temp: %" PRIu32 "/255" "Voltaje Medido: %.3f V", pot_duty_cycle, ntc_duty_cycle,ntc_voltaje);
        } else if (temp_celsius > 50) {
            ESP_LOGI(TAG, "Pot: %" PRIu32 "/255 | Temp: > 50 C -> PWM Temp: %" PRIu32 "/255""Voltaje Medido: %.3f V", pot_duty_cycle, ntc_duty_cycle,ntc_voltaje);
        } else {
            ESP_LOGI(TAG, "Pot: %" PRIu32 "/255 | Temp: %.2f C -> PWM Temp: %" PRIu32 "/255""Voltaje Medido: %.3f V", pot_duty_cycle, temp_celsius, ntc_duty_cycle,ntc_voltaje);
        }

        // Intervalo de espera para el próximo ciclo
        vTaskDelay(pdMS_TO_TICKS(MONITOR_INTERVAL_MS));
    }
}