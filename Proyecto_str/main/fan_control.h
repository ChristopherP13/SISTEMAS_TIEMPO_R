#ifndef MAIN_FAN_CONTROL_H_
#define MAIN_FAN_CONTROL_H_

#include <stdbool.h>
#include <time.h>
#include "esp_err.h"

// Fan operating modes
typedef enum {
    FAN_MODE_MANUAL = 0,
    FAN_MODE_AUTO,
    FAN_MODE_SCHEDULE
} fan_mode_t;

// Registro de ejecución del ventilador
typedef struct {
    time_t start_ts;      // Epoch inicio
    time_t end_ts;        // Epoch fin
    float  temp_min;      // Temperatura mínima observada
    float  temp_max;      // Temperatura máxima observada
    float  max_pwm;       // Máximo PWM aplicado (0-100)
} fan_run_log_t;

#define FAN_LOG_ENTRIES 10

// One schedule entry (Programado)
typedef struct {
    bool   active;           // true si el registro está habilitado
    uint8_t start_hour;      // 0-23
    uint8_t start_minute;    // 0-59
    uint8_t end_hour;        // 0-23
    uint8_t end_minute;      // 0-59
    uint8_t days_mask;       // bit0=domingo ... bit6=sábado
    float  temp_0;           // Temperatura para 0% PWM
    float  temp_100;         // Temperatura para 100% PWM
} fan_schedule_entry_t;

// Snapshot de estado y configuración
typedef struct {
    fan_mode_t mode;
    float manual_pwm;          // 0-100 %
    float auto_t_min;          // °C
    float auto_t_max;          // °C
    fan_schedule_entry_t schedules[3];
    // Estado instantáneo
    float current_temp_c;
    bool  pir_active;
    float current_pwm;         // 0-100 % aplicado
} fan_state_t;

// Inicializa PWM, GPIOs, NVS y tarea de control
esp_err_t fan_control_init(void);

// Acceso a configuración
esp_err_t fan_control_set_mode(fan_mode_t mode);
esp_err_t fan_control_set_manual_pwm(float pwm_pct);
esp_err_t fan_control_set_auto_thresholds(float t_min, float t_max);
esp_err_t fan_control_set_schedule(uint8_t index, const fan_schedule_entry_t *entry);

// Lee estado actual (puntero válido mientras el módulo esté vivo)
const fan_state_t *fan_control_get_state(void);

// Obtiene logs de ejecución (puntero a buffer interno y cantidad)
const fan_run_log_t *fan_control_get_logs(size_t *count, uint8_t *start_index);

#endif // MAIN_FAN_CONTROL_H_
