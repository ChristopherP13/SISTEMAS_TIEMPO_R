# Nota de aplicación — Ventilador inteligente ESP32

## Arquitectura y características
- ESP32 DevKitC controlando ventilador DC por PWM (LEDC 20 kHz, 8 bits).
- Modos: Manual, Automático, Programado. PIR obligatorio en modos Automático/Programado.
- Interfaz web (HTML/CSS/JS) que consume endpoints JSON del firmware.
- Persistencia en NVS: configuraciones y logs de ejecución.
- SNTP para fecha/hora y para evaluar programación horaria.

## Estructuras clave (fan_control.h)
```c
typedef enum {
    FAN_MODE_MANUAL = 0,   // PWM fijo, sin PIR/temperatura
    FAN_MODE_AUTO,         // PIR requerido, PWM proporcional t_min–t_max
    FAN_MODE_SCHEDULE      // PIR requerido, PWM según registros activos
} fan_mode_t;

typedef struct {
    bool    active;        // Registro habilitado
    uint8_t start_hour;    // 0-23
    uint8_t start_minute;  // 0-59
    uint8_t end_hour;      // 0-23
    uint8_t end_minute;    // 0-59
    uint8_t days_mask;     // bit0=Dom ... bit6=Sáb
    float   temp_0;        // Temp → 0% PWM
    float   temp_100;      // Temp → 100% PWM
} fan_schedule_entry_t;

typedef struct {
    time_t start_ts;   // Epoch inicio de sesión
    time_t end_ts;     // Epoch fin de sesión
    float  temp_min;   // Temperatura mínima observada
    float  temp_max;   // Temperatura máxima observada
    float  max_pwm;    // PWM máximo aplicado (0-100 %)
} fan_run_log_t;

typedef struct {
    fan_mode_t mode;                   // Modo actual
    float manual_pwm;                  // PWM 0-100 % (manual)
    float auto_t_min;                  // Umbral inferior auto
    float auto_t_max;                  // Umbral superior auto
    fan_schedule_entry_t schedules[3]; // Registros programados (hasta 3)
    float current_temp_c;              // Lectura NTC en °C
    bool  pir_active;                  // Estado PIR
    float current_pwm;                 // PWM aplicado 0-100 %
} fan_state_t;
```

## Flujo web → firmware
`main/webpage/app.js` envía y recibe JSON:
```js
// Modo
$.ajax({ url:'/fan/mode', method:'POST',
         data: JSON.stringify({ mode }), contentType:'application/json' });
// PWM manual
$.ajax({ url:'/fan/manual', method:'POST',
         data: JSON.stringify({ pwm }), contentType:'application/json' });
// Auto (umbrales)
$.ajax({ url:'/fan/auto', method:'POST',
         data: JSON.stringify({ t_min:tmin, t_max:tmax }), contentType:'application/json' });
// Programación (registro i)
$.ajax({ url:'/fan/schedule', method:'POST',
         data: JSON.stringify(payload), contentType:'application/json' });
```
Los handlers en `http_server.c` actualizan `s_state` (definido en `fan_control.c`) y llaman a `fan_save_config()` para persistir.

## Persistencia en NVS
Blob `fan_config_blob_t` (`fan_control.c`) guardado en la namespace "storage":
```c
typedef struct {
    fan_mode_t mode;
    float manual_pwm, auto_t_min, auto_t_max;
    fan_schedule_entry_t schedules[3];
    fan_run_log_t logs[FAN_LOG_ENTRIES];
    uint8_t log_start, log_count;
} fan_config_blob_t;
```
Se escribe en `fan_save_config()` y se restaura en `fan_load_config()` al arrancar. Incluye configuraciones y logs.

## Programación (registros)
- Estructura `fan_schedule_entry_t` con inicio/fin, máscara de días, T_0%, T_100%, activo.
- Hasta 3 registros. La UI evita solapes de horario/días antes de enviar.
- En modo Programado: se evalúa el horario y días activos, se requiere PIR=1, luego se mapea la temperatura entre T_0% y T_100% a PWM.

## Lógica de control (control_step)
- Lee PIR en GPIO33.
- Lee NTC en ADC1_CH4 (GPIO32) con divisor NTC→3.3 V, Rref 100 kΩ a GND:
```c
float v = (raw / 4095.0f) * 3.3f;
float r_ntc = Rref * ((Vref - v) / v);
float inv_t = (1/T0) + (1/BETA)*logf(r_ntc/R0);
temp_C = (1/inv_t) - 273.15f;
```
- Modos:
  - Manual: PWM = manual_pwm.
  - Automático: si PIR=1, `map_temp_to_pwm(temp, T_min, T_max)`.
  - Programado: si PIR=1 y horario activo, PWM proporcional con T_0%/T_100% del registro.
- Aplica PWM con LEDC (Timer1 HS, canal 3) y guarda `current_pwm`.

## Logs de ejecución
- Se abre sesión cuando PWM > ~0.5% y se cierra al volver a 0.
- Guarda inicio/fin, temp min/max, PWM máx en `s_logs` (10 entradas, ring buffer con `log_start/log_count`).
- Endpoint `/fan/logs` entrega JSON para la UI (tabla “Últimas ejecuciones”).

## SNTP y fecha/hora
- `wifi_app.c`: tras `IP_EVENT_STA_GOT_IP` arranca SNTP (`esp_sntp_setservername`, `esp_sntp_init`).
- Espera a “System time is set!” antes de usar programación o fechar logs.
- Ajustar TZ con `setenv("TZ", "...", 1); tzset();` (p. ej., CST-5, etc.).

## Pines usados (ESP32 DevKitC)
- GPIO19: PWM ventilador (LEDC HS Timer1 canal 3, 20 kHz).
- GPIO33: PIR (HC-SR501, OUT 3.3 V), pull-down habilitado.
- GPIO32: NTC (ADC1_CH4) divisor 100 kΩ a GND, NTC a 3.3 V.
- GPIO21/22/23: LED RGB de estado (WiFi/app).

## Buenas prácticas y pruebas
- SNTP: verificar hora válida antes de Programado y logs (evita fechas 1969).
- PIR: alimentar HC-SR501 a 5 V (pin VIN/5V de la placa), OUT a GPIO33.
- NTC: al calentar debe subir la temperatura; si baja, revisar divisor/conexión y cálculo.
- Logs: cada ON→OFF genera una entrada (no cada 6 s). La UI consulta `/fan/logs` cada 6 s.
