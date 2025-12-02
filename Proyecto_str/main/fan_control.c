#include "fan_control.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define FAN_PWM_GPIO        GPIO_NUM_19
#define FAN_PIR_GPIO        GPIO_NUM_33
// NTC en GPIO32 (ADC1_CH4) con divisor 100k (NTC a 3.3V, Rref 100k a GND)
#define FAN_NTC_GPIO        GPIO_NUM_32
#define FAN_NTC_CHANNEL     ADC_CHANNEL_4
#define FAN_NTC_R_REF       100000.0f
#define FAN_NTC_BETA        4190.0f
#define FAN_NTC_R0          100000.0f
#define FAN_NTC_T0_K        298.15f   // 25C en Kelvin

// PWM config (no interfiere con RGB que usa timer0)
#define FAN_LEDC_TIMER      LEDC_TIMER_1
#define FAN_LEDC_MODE       LEDC_HIGH_SPEED_MODE
#define FAN_LEDC_CHANNEL    LEDC_CHANNEL_3
#define FAN_LEDC_FREQ_HZ    20000     // 20 kHz
#define FAN_LEDC_RES        LEDC_TIMER_8_BIT

#define FAN_CONTROL_TASK_STACK   4096
#define FAN_CONTROL_TASK_PRIO    4
#define FAN_CONTROL_TASK_CORE    1

#define ADC_SAMPLES 16

static const char *TAG = "fan_ctrl";

// Config persistida
typedef struct {
    fan_mode_t mode;
    float manual_pwm;
    float auto_t_min;
    float auto_t_max;
    fan_schedule_entry_t schedules[3];
    fan_run_log_t logs[FAN_LOG_ENTRIES];
    uint8_t log_start;  // índice del más antiguo
    uint8_t log_count;  // cantidad válida
} fan_config_blob_t;

static fan_state_t s_state = {
    .mode = FAN_MODE_MANUAL,
    .manual_pwm = 0.0f,
    .auto_t_min = 24.0f,
    .auto_t_max = 30.0f,
};

static fan_run_log_t s_logs[FAN_LOG_ENTRIES];
static uint8_t s_log_start = 0;
static uint8_t s_log_count = 0;

static bool s_initialized = false;
static bool s_session_active = false;
static time_t s_session_start = 0;
static float s_session_temp_min = 1000.0f;
static float s_session_temp_max = -1000.0f;
static float s_session_pwm_max = 0.0f;

static adc_oneshot_unit_handle_t s_adc_unit = NULL;

static void fan_save_log_entry(time_t end_ts)
{
    if (!s_session_active) return;
    fan_run_log_t entry = {
        .start_ts = s_session_start,
        .end_ts = end_ts,
        .temp_min = s_session_temp_min,
        .temp_max = s_session_temp_max,
        .max_pwm = s_session_pwm_max,
    };
    uint8_t pos = (uint8_t)((s_log_start + s_log_count) % FAN_LOG_ENTRIES);
    s_logs[pos] = entry;
    if (s_log_count < FAN_LOG_ENTRIES) {
        s_log_count++;
    } else {
        s_log_start = (uint8_t)((s_log_start + 1) % FAN_LOG_ENTRIES);
    }
    s_session_active = false;
    s_session_temp_min = 1000.0f;
    s_session_temp_max = -1000.0f;
    s_session_pwm_max = 0.0f;
}

static esp_err_t fan_save_config(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }
    fan_config_blob_t blob = {
        .mode = s_state.mode,
        .manual_pwm = s_state.manual_pwm,
        .auto_t_min = s_state.auto_t_min,
        .auto_t_max = s_state.auto_t_max,
        .log_start = s_log_start,
        .log_count = s_log_count,
    };
    memcpy(blob.schedules, s_state.schedules, sizeof(blob.schedules));
    memcpy(blob.logs, s_logs, sizeof(s_logs));

    err = nvs_set_blob(nvs, "fan_cfg", &blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }
    nvs_close(nvs);
    return err;
}

static esp_err_t fan_load_config(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return err;
    }
    size_t required = 0;
    err = nvs_get_blob(nvs, "fan_cfg", NULL, &required);
    if (err != ESP_OK || required != sizeof(fan_config_blob_t)) {
        nvs_close(nvs);
        return ESP_FAIL;
    }
    fan_config_blob_t blob;
    err = nvs_get_blob(nvs, "fan_cfg", &blob, &required);
    if (err == ESP_OK) {
        s_state.mode = blob.mode;
        s_state.manual_pwm = blob.manual_pwm;
        s_state.auto_t_min = blob.auto_t_min;
        s_state.auto_t_max = blob.auto_t_max;
        memcpy(s_state.schedules, blob.schedules, sizeof(s_state.schedules));
        memcpy(s_logs, blob.logs, sizeof(s_logs));
        s_log_start = blob.log_start;
        s_log_count = blob.log_count <= FAN_LOG_ENTRIES ? blob.log_count : FAN_LOG_ENTRIES;
    }
    nvs_close(nvs);
    return err;
}

static float clamp_pct(float v)
{
    if (v < 0.0f) return 0.0f;
    if (v > 100.0f) return 100.0f;
    return v;
}

static void fan_apply_pwm(float pwm_pct)
{
    float pct = clamp_pct(pwm_pct);
    uint32_t duty = (uint32_t)((pct / 100.0f) * ((1 << 8) - 1));
    ledc_set_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL, duty);
    ledc_update_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL);
    s_state.current_pwm = pct;
}

static float read_temperature_c(void)
{
    if (!s_adc_unit) return 25.0f;
    int acc = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        int val = 0;
        if (adc_oneshot_read(s_adc_unit, FAN_NTC_CHANNEL, &val) == ESP_OK) {
            acc += val;
        }
    }
    float raw = (float)acc / ADC_SAMPLES;
    float v = (raw / 4095.0f) * 3.3f; // Vref 3.3V aproximada
    float vref = 3.3f;
    if (v <= 0.01f || v >= vref - 0.01f) {
        return 25.0f; // fuera de rango, valor seguro
    }
    float r_ntc = FAN_NTC_R_REF * (v / (vref - v));
    float inv_t = (1.0f / FAN_NTC_T0_K) + (1.0f / FAN_NTC_BETA) * logf(r_ntc / FAN_NTC_R0);
    float t_k = 1.0f / inv_t;
    return t_k - 273.15f;
}

static bool read_pir(void)
{
    int level = gpio_get_level(FAN_PIR_GPIO);
    return level > 0;
}

static bool is_schedule_active(const fan_schedule_entry_t *entry, const struct tm *now_tm)
{
    if (!entry->active || !now_tm) {
        return false;
    }
    int wday = now_tm->tm_wday; // 0 = domingo
    if (((entry->days_mask >> wday) & 0x1) == 0) {
        return false;
    }
    int start = entry->start_hour * 60 + entry->start_minute;
    int end   = entry->end_hour   * 60 + entry->end_minute;
    int now   = now_tm->tm_hour * 60 + now_tm->tm_min;

    if (start <= end) {
        return now >= start && now <= end;
    }
    return (now >= start) || (now <= end);
}

static float map_temp_to_pwm(float temp_c, float t0, float t100)
{
    if (t100 <= t0) {
        return (temp_c >= t0) ? 100.0f : 0.0f;
    }
    if (temp_c <= t0) return 0.0f;
    if (temp_c >= t100) return 100.0f;
    float span = t100 - t0;
    return (temp_c - t0) * 100.0f / span;
}

static void control_step(void)
{
    s_state.pir_active = read_pir();
    s_state.current_temp_c = read_temperature_c();

    float pwm = 0.0f;

    if (s_state.mode == FAN_MODE_MANUAL) {
        pwm = s_state.manual_pwm;
    } else {
        if (!s_state.pir_active) {
            pwm = 0.0f;
        } else if (s_state.mode == FAN_MODE_AUTO) {
            pwm = map_temp_to_pwm(s_state.current_temp_c, s_state.auto_t_min, s_state.auto_t_max);
        } else if (s_state.mode == FAN_MODE_SCHEDULE) {
            time_t now = 0;
            struct tm timeinfo = {0};
            bool has_time = false;
            time(&now);
            if (localtime_r(&now, &timeinfo) && timeinfo.tm_year > (2020 - 1900)) {
                has_time = true;
            }
            if (!has_time) {
                pwm = 0.0f;
            } else {
                for (int i = 0; i < 3; i++) {
                    if (is_schedule_active(&s_state.schedules[i], &timeinfo)) {
                        pwm = map_temp_to_pwm(s_state.current_temp_c,
                                              s_state.schedules[i].temp_0,
                                              s_state.schedules[i].temp_100);
                        break;
                    }
                }
            }
        }
    }

    // Sesiones de log
    if (pwm > 0.5f) {
        if (!s_session_active) {
            time(&s_session_start);
            s_session_temp_min = s_state.current_temp_c;
            s_session_temp_max = s_state.current_temp_c;
            s_session_pwm_max = pwm;
            s_session_active = true;
        } else {
            if (s_state.current_temp_c < s_session_temp_min) s_session_temp_min = s_state.current_temp_c;
            if (s_state.current_temp_c > s_session_temp_max) s_session_temp_max = s_state.current_temp_c;
            if (pwm > s_session_pwm_max) s_session_pwm_max = pwm;
        }
    } else {
        if (s_session_active) {
            time_t end_ts = 0;
            time(&end_ts);
            fan_save_log_entry(end_ts);
            fan_save_config();
        }
    }

    fan_apply_pwm(pwm);
}

static void fan_control_task(void *arg)
{
    ESP_LOGI(TAG, "fan control task started");
    while (1) {
        control_step();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static esp_err_t fan_gpio_pwm_init(void)
{
    // PIR
    gpio_config_t pir_cfg = {
        .pin_bit_mask = 1ULL << FAN_PIR_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&pir_cfg));

    // ADC config (oneshot)
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_unit));
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12, // hasta ~3.3 V
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_unit, FAN_NTC_CHANNEL, &chan_cfg));

    // PWM timer
    ledc_timer_config_t timer = {
        .duty_resolution = FAN_LEDC_RES,
        .freq_hz = FAN_LEDC_FREQ_HZ,
        .speed_mode = FAN_LEDC_MODE,
        .timer_num = FAN_LEDC_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch = {
        .channel = FAN_LEDC_CHANNEL,
        .duty = 0,
        .gpio_num = FAN_PWM_GPIO,
        .speed_mode = FAN_LEDC_MODE,
        .hpoint = 0,
        .timer_sel = FAN_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));

    return ESP_OK;
}

esp_err_t fan_control_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    if (fan_load_config() != ESP_OK) {
        ESP_LOGW(TAG, "No fan config found, using defaults");
        // Inicializar days_mask por defecto (todos los días activos)
        for (int i = 0; i < 3; i++) {
            s_state.schedules[i].days_mask = 0x7F;
        }
        fan_save_config();
    }

    ESP_ERROR_CHECK(fan_gpio_pwm_init());

    control_step();

    BaseType_t res = xTaskCreatePinnedToCore(fan_control_task, "fan_control_task",
                                             FAN_CONTROL_TASK_STACK, NULL,
                                             FAN_CONTROL_TASK_PRIO, NULL,
                                             FAN_CONTROL_TASK_CORE);
    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create fan_control_task");
        return ESP_FAIL;
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t fan_control_set_mode(fan_mode_t mode)
{
    if (mode < FAN_MODE_MANUAL || mode > FAN_MODE_SCHEDULE) {
        return ESP_ERR_INVALID_ARG;
    }
    s_state.mode = mode;
    return fan_save_config();
}

esp_err_t fan_control_set_manual_pwm(float pwm_pct)
{
    s_state.manual_pwm = clamp_pct(pwm_pct);
    return fan_save_config();
}

esp_err_t fan_control_set_auto_thresholds(float t_min, float t_max)
{
    s_state.auto_t_min = t_min;
    s_state.auto_t_max = t_max;
    return fan_save_config();
}

esp_err_t fan_control_set_schedule(uint8_t index, const fan_schedule_entry_t *entry)
{
    if (!entry || index >= 3) {
        return ESP_ERR_INVALID_ARG;
    }
    s_state.schedules[index] = *entry;
    return fan_save_config();
}

const fan_state_t *fan_control_get_state(void)
{
    return &s_state;
}

const fan_run_log_t *fan_control_get_logs(size_t *count, uint8_t *start_index)
{
    if (count) *count = s_log_count;
    if (start_index) *start_index = s_log_start;
    return s_logs;
}
