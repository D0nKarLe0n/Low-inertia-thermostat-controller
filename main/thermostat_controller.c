#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "THERMOSTAT";

// === НАЛАШТУВАННЯ ПІНІВ ===
#define MOTOR_PWM_PIN 18
#define NTC_ADC_CHAN ADC_CHANNEL_4      // GPIO 4
#define PRESSURE_ADC_CHAN ADC_CHANNEL_5 // GPIO 5

// === ПАРАМЕТРИ ТЕРМІСТОРА NTCLE100 ===
const float SERIES_RESISTOR = 10000.0;
const float NOMINAL_RESISTANCE = 10000.0;
const float NOMINAL_TEMPERATURE = 298.15;
const float B_COEFFICIENT = 3977.0;

// === ПАРАМЕТРИ СИСТЕМИ КЕРУВАННЯ ===
float setpoint_temp = 38.0; // Бажана температура
float Kp = 15.0;            // Пропорційний коефіцієнт
float Ki = 0.5;             // Інтегральний коефіцієнт
float Kd = 2.0;             // Диференціальний коефіцієнт
float K_ff = 5.0;           // Коефіцієнт Feed-forward

float integral_error = 0.0;
float previous_error = 0.0;

adc_oneshot_unit_handle_t adc1_handle;

// Ініціалізація апаратної периферії
void hw_init() {
    // 1. Ініціалізація АЦП (Оновлений API для ESP-IDF v5)
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, NTC_ADC_CHAN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PRESSURE_ADC_CHAN, &config));

    // 2. Ініціалізація ШІМ (LEDC)
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .freq_hz          = 5000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_PWM_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

// Лінеаризація Стейнхарта-Харта
float read_temperature() {
    int adc_value = 0;
    adc_oneshot_read(adc1_handle, NTC_ADC_CHAN, &adc_value);
    if (adc_value <= 0) return 0.0;

    float resistance = SERIES_RESISTOR / ((4095.0 / adc_value) - 1.0);
    float steinhart = resistance / NOMINAL_RESISTANCE;
    steinhart = log(steinhart) / B_COEFFICIENT;
    steinhart += 1.0 / NOMINAL_TEMPERATURE;
    return (1.0 / steinhart) - 273.15;
}

// Пряма компенсація тиску
float calculate_feed_forward() {
    int pressure_adc = 0;
    adc_oneshot_read(adc1_handle, PRESSURE_ADC_CHAN, &pressure_adc);
    float pressure_drop = (pressure_adc / 4095.0) * 100.0;
    return pressure_drop * K_ff;
}

// Головний цикл (Замість setup() та loop())
void app_main(void) {
    hw_init();
    ESP_LOGI(TAG, "System Initialized: Low-Inertia Thermostat Controller");

    uint64_t last_time = esp_timer_get_time();

    while (1) {
        uint64_t current_time = esp_timer_get_time();
        float dt = (current_time - last_time) / 1000000.0; // Конвертація мікросекунд у секунди

        if (dt >= 0.01) { // 100 Гц цикл керування
            float current_temp = read_temperature();
            float error = setpoint_temp - current_temp;

            // Feedback: ПІД-алгоритм
            integral_error += error * dt;
            // Anti-windup
            if (integral_error > 50) integral_error = 50;
            if (integral_error < -50) integral_error = -50;

            float derivative = (error - previous_error) / dt;
            float pid_output = (Kp * error) + (Ki * integral_error) + (Kd * derivative);

            // Feed-forward: Додаємо компенсацію тиску
            float ff_output = calculate_feed_forward();
            float total_control_signal = pid_output + ff_output;

            // Обмеження ШІМ
            int pwm_value = (int)fabs(total_control_signal);
            if (pwm_value > 255) pwm_value = 255;

            // Вивід на мотор
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_value);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

            // Логування в консоль (Monitor)
            ESP_LOGI(TAG, "Set:%.1f | Temp:%.1f | PWM:%d", setpoint_temp, current_temp, pwm_value);

            previous_error = error;
            last_time = current_time;
        }
        
        // Віддаємо час планувальнику FreeRTOS, щоб не спрацював Watchdog
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}