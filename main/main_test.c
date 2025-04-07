#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>
#include "imu.h"

#define DIR_PIN     32
#define STEP_PIN    33

#define MS1_PIN     12
#define MS2_PIN     14
#define MS3_PIN     27

#define PI          3.14159265

#define KP          3.0f  // Proportional gain for angle

static const char *TAG = "STEPPER";

void app_main(void)
{
    // GPIO config
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DIR_PIN) | (1ULL << STEP_PIN) | (1ULL << MS1_PIN) | (1ULL << MS2_PIN) | (1ULL << MS3_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    gpio_set_level(MS1_PIN, 0);
    gpio_set_level(MS2_PIN, 0);
    gpio_set_level(MS3_PIN, 0);

    // Initialize IMU
    imu_init();
    mpu6050_acce_value_t accel;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;

    while (1) {
        imu_read(&accel, &gyro, &temp);
        
        // Compute tilt angle using atan2(z, y)
        float angle_rad = atan2f(accel.acce_z, accel.acce_y);
        float angle_deg = angle_rad * (180.0f / PI);
        float scaled_angle_deg = 0.00001 * angle_deg;

        float error = 0.0f - scaled_angle_deg;  // target is 0 degrees tilt
        float output = KP * error;

        // Set motor direction based on sign of output
        if (output >= 0) {
            gpio_set_level(DIR_PIN, 1);  // Clockwise
        } else {
            gpio_set_level(DIR_PIN, 0);  // Counter-clockwise
        }

        // Convert output to delay (inverse proportional)
        float abs_output = fabs(output);
        if (abs_output == 0){
            abs_output = 50;
        }

        int delay_us = (int)(1 / abs_output);
        
        ESP_LOGI(TAG, "Angle: %.2f deg, Direction: %s, Delay: %d us, PID Output: %f", angle_deg, output >= 0 ? "CW" : "CCW", delay_us, abs_output);

        delay_us = 4000;

        // Step the motor
        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(delay_us);
        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(delay_us);
    }
}
