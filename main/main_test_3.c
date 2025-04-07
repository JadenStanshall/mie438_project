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

#define DELAY_US      5500
#define MIN_DELAY_US  800
#define MAX_DELAY_US  5500

#define MIN_PID       0.6
#define MAX_PID       25
// min speed = 5500
// max speed = 800

#define STEPS_PER_REV 400 // full = 200, half = 400

static const char *TAG = "IMU TEST";

void app_main(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DIR_PIN) | (1ULL << STEP_PIN) | (1ULL << MS1_PIN) | (1ULL << MS2_PIN) | (1ULL << MS3_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // config half step
    gpio_set_level(MS1_PIN, 1);
    gpio_set_level(MS2_PIN, 0);
    gpio_set_level(MS3_PIN, 0);

    gpio_set_level(DIR_PIN, 1);  // Set fixed direction

    float rpm = (60.0f * 1000000.0f) / (STEPS_PER_REV * 2.0f * DELAY_US);
    ESP_LOGI(TAG, "Running motor at delay %d us (%.2f RPM)", DELAY_US, rpm);

    imu_init();
    mpu6050_acce_value_t accel;

    float KP = 100;

    int prev_filtered_motor_input = 0;

    while (1) {

        imu_read(&accel);

        float angle_rad = atan2f(accel.acce_z, accel.acce_y);

        float err = 0.04 - angle_rad;
        float abs_err = fabs(err);
        // assume operation within +/- 0.5 rad

        if (err >= 0) {
            gpio_set_level(DIR_PIN, 1);  // Clockwise
        } else {
            gpio_set_level(DIR_PIN, 0);  // Counter-clockwise
        }


        float pid_output = abs_err * KP;
        // min pid out = 0.6
        // max pid out = 25

        int motor_input = (int)round(MAX_DELAY_US - (((pid_output - MIN_PID) * (MAX_DELAY_US - MIN_DELAY_US)) / (MAX_PID - MIN_PID)));

        float alpha = 0.5;

        if (motor_input > MAX_DELAY_US){
            motor_input = MAX_DELAY_US;
        }

        if (motor_input < MIN_DELAY_US){
            motor_input = MIN_DELAY_US;
        }

        int filtered_motor_input = alpha * motor_input + (1 - alpha) * prev_filtered_motor_input;
        prev_filtered_motor_input = filtered_motor_input;

        ESP_LOGI(TAG, "Angle: %.4f | Error: %.4f | PID Out: %.4f | Motor In: %d", angle_rad, err, pid_output, filtered_motor_input);
    

        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(filtered_motor_input);
        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(filtered_motor_input);

        

    }
}
