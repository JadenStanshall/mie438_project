// main.c
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "motor.h"
#include "imu.h"

#define CONTROL_LOOP_HZ 200
#define DT_SEC (1.0f / CONTROL_LOOP_HZ)
#define ALPHA 0.98f
#define SETPOINT_ANGLE 0.0f
#define Kp  5.0f
#define Ki  0.0f
#define Kd  0.0f

static const char *TAG = "BalancingRobot";

void stepper_task(void *arg)
{
    esp_task_wdt_add(NULL);

    imu_init();
    vTaskDelay(pdMS_TO_TICKS(100));

    float angle = 0;
    float integral = 0;
    float prev_error = 0;

    int64_t last_feed_us = esp_timer_get_time();

    while (1) {
        mpu6050_acce_value_t acc;
        mpu6050_gyro_value_t gyro;
        mpu6050_temp_value_t temp;
        imu_read(&acc, &gyro, &temp);

        float acc_angle = atan2f(acc.acce_z, acc.acce_y) * 180.0f / M_PI;
        float gyro_rate = gyro.gyro_x;

        // float acc_angle = acc.acce_y * 9.81;

        angle = acc_angle;

        // angle = ALPHA * (angle + gyro_rate * DT_SEC) + (1 - ALPHA) * acc_angle;

        float error = SETPOINT_ANGLE - angle;
        // integral += error * DT_SEC;
        float integral = 0.0;
        // float derivative = (error - prev_error) / DT_SEC;
        float derivative = 0.0;
        float output = Kp * error + Ki * integral + Kd * derivative;
        prev_error = error;

        // float rpm = fminf(fabsf(output), 1000.0f);
        float rpm = 10 * fabsf(output);

        #define DEADZONE_THRESHOLD 20.0f

        if (rpm > DEADZONE_THRESHOLD) {
            int direction = (output > 0) ? DIR_CW : DIR_CCW;
            stepper_move(rpm, direction);
}
        ESP_LOGI(TAG, "RPM = %f | ANGLE = %f", rpm, angle);

        if ((esp_timer_get_time() - last_feed_us) > 100000) {
            esp_task_wdt_reset();
            last_feed_us = esp_timer_get_time();
        }

        vTaskDelay(pdMS_TO_TICKS(1000 / CONTROL_LOOP_HZ));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing balancing robot...");
    stepper_init();
    xTaskCreate(stepper_task, "stepper_task", 4096, NULL, 5, NULL);
}
