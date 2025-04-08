#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 25
#define I2C_MASTER_SDA_IO 26
#define I2C_MASTER_FREQ_HZ 400000

#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

#define DIR_PIN 32
#define STEP_PIN 33

#define MS1_PIN 12
#define MS2_PIN 14
#define MS3_PIN 27

#define MOTOR_ENABLE 13

#define MIN_PID 200
#define MAX_PID 3000

#define MIN_DELAY 700
#define MAX_DELAY 5500

static const char *TAG = "main";

// PID parameters
const double setpoint = 89.5;
double input = 0.0;
double output = 0.0;
int step_delay = 0;
double Kp = 15.0, Ki = 0.05, Kd = 0.001;

double Ap = 400, Ai = 0, Ad = 0.2;
double threshold = 5;

double previous_angle = setpoint;

double lastError = 0.0;
double integral = 0.0;

// Motor
int pid_output = 0;

void i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// median filter package
#define MEDIAN_WINDOW_SIZE 45
#define SPIKE_THRESHOLD 5.0f // Adjust based on real-world behavior

float angle_buffer[MEDIAN_WINDOW_SIZE] = {0};
int angle_index = 0;

float get_median(float *arr, int size)
{
    float temp[MEDIAN_WINDOW_SIZE];
    memcpy(temp, arr, sizeof(float) * size);

    // Simple bubble sort
    for (int i = 0; i < size - 1; ++i)
    {
        for (int j = 0; j < size - i - 1; ++j)
        {
            if (temp[j] > temp[j + 1])
            {
                float tmp = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = tmp;
            }
        }
    }

    return temp[size / 2];
}

void mpu6050_init()
{
    uint8_t wake_data[2] = {MPU6050_PWR_MGMT_1, 0};
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, wake_data, 2, 1000 / portTICK_PERIOD_MS);
}

int16_t read_raw_data(uint8_t reg)
{
    uint8_t data[2];
    i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, data, 2, 1000 / portTICK_PERIOD_MS);
    return (int16_t)(data[0] << 8 | data[1]);
}

double get_angle()
{
    int16_t accel_y = read_raw_data(MPU6050_ACCEL_XOUT_H + 2);
    int16_t accel_z = read_raw_data(MPU6050_ACCEL_XOUT_H + 4);
    return atan2((double)accel_z, (double)accel_y) * 180.0 / M_PI;
}

typedef struct
{
    float Q; // process noise covariance
    float R; // measurement noise covariance
    float x; // estimated value
    float P; // estimation error covariance
    float K; // Kalman gain
} KalmanFilter;

void kalman_init(KalmanFilter *kf, float q, float r, float initial_value)
{
    kf->Q = q;
    kf->R = r;
    kf->x = initial_value;
    kf->P = 1.0;
    kf->K = 0.0;
}

float kalman_update(KalmanFilter *kf, float measurement)
{
    // Prediction update
    kf->P += kf->Q;

    // Measurement update
    kf->K = kf->P / (kf->P + kf->R);
    kf->x += kf->K * (measurement - kf->x);
    kf->P *= (1 - kf->K);

    return kf->x;
}

KalmanFilter angle_filter;

bool sameSign(int a, int b)
{
    return (a >= 0 && b >= 0) || (a < 0 && b < 0);
}

void pid_update(double current_angle)
{
    double error = setpoint - current_angle;

    // clamp error to between 150 and 50% of the previous error
    if (fabs(error) < fabs(lastError) / 1.5)
    {
        error = lastError * 0.5;
    }
    else if (fabs(error) > fabs(lastError) * 1.5)
    {
        error = lastError * 2;
    }

    // zero crossing integral reset
    if (sameSign(error, lastError) == false)
    {
        integral = 0;
    }

    integral += error;
    double derivative = error - lastError;

    if (fabs(error) < threshold)
    {
        output = Kp * error + Ki * integral + Kd * derivative;
    }
    else
    {
        output = Ap * error + Ai * integral + Ad * derivative;
    }

    lastError = error;
    pid_output = output;
}

int pid_to_delay(double pid_output)
{
    if (fabs(pid_output) < MIN_PID)
    {
        return MAX_DELAY;
    }
    double delay = MAX_DELAY - ((fabs(pid_output) - MIN_PID) / (MAX_PID - MIN_PID) * (MAX_DELAY - MIN_DELAY));
    return (int)delay;
}

void stop_motor()
{
    gpio_set_level(MOTOR_ENABLE, 1); // Assuming 1 disables the motor (active low logic)
}

void stepper_task(void *arg)
{

    while (1)
    {
        gpio_set_level(MOTOR_ENABLE, 0);
        // Check if PID output is greater than zero (non-zero movement)
        if (abs(pid_output) > 0)
        {
            // Set the direction based on PID output (positive or negative)
            gpio_set_level(DIR_PIN, pid_output > 0 ? 1 : 0);

            // Calculate the delay based on PID output
            step_delay = pid_to_delay(pid_output);

            if (step_delay > (MAX_DELAY - 400))
            {
                stop_motor();
                continue;
            }

            if (step_delay < MIN_DELAY)
            {
                // Stop motor if the delay is below the minimum
                step_delay = MIN_DELAY;
            }

            // Generate the step pulse for the stepper motor
            gpio_set_level(STEP_PIN, 1);
            esp_rom_delay_us(step_delay);
            gpio_set_level(STEP_PIN, 0);
            esp_rom_delay_us(step_delay);
        }
    }
}

void pid_task(void *arg)
{
    while (1)
    {
        double raw_angle = get_angle();

        // Add to median buffer
        angle_buffer[angle_index] = raw_angle;
        angle_index = (angle_index + 1) % MEDIAN_WINDOW_SIZE;

        if (fabs(raw_angle - previous_angle) > SPIKE_THRESHOLD)
        {
            raw_angle = previous_angle;
        }

        // Compute median
        double filtered_angle = get_median(angle_buffer, MEDIAN_WINDOW_SIZE);

        filtered_angle = kalman_update(&angle_filter, raw_angle);
        previous_angle = filtered_angle;
        pid_update(filtered_angle);
        ESP_LOGI(TAG, "Raw_angle: %.2f, Angle: %.2f, PID Output: %d, Delay: %d", raw_angle, filtered_angle, pid_output, step_delay);
        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

void app_main()
{
    i2c_master_init();
    mpu6050_init();

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DIR_PIN) | (1ULL << STEP_PIN) | (1ULL << MS1_PIN) | (1ULL << MS2_PIN) | (1ULL << MS3_PIN) | (1ULL << MOTOR_ENABLE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};

    gpio_config(&io_conf);

    // config half step
    gpio_set_level(MS1_PIN, 0);
    gpio_set_level(MS2_PIN, 1);
    gpio_set_level(MS3_PIN, 0);
    // full step: 000
    // half step: 100
    // quarter step: 010

    kalman_init(&angle_filter, 1, 1, 0.0); // Tune Q and R as needed

    xTaskCreatePinnedToCore(pid_task, "pid_task", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(stepper_task, "stepper_task", 2048, NULL, 1, NULL, 1);
}
