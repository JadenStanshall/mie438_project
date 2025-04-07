#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 25
#define I2C_MASTER_SDA_IO 26
#define I2C_MASTER_FREQ_HZ 400000

#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

#define DIR_PIN 32
#define STEP_PIN 33

#define MS1_PIN         12
#define MS2_PIN         14
#define MS3_PIN         27

#define MIN_PID         150
#define MAX_PID         10000

#define MIN_DELAY       1400
#define MAX_DELAY       5700

#define JUMP_THRESH     20

static const char *TAG = "main";

// PID parameters
double setpoint = 6.0;
double input = 0.0;
double output = 0.0;
int step_delay = 0;
double Kp = 400.0, Ki = 0.0, Kd = 0.0;
double previous_angle = 123.456;

double lastError = 0.0;
double integral = 0.0;

// Motor
int pid_output = 0;

void i2c_master_init() {
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
#define MEDIAN_WINDOW_SIZE 1
#define SPIKE_THRESHOLD 20.0f  // Adjust based on real-world behavior

float angle_buffer[MEDIAN_WINDOW_SIZE] = {0};
int angle_index = 0;

float get_median(float *arr, int size) {
    float temp[MEDIAN_WINDOW_SIZE];
    memcpy(temp, arr, sizeof(float) * size);

    // Simple bubble sort
    for (int i = 0; i < size - 1; ++i) {
        for (int j = 0; j < size - i - 1; ++j) {
            if (temp[j] > temp[j + 1]) {
                float tmp = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = tmp;
            }
        }
    }

    return temp[size / 2];
}


void mpu6050_init() {
    uint8_t wake_data[2] = {MPU6050_PWR_MGMT_1, 0};
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, wake_data, 2, 1000 / portTICK_PERIOD_MS);
}

int16_t read_raw_data(uint8_t reg) {
    uint8_t data[2];
    i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, data, 2, 1000 / portTICK_PERIOD_MS);
    return (int16_t)(data[0] << 8 | data[1]);
}

double get_angle() {
    int16_t accel_y = read_raw_data(MPU6050_ACCEL_XOUT_H + 2);
    int16_t accel_z = read_raw_data(MPU6050_ACCEL_XOUT_H + 4);
    return atan2((double)accel_z, (double)accel_y) * 180.0 / M_PI;
}

void pid_update(double current_angle) {
    double error = setpoint - current_angle;
    integral += error;
    double derivative = error - lastError;
    output = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;
    pid_output = output;
}

int pid_to_delay(double pid_output){
    if (fabs(pid_output) < MIN_PID){
        return MAX_DELAY;
    }
    double delay = MAX_DELAY - ((fabs(pid_output) - MIN_PID)/(MAX_PID - MIN_PID) * (MAX_DELAY - MIN_DELAY));
    return (int)delay;
}

void stepper_task(void *arg) {
    while (1) {
        if (abs(pid_output) > 0) {
            gpio_set_level(DIR_PIN, pid_output > 0 ? 1 : 0);
        
        step_delay = pid_to_delay(pid_output);

        gpio_set_level(STEP_PIN, 1);
        esp_rom_delay_us(step_delay);
        gpio_set_level(STEP_PIN, 0);
        esp_rom_delay_us(step_delay);
        }
    }
}

void pid_task(void *arg) {
    while (1) {
        double raw_angle = get_angle();

        // Add to median buffer
        angle_buffer[angle_index] = raw_angle;
        angle_index = (angle_index + 1) % MEDIAN_WINDOW_SIZE;

        // Compute median
        double filtered_angle = get_median(angle_buffer, MEDIAN_WINDOW_SIZE);

        // Optional: Spike rejection after filtering (useful if sensor is really jumpy)
        if (previous_angle != 123.456) {
            if (fabs(previous_angle - filtered_angle) > SPIKE_THRESHOLD) {
                filtered_angle = previous_angle;
            }
        }
        previous_angle = filtered_angle;
        pid_update(raw_angle);
        ESP_LOGI(TAG, "Angle: %.2f, PID Output: %d, Delay: %d", filtered_angle, pid_output, step_delay);
        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

void app_main() {
    i2c_master_init();
    mpu6050_init();

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DIR_PIN) | (1ULL << STEP_PIN) | (1ULL << MS1_PIN) | (1ULL << MS2_PIN) | (1ULL << MS3_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    gpio_config(&io_conf);
    
    // config half step
    gpio_set_level(MS1_PIN, 0);
    gpio_set_level(MS2_PIN, 0);
    gpio_set_level(MS3_PIN, 0);
    

    xTaskCreatePinnedToCore(pid_task, "pid_task", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(stepper_task, "stepper_task", 2048, NULL, 1, NULL, 1);
}
