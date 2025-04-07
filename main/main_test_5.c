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

#define MOTOR_ENABLE    13

#define MIN_PID         1
#define MAX_PID         4000

#define MIN_DELAY       70
#define MAX_DELAY       90

static const char *TAG = "main";

// Setpoint and PID variables
const double setpoint = 89.30;
double output = 0.0;
int step_delay = 0;

// Default PID gains (will be updated based on error magnitude)
double Kp = 80.0, Ki = 0.2, Kd = 20.0;
double previous_angle = setpoint;
double lastError = 0.0;
double integral = 0.0;
int pid_output = 0;

// Gain scheduling function: select fixed gains based on error magnitude
void update_pid_gains(double error) {
    double abs_error = fabs(error);

    if (abs_error > 10.0) {
        // Large error: more aggressive control
        Kp = 150.0;
        Ki = 1.0;
        Kd = 30.0;
    } else if (abs_error > 5.0) {
        // Moderate error: moderate gains
        Kp = 100.0;
        Ki = 0.5;
        Kd = 25.0;
    } else {
        // Small error: gentle control
        Kp = 80.0;
        Ki = 0.2;
        Kd = 20.0;
    }
}

// I2C initialization
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

// Median filter definitions
#define MEDIAN_WINDOW_SIZE 11
#define SPIKE_THRESHOLD 5.0f

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

// MPU6050 initialization
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

// PID update function using gain scheduling
void pid_update(double current_angle) {
    double error = setpoint - current_angle;
    
    // Deadband: if error is within ±2.0, hold position and do not update
    if (fabs(error) < 2.0) {
        output = 0.0;
        pid_output = 0;
        return;
    }

    // Update PID gains based on error magnitude
    update_pid_gains(error);

    integral += error;
    double derivative = error - lastError;
    output = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;
    pid_output = output;
}

// Convert PID output to a delay value for stepper motor stepping
int pid_to_delay(double pid_output){
    if (fabs(pid_output) < MIN_PID){
        return MAX_DELAY;
    }
    double delay = MAX_DELAY - ((fabs(pid_output) - MIN_PID) / (MAX_PID - MIN_PID) * (MAX_DELAY - MIN_DELAY));
    return (int)delay;
}

// Disable the motor (assumes active low enable)
void stop_motor() {
    gpio_set_level(MOTOR_ENABLE, 1);
}

// Stepper task: generate motor steps based on PID output
void stepper_task(void *arg) {
    // Enable the motor driver (active low logic)
    gpio_set_level(MOTOR_ENABLE, 0);
    while (1) {
        if (abs(pid_output) > 0) {
            // Set direction based on PID output
            gpio_set_level(DIR_PIN, pid_output > 0 ? 1 : 0);
            step_delay = pid_to_delay(pid_output);
            if (step_delay > MAX_DELAY) {
                stop_motor();
            }
            if (step_delay < MIN_DELAY) {
                step_delay = MIN_DELAY;
            }
            // Generate step pulse
            gpio_set_level(STEP_PIN, 1);
            esp_rom_delay_us(step_delay);
            gpio_set_level(STEP_PIN, 0);
            esp_rom_delay_us(step_delay);
        }
    }
}

// PID task: read sensor, filter angle, update PID, and log data
void pid_task(void *arg) {
    while (1) {
        double raw_angle = get_angle();
        // Store reading for median filter
        angle_buffer[angle_index] = raw_angle;
        angle_index = (angle_index + 1) % MEDIAN_WINDOW_SIZE;
        
        // Spike filtering: if the reading jumps too much, use the previous value
        if (fabs(raw_angle - previous_angle) > SPIKE_THRESHOLD) {
            raw_angle = previous_angle;
        }
        
        // Apply median filtering for a more stable reading
        double filtered_angle = get_median(angle_buffer, MEDIAN_WINDOW_SIZE);
        previous_angle = filtered_angle;
        
        pid_update(filtered_angle);
        ESP_LOGI(TAG, "Raw_angle: %.2f, Filtered: %.2f, PID Output: %d, Delay: %d, Kp: %.2f, Ki: %.2f, Kd: %.2f", 
                 raw_angle, filtered_angle, pid_output, step_delay, Kp, Ki, Kd);
        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

void app_main() {
    i2c_master_init();
    mpu6050_init();
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DIR_PIN) | (1ULL << STEP_PIN) | (1ULL << MS1_PIN) | 
                        (1ULL << MS2_PIN) | (1ULL << MS3_PIN) | (1ULL << MOTOR_ENABLE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Configure microstepping mode (verify with your driver datasheet)
    gpio_set_level(MS1_PIN, 1);
    gpio_set_level(MS2_PIN, 0);
    gpio_set_level(MS3_PIN, 0);
    
    xTaskCreatePinnedToCore(pid_task, "pid_task", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(stepper_task, "stepper_task", 2048, NULL, 1, NULL, 1);
}
