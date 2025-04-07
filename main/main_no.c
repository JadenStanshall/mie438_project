// #include "imu.h"
// #include "esp_log.h"

// #define TEST_IMU

// // FOR IMU
// static const char *TAG = "MAIN";

// #ifdef TEST_IMU

// void app_main() {
//     ESP_LOGI(TAG, "Starting IMU Test...");
    
//     imu_init();

//     mpu6050_acce_value_t accel;
//     mpu6050_gyro_value_t gyro;
//     mpu6050_temp_value_t temp;

//     while (1) {
//         imu_read(&accel, &gyro, &temp);
        
//         ESP_LOGI(TAG, "Accel (G): X=%.2f Y=%.2f Z=%.2f | Gyro (°/s): X=%.2f Y=%.2f Z=%.2f | Temp: %.2f°C",
//                  accel.acce_x, accel.acce_y, accel.acce_z,
//                  gyro.gyro_x, gyro.gyro_y, gyro.gyro_z,
//                  temp.temp);

//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
// }
// #endif


// FOR MOTOR
// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "motor.h"

// void app_main() {
//     motor_init();  // Initialize motor driver

//     vTaskDelay(pdMS_TO_TICKS(2000));  // Wait before starting

//     printf("Testing stepper motor...\n");

//     // Run manual step test
//     test_manual_step();
//     vTaskDelay(pdMS_TO_TICKS(3000));

//     while (1) {
//         printf("Rotating forward...\n");
//         step_motor(STEPS_PER_REV, 1, 50);  // Rotate forward
//         vTaskDelay(pdMS_TO_TICKS(1000)); 

//         printf("Rotating backward...\n");
//         step_motor(STEPS_PER_REV, 0, 50);  // Rotate backward
//         vTaskDelay(pdMS_TO_TICKS(1000)); 
//     }
// }

// SMOOTH MOTOR TEST
// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "motor.h"

// #define MAX_SPEED_DELAY  50   // Fastest step delay (smaller = faster)
// #define MIN_SPEED_DELAY  2000 // Slowest step delay (larger = slower)
// #define ACCELERATION_STEP  10  // How much to change delay per cycle

// void speedTestMotor() {
//     int stepDelay = MIN_SPEED_DELAY;  // Start slow
//     int direction = 1;  // Start moving forward

//     while (1) {
//         // **Phase 1: Accelerate (Decrease delay)**
//         printf("Accelerating...\n");
//         while (stepDelay > MAX_SPEED_DELAY) {
//             step_motor(1, direction, stepDelay);
//             stepDelay -= ACCELERATION_STEP;
//             vTaskDelay(pdMS_TO_TICKS(2));  // Small pause to control acceleration smoothness
//         }

//         // **Phase 2: Hold Max Speed**
//         printf("Holding max speed...\n");
//         for (int i = 0; i < 500; i++) {
//             step_motor(1, direction, MAX_SPEED_DELAY);
//         }

//         // **Phase 3: Decelerate (Increase delay)**
//         printf("Decelerating...\n");
//         while (stepDelay < MIN_SPEED_DELAY) {
//             step_motor(1, direction, stepDelay);
//             stepDelay += ACCELERATION_STEP;
//             vTaskDelay(pdMS_TO_TICKS(2));
//         }

//         // **Phase 4: Hold Min Speed**
//         printf("Holding min speed...\n");
//         for (int i = 0; i < 200; i++) {
//             step_motor(1, direction, MIN_SPEED_DELAY);
//         }

//         // Change direction each cycle (like a wheel reversing)
//         direction = !direction;
//     }
// }

// void app_main() {
//     motor_init();  // Initialize stepper motor driver
//     vTaskDelay(pdMS_TO_TICKS(2000));  // Wait before starting

//     printf("Starting motor speed test...\n");

//     speedTestMotor();  // Run speed test sequence
// }

// PRETTY GOOD MOTOR CODE
// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "motor.h"
// #include "imu.h"
// #include "esp_log.h"
// #include "math.h"

// // PID Constants (TUNE THESE!)
// float Kp = 15.0;  // Proportional Gain
// float Ki = 0.5;   // Integral Gain
// float Kd = 10.0;  // Derivative Gain

// // PID variables
// float error = 0, prevError = 0, integral = 0;
// float desiredAngle = 0.0;  // Setpoint (upright balance)
// static const char *TAG = "PID_CONTROLLER";

// // Motor speed variables
// volatile int stepDelay = 1000;  // Start with a safe delay (lower = faster)
// volatile int direction = 1;      // Default motor direction

// // Function to get tilt angle from IMU
// float getTiltAngle() {
//     mpu6050_acce_value_t accel;
//     mpu6050_gyro_value_t gyro;
//     mpu6050_temp_value_t temp;

//     imu_read(&accel, &gyro, &temp);

//     ESP_LOGI(TAG, "Accel (G): X=%.2f Y=%.2f Z=%.2f | Gyro (°/s): X=%.2f Y=%.2f Z=%.2f | Temp: %.2f°C",
//              accel.acce_x, accel.acce_y, accel.acce_z,
//              gyro.gyro_x, gyro.gyro_y, gyro.gyro_z,
//              temp.temp);

//     // Calculate pitch angle using accelerometer
//     float accelAngle = atan2(accel.acce_y, accel.acce_z) * 180.0 / M_PI;

//     // Use complementary filter for better angle estimation
//     static float angle = 0;
//     angle = 0.98 * (angle + gyro.gyro_x * 0.01) + 0.02 * accelAngle;

//     return angle;
// }

// // PID Controller Function (Calculates velocity, not steps)
// void updateMotorVelocity() {
//     float currentAngle = getTiltAngle();
//     error = desiredAngle - currentAngle;

//     // PID calculations
//     integral += error * 0.01;
//     float derivative = (error - prevError) / 0.01;
//     float controlOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);

//     // Limit control output
//     if (controlOutput > 1000) controlOutput = 1000;
//     if (controlOutput < -1000) controlOutput = -1000;

//     // Convert PID output to motor velocity (step delay)
//     int targetStepDelay = (int)(1000 - fabs(controlOutput));  // Smaller delay = faster
//     if (targetStepDelay < 1) targetStepDelay = 1;  // Prevent divide by zero

//     // Smooth acceleration and deceleration
//     static int prevStepDelay = 1000;
//     int accelerationRate = 10; // Rate of speed change

//     if (targetStepDelay < prevStepDelay) {
//         prevStepDelay -= accelerationRate;  // Speed up
//         if (prevStepDelay < targetStepDelay) prevStepDelay = targetStepDelay;
//     } else {
//         prevStepDelay += accelerationRate;  // Slow down
//         if (prevStepDelay > targetStepDelay) prevStepDelay = targetStepDelay;
//     }

//     stepDelay = prevStepDelay;  // Update global step delay
//     direction = (controlOutput > 0) ? 1 : 0;  // Set motor direction

//     prevError = error;  // Store previous error for derivative calculation
// }

// // Continuous Stepper Motor Task
// void stepperMotorTask(void *arg) {
//     while (1) {
//         gpio_set_level(DIR_PIN, direction);
//         gpio_set_level(STEP_PIN, 1);
//         esp_rom_delay_us(5);  // Short pulse width
//         gpio_set_level(STEP_PIN, 0);
//         esp_rom_delay_us(stepDelay);  // Controlled by PID

//         // Yield to other tasks (important for FreeRTOS)
//         taskYIELD();
//     }
// }

// // Main application task
// void app_main() {
//     ESP_LOGI(TAG, "Initializing system...");
//     motor_init();
//     imu_init();

//     vTaskDelay(pdMS_TO_TICKS(2000));
//     ESP_LOGI(TAG, "Starting self-balancing control...");

//     // Create the motor control task (runs continuously)
//     xTaskCreate(stepperMotorTask, "Stepper Task", 2048, NULL, 1, NULL);

//     // PID update loop
//     while (1) {
//         updateMotorVelocity();  // Adjust motor speed based on PID
//         vTaskDelay(pdMS_TO_TICKS(10));  // Run every 10ms
//     }
// }


// PRETTY GOOD STEPPER JAWNS
// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"
// #include "driver/adc.h"
// #include "esp_adc/adc_oneshot.h"
// #include "esp_log.h"

// // GPIO Definitions
// #define STEP_PIN GPIO_NUM_25   // A4988 Step pin
// #define DIR_PIN GPIO_NUM_26    // A4988 Direction pin
// #define POT_PIN ADC_CHANNEL_6  // ADC1 Channel 6 (GPIO34)

// // Microstepping Configuration (Fixed to 1/4 Step Mode)
// #define MICROSTEPPING 4  // 1/4 Step Mode

// // ADC Configuration
// #define ADC_ATTEN ADC_ATTEN_DB_11  // Full 0-3.3V range
// #define MIN_DELAY_US 1000     // **Increased Max Speed (1000 Hz Step Rate)**
// #define MAX_DELAY_US 20000    // **Better Low-Speed Control**
// #define STOP_THRESHOLD 50     // ADC threshold to stop motor (~0.04V)
// #define RAMP_STEP 250         // **Faster Ramp-up Rate**

// static adc_oneshot_unit_handle_t adc1_handle;
// static const char *TAG = "STEPPER";

// // GPIO Setup
// void setup_gpio() {
//     gpio_config_t io_conf = {
//         .pin_bit_mask = (1ULL << STEP_PIN) | (1ULL << DIR_PIN),
//         .mode = GPIO_MODE_OUTPUT,
//         .pull_up_en = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_DISABLE
//     };
//     gpio_config(&io_conf);

//     // Set default direction
//     gpio_set_level(DIR_PIN, 1);

//     ESP_LOGI(TAG, "GPIO initialized, Fixed 1/4 Step Mode Set in Hardware");
// }

// // ADC Setup
// void setup_adc() {
//     adc_oneshot_unit_init_cfg_t init_config = {
//         .unit_id = ADC_UNIT_1,
//     };
//     adc_oneshot_new_unit(&init_config, &adc1_handle);

//     adc_oneshot_chan_cfg_t config = {
//         .bitwidth = ADC_BITWIDTH_12, // 12-bit resolution
//         .atten = ADC_ATTEN_DB_11
//     };
//     adc_oneshot_config_channel(adc1_handle, POT_PIN, &config);
    
//     ESP_LOGI(TAG, "ADC initialized");
// }

// // Stepper Control Task (Smooth Speed Mapping)
// void stepper_task(void *arg) {
//     int raw_value = 0;
//     int target_delay = MAX_DELAY_US;   // Target step delay
//     int current_delay = MAX_DELAY_US;  // Smoothly changing delay

//     while (1) {
//         // Read potentiometer value
//         adc_oneshot_read(adc1_handle, POT_PIN, &raw_value);
//         ESP_LOGI(TAG, "ADC Value: %d", raw_value);

//         // Stop the motor completely if ADC is below the threshold
//         if (raw_value < STOP_THRESHOLD) {
//             ESP_LOGI(TAG, "Motor Stopped");
//             current_delay = MAX_DELAY_US;  // Reset to slowest speed on next start
//             vTaskDelay(pdMS_TO_TICKS(50)); // Prevent watchdog timeout
//             continue;
//         }

//         // Map ADC Value (50-4095) to Step Delay (1000-20000µs) smoothly
//         float scale_factor = (float)(raw_value - STOP_THRESHOLD) / (4095 - STOP_THRESHOLD);
//         target_delay = MAX_DELAY_US - (scale_factor * (MAX_DELAY_US - MIN_DELAY_US));

//         // Smoothly adjust speed (ramp-up and ramp-down)
//         if (current_delay > target_delay) {
//             current_delay -= RAMP_STEP;  // Speed up
//             if (current_delay < target_delay) current_delay = target_delay;
//         } else if (current_delay < target_delay) {
//             current_delay += RAMP_STEP;  // Slow down
//             if (current_delay > target_delay) current_delay = target_delay;
//         }

//         // Ensure step delay never goes below the minimum
//         if (current_delay < MIN_DELAY_US) current_delay = MIN_DELAY_US;

//         // **Adjust delay for 1/4 microstepping**
//         int microstep_delay = current_delay / MICROSTEPPING;

//         // Step the motor at the adjusted speed
//         for (int i = 0; i < MICROSTEPPING; i++) {  // 4 microsteps per step
//             gpio_set_level(STEP_PIN, 1);
//             esp_rom_delay_us(microstep_delay);
//             gpio_set_level(STEP_PIN, 0);
//             esp_rom_delay_us(microstep_delay);
//         }

//         vTaskDelay(pdMS_TO_TICKS(1));  // Allow FreeRTOS scheduling
//     }
// }

// // Main Entry
// void app_main() {
//     setup_gpio();
//     setup_adc();

//     // Create stepper task
//     xTaskCreate(stepper_task, "stepper_task", 2048, NULL, 5, NULL);
// }

#include "imu.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include <math.h>

// GPIO Definitions
#define STEP_PIN GPIO_NUM_25   // A4988 Step pin
#define DIR_PIN GPIO_NUM_26    // A4988 Direction pin

// PID Tuning Parameters
#define Kp  1.0   // Proportional Gain
#define Ki  0.0    // Integral Gain
#define Kd  0.0    // Derivative Gain

// Balance Setpoint (target upright angle)
#define SETPOINT 0.0
#define MIN_DELAY_US 500    // **Max Speed (~1200 RPM)**
#define MAX_DELAY_US 20000  // **Low Speed (~10 RPM)**
#define STOP_THRESHOLD 0.5  // Angle threshold to stop
#define MICROSTEPPING 4

// IMU Data Structures
mpu6050_acce_value_t accel;
mpu6050_gyro_value_t gyro;
mpu6050_temp_value_t temp;

// PID Variables
float last_error = 0.0;
float integral = 0.0;

// Logging Tag
static const char *TAG = "BALANCE";

// GPIO Setup
void setup_gpio() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STEP_PIN) | (1ULL << DIR_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "GPIO initialized for Stepper Motor");
}

// Reads IMU Data and Computes Tilt Angle Directly
float get_tilt_angle() {
    imu_read(&accel, &gyro, &temp);
    
    // Compute tilt angle using accelerometer directly
    float angle_acc = atan2(accel.acce_y, accel.acce_z) * 180.0 / M_PI;
    
    // Apply a complementary filter (less gyro, more accelerometer)
    static float last_angle = 0.0;
    float angle_filtered = 0.90 * last_angle + 0.10 * angle_acc;
    last_angle = angle_filtered;

    return angle_filtered;
}

// PID Controller Function (Uses Angle Directly)
float pid_compute(float current_angle) {
    float error = SETPOINT - current_angle;  // Tilt deviation
    integral += error;  // Accumulate integral
    float derivative = error - last_error;
    last_error = error;

    // PID Output (Speed Control)
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    return output;
}

// Stepper Control Task (Uses PID Output)
void stepper_control_task(void *arg) {
    float target_delay = MAX_DELAY_US;  // Start at slowest speed

    while (1) {
        float tilt_angle = get_tilt_angle();
        float pid_output = pid_compute(tilt_angle);
        
        ESP_LOGI(TAG, "Tilt: %.2f° /tab PID Output: %.2f", tilt_angle, pid_output);

        // Determine Direction
        if (pid_output > 0) {
            gpio_set_level(DIR_PIN, 1);  // Forward
        } else {
            gpio_set_level(DIR_PIN, 0);  // Reverse
        }

        // Convert PID output to step delay (speed)
        float scale_factor = fabs(pid_output) / 30.0;  // Normalize to ±30° range
        if (scale_factor > 1.0) scale_factor = 1.0;  // Cap to prevent overflow

        target_delay = MAX_DELAY_US - (scale_factor * (MAX_DELAY_US - MIN_DELAY_US));

        // Ensure valid range
        if (target_delay < MIN_DELAY_US) target_delay = MIN_DELAY_US;
        if (target_delay > MAX_DELAY_US) target_delay = MAX_DELAY_US;

        // **Adjust delay for 1/4 microstepping**
        int microstep_delay = target_delay / MICROSTEPPING;

        // Step the motor at the adjusted speed
        for (int i = 0; i < MICROSTEPPING; i++) {
            gpio_set_level(STEP_PIN, 1);
            esp_rom_delay_us(10);
            gpio_set_level(STEP_PIN, 0);
            esp_rom_delay_us(microstep_delay);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Main Entry
void app_main() {
    ESP_LOGI(TAG, "Starting Self-Balancing Robot...");
    
    setup_gpio();
    imu_init();

    // Start Stepper Control Task
    xTaskCreate(stepper_control_task, "stepper_task", 2048, NULL, 5, NULL);
}
