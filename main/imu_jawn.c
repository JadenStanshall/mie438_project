#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6050.h"

#define I2C_MASTER_SCL_IO    GPIO_NUM_22  // SCL Pin
#define I2C_MASTER_SDA_IO    GPIO_NUM_21  // SDA Pin
#define I2C_MASTER_NUM       I2C_NUM_0

static const char *TAG = "MPU6050";

typedef enum {
    MPU6050_ACCEL_FS_2G  = 0,   /*!< Accelerometer full scale range is ±2g */
    MPU6050_ACCEL_FS_4G  = 1,   /*!< Accelerometer full scale range is ±4g */
    MPU6050_ACCEL_FS_8G  = 2,   /*!< Accelerometer full scale range is ±8g */
    MPU6050_ACCEL_FS_16G = 3    /*!< Accelerometer full scale range is ±16g */
} mpu6050_accel_fs_t;


void i2c_master_init() {
    ESP_LOGI(TAG, "Initializing I2C...");

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
}


void app_main() {
    i2c_master_init();

    // Create MPU6050 instance
    mpu6050_handle_t mpu = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    if (mpu == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050");
        return;
    }

    // Configure MPU6050
    esp_err_t err = mpu6050_config(mpu, ACCE_FS_2G, GYRO_FS_250DPS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 configuration failed");
        return;
    }

    esp_err_t err_wakeup = mpu6050_wake_up(mpu);
        if (err_wakeup != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
    }


    mpu6050_acce_value_t accel;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;

    while (true) {
        if (mpu6050_get_acce(mpu, &accel) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read accelerometer data");
            continue;
        }
        if (mpu6050_get_gyro(mpu, &gyro) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read gyroscope data");
            continue;
        }
        if (mpu6050_get_temp(mpu, &temp) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read temperature data");
            continue;
        }

        // ✅ Using the correct field names
        ESP_LOGI(TAG, "Accel (G): X=%.2f Y=%.2f Z=%.2f | Gyro (°/s): X=%.2f Y=%.2f Z=%.2f | Temp: %.2f°C",
                 accel.acce_x, accel.acce_y, accel.acce_z,
                 gyro.gyro_x, gyro.gyro_y, gyro.gyro_z,
                 temp.temp);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

