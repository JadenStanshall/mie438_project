#include "imu.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6050.h"

#define I2C_MASTER_SCL_IO    GPIO_NUM_25  // SCL Pin
#define I2C_MASTER_SDA_IO    GPIO_NUM_26  // SDA Pin
#define I2C_MASTER_NUM       I2C_NUM_0

static const char *TAG = "IMU";
static mpu6050_handle_t mpu;

void imu_init() {
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

    // Create MPU6050 instance
    mpu = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
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

    if (mpu6050_wake_up(mpu) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050");
    }

    ESP_LOGI(TAG, "MPU6050 initialized successfully.");
}

void imu_read(mpu6050_acce_value_t *accel) {
    if (mpu6050_get_acce(mpu, accel) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer data");
    }
}
