#ifndef IMU_H
#define IMU_H

#include "esp_err.h"
#include "mpu6050.h"

/**
 * @brief Initializes the IMU (MPU6050).
 */
void imu_init();

/**
 * @brief Reads accelerometer, gyroscope, and temperature data.
 *
 * @param accel Pointer to store accelerometer values
 * @param gyro Pointer to store gyroscope values
 * @param temp Pointer to store temperature value
 */
void imu_read(mpu6050_acce_value_t *accel);

#endif // IMU_H
