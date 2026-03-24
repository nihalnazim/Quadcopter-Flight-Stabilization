#ifndef IMU_I2C_H
#define IMU_I2C_H

#include <stdbool.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"

// imu_i2c.h
#define IMU_I2C_PORT      i2c1
#define IMU_I2C_SDA_PIN   6
#define IMU_I2C_SCL_PIN   7
#define IMU_I2C_BAUDRATE  400000
#define MPU6050_ADDR      0x68

void imu_i2c_init(void);
bool imu_scan_for_mpu(void);

#endif // IMU_I2C_H
