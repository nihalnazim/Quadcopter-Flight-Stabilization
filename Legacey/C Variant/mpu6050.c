#include "mpu6050.h"
#include "imu_i2c.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_INT_ENABLE   0x38
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

static bool mpu6050_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    int res = i2c_write_blocking(IMU_I2C_PORT,
                                 MPU6050_ADDR,
                                 buf,
                                 2,
                                 false);
    return res == 2;
}

static bool mpu6050_read_reg(uint8_t reg, uint8_t *value) {
    int res = i2c_write_blocking(IMU_I2C_PORT,
                                 MPU6050_ADDR,
                                 &reg,
                                 1,
                                 true);
    if (res != 1) return false;

    res = i2c_read_blocking(IMU_I2C_PORT,
                            MPU6050_ADDR,
                            value,
                            1,
                            false);
    return res == 1;
}

static bool mpu6050_read_burst(uint8_t start_reg, uint8_t *buffer, uint8_t length) {
    int res = i2c_write_blocking(IMU_I2C_PORT,
                                 MPU6050_ADDR,
                                 &start_reg,
                                 1,
                                 true);
    if (res != 1) return false;

    res = i2c_read_blocking(IMU_I2C_PORT,
                            MPU6050_ADDR,
                            buffer,
                            length,
                            false);
    return res == length;
}

bool mpu6050_init(void) {
    uint8_t who = 0;
    if (!mpu6050_read_reg(MPU6050_REG_WHO_AM_I, &who)) {
        return false;
    }
    if (who != 0x68) {
        return false;
    }

    if (!mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00)) return false;

    if (!mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV, 7)) return false;

    if (!mpu6050_write_reg(MPU6050_REG_CONFIG, 0x03)) return false;

    if (!mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, 0x08)) return false;

    if (!mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, 0x08)) return false;

    if (!mpu6050_write_reg(MPU6050_REG_INT_ENABLE, 0x00)) return false;

    return true;
}

bool mpu6050_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                      int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t buf[14];
    if (!mpu6050_read_burst(MPU6050_REG_ACCEL_XOUT_H, buf, 14)) {
        return false;
    }

    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);

    *gx = (int16_t)((buf[8]  << 8) | buf[9]);
    *gy = (int16_t)((buf[10] << 8) | buf[11]);
    *gz = (int16_t)((buf[12] << 8) | buf[13]);

    return true;
}
