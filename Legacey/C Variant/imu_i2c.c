#include "imu_i2c.h"

void imu_i2c_init(void) {
    i2c_init(IMU_I2C_PORT, IMU_I2C_BAUDRATE);

    gpio_set_function(IMU_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_I2C_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(IMU_I2C_SDA_PIN);
    gpio_pull_up(IMU_I2C_SCL_PIN);
}

bool imu_scan_for_mpu(void) {
    uint8_t dummy = 0;
    int res = i2c_write_blocking(IMU_I2C_PORT,
                                 MPU6050_ADDR,
                                 &dummy,
                                 1,
                                 true); // no stop
    return res >= 0;
}
