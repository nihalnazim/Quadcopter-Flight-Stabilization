#include "calibration.h"
#include "mpu6050.h"
#include "pico/stdlib.h"
#include <stdio.h>

bool calibrate_gyro(AttitudeState *state, int sample_count) {
    int64_t sum_x = 0;
    int64_t sum_y = 0;
    int64_t sum_z = 0;

    printf("Starting gyro calibration... keep the quad still.\n");

    for (int i = 0; i < sample_count; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        if (!mpu6050_read_raw(&ax, &ay, &az, &gx, &gy, &gz)) {
            printf("IMU read failed during calibration.\n");
            return false;
        }

        sum_x += gx;
        sum_y += gy;
        sum_z += gz;

        sleep_ms(2);
    }

    float avg_x = (float)sum_x / sample_count;
    float avg_y = (float)sum_y / sample_count;
    float avg_z = (float)sum_z / sample_count;

    const float GYRO_SENS = 65.5f;

    float bias_x = avg_x / GYRO_SENS;
    float bias_y = avg_y / GYRO_SENS;
    float bias_z = avg_z / GYRO_SENS;

    attitude_set_gyro_bias(state, bias_x, bias_y, bias_z);

    printf("Gyro bias (deg/s): X=%.3f Y=%.3f Z=%.3f\n",
           bias_x, bias_y, bias_z);

    return true;
}
