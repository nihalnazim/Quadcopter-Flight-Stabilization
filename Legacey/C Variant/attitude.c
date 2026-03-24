#include "attitude.h"
#include <math.h>

#define GYRO_SENSITIVITY   65.5f     // ±500 deg/s
#define ACCEL_SENSITIVITY  8192.0f   // ±4 g
#define COMPLEMENTARY_ALPHA 0.98f

void attitude_init(AttitudeState *state) {
    state->roll = 0.0f;
    state->pitch = 0.0f;

    state->gyro_bias_x = 0.0f;
    state->gyro_bias_y = 0.0f;
    state->gyro_bias_z = 0.0f;

    state->calibrated = false;
}

void attitude_set_gyro_bias(AttitudeState *state,
                            float bx, float by, float bz) {
    state->gyro_bias_x = bx;
    state->gyro_bias_y = by;
    state->gyro_bias_z = bz;
    state->calibrated  = true;
}

void attitude_update(AttitudeState *state,
                     int16_t ax_raw, int16_t ay_raw, int16_t az_raw,
                     int16_t gx_raw, int16_t gy_raw, int16_t gz_raw,
                     float dt) {

    float ax = (float)ax_raw / ACCEL_SENSITIVITY;
    float ay = (float)ay_raw / ACCEL_SENSITIVITY;
    float az = (float)az_raw / ACCEL_SENSITIVITY;

    float gx = (float)gx_raw / GYRO_SENSITIVITY - state->gyro_bias_x;
    float gy = (float)gy_raw / GYRO_SENSITIVITY - state->gyro_bias_y;
    float gz = (float)gz_raw / GYRO_SENSITIVITY - state->gyro_bias_z;
    (void)gz;

    float roll_acc  = atan2f(ay, az) * 180.0f / (float)M_PI;
    float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / (float)M_PI;

    float roll_gyro  = state->roll  + gx * dt;
    float pitch_gyro = state->pitch + gy * dt;

    state->roll  = COMPLEMENTARY_ALPHA * roll_gyro  +
                   (1.0f - COMPLEMENTARY_ALPHA) * roll_acc;

    state->pitch = COMPLEMENTARY_ALPHA * pitch_gyro +
                   (1.0f - COMPLEMENTARY_ALPHA) * pitch_acc;
}
