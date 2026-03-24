#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float roll;
    float pitch;

    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;

    bool calibrated;
} AttitudeState;

void attitude_init(AttitudeState *state);
void attitude_set_gyro_bias(AttitudeState *state,
                            float bx, float by, float bz);

void attitude_update(AttitudeState *state,
                     int16_t ax_raw, int16_t ay_raw, int16_t az_raw,
                     int16_t gx_raw, int16_t gy_raw, int16_t gz_raw,
                     float dt);

#endif // ATTITUDE_H
