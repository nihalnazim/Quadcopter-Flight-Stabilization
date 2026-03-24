#include "control.h"
#include <math.h>

static float clamp(float x, float min, float max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

static float pd_update(PDController *pd, float error, float dt) {
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - pd->prev_error) / dt;
    }
    pd->prev_error = error;
    return pd->kp * error + pd->kd * derivative;
}

void control_init(ControlState *ctrl) {
    ctrl->setpoint_roll  = 0.0f;
    ctrl->setpoint_pitch = 0.0f;

    ctrl->roll.kp  = 3.0f;
    ctrl->roll.kd  = 0.5f;
    ctrl->roll.prev_error = 0.0f;

    ctrl->pitch.kp = 3.0f;
    ctrl->pitch.kd = 0.5f;
    ctrl->pitch.prev_error = 0.0f;

    ctrl->base_throttle = 0.5f;
}

void control_compute(ControlState *ctrl,
                     const AttitudeState *att,
                     float dt,
                     float *m1, float *m2, float *m3, float *m4) {

    float error_roll  = ctrl->setpoint_roll  - att->roll;
    float error_pitch = ctrl->setpoint_pitch - att->pitch;

    float u_roll  = pd_update(&ctrl->roll,  error_roll,  dt);
    float u_pitch = pd_update(&ctrl->pitch, error_pitch, dt);

    float corr_roll  = clamp(u_roll,  -0.2f, 0.2f);
    float corr_pitch = clamp(u_pitch, -0.2f, 0.2f);

    float t = ctrl->base_throttle;

    *m1 = clamp(t + corr_pitch + corr_roll, 0.0f, 1.0f);
    *m2 = clamp(t + corr_pitch - corr_roll, 0.0f, 1.0f);
    *m3 = clamp(t - corr_pitch + corr_roll, 0.0f, 1.0f);
    *m4 = clamp(t - corr_pitch - corr_roll, 0.0f, 1.0f);
}
