#ifndef CONTROL_H
#define CONTROL_H

#include "attitude.h"

typedef struct {
    float kp;
    float kd;
    float prev_error;
} PDController;

typedef struct {
    PDController roll;
    PDController pitch;

    float setpoint_roll;
    float setpoint_pitch;

    float base_throttle;   // 0.0–1.0
} ControlState;

void control_init(ControlState *ctrl);

// Given attitude + dt, compute 4 motor commands in 0.0–1.0
void control_compute(ControlState *ctrl,
                     const AttitudeState *att,
                     float dt,
                     float *m1, float *m2, float *m3, float *m4);

#endif // CONTROL_H
