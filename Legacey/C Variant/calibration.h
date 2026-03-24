#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdbool.h>
#include "attitude.h"

bool calibrate_gyro(AttitudeState *state, int sample_count);

#endif // CALIBRATION_H
