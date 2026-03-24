#include "safety.h"
#include <math.h>

static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void safety_init(SafetyState *s, const SafetyConfig *cfg, uint32_t now_ms) {
    (void)cfg;
    s->mode = SAFETY_DISARMED;

    s->arm_requested = false;
    s->arm_request_start_ms = 0;
    s->ready_since_ms = 0;

    s->last_imu_ok_ms = now_ms;

    s->vbat = 0.0f;
    s->vbat_warned = false;
    s->vbat_critical = false;
    s->last_vbat_check_ms = now_ms;

    s->armed_throttle_scale = 0.0f;
}

void safety_set_mode(SafetyState *s, SafetyMode m, uint32_t now_ms) {
    s->mode = m;

    if (m == SAFETY_READY) {
        s->ready_since_ms = now_ms;
    }

    if (m == SAFETY_ARMED) {
        s->armed_throttle_scale = 0.0f; // start ramp at 0
    }

    if (m == SAFETY_DISARMED || m == SAFETY_FAILSAFE) {
        s->arm_requested = false;
        s->armed_throttle_scale = 0.0f;
    }
}

void safety_notify_imu_ok(SafetyState *s, uint32_t now_ms) {
    s->last_imu_ok_ms = now_ms;
}

void safety_cmd_arm(SafetyState *s, uint32_t now_ms) {
    if (!s->arm_requested) {
        s->arm_requested = true;
        s->arm_request_start_ms = now_ms;
    }
}

void safety_cmd_disarm(SafetyState *s, uint32_t now_ms) {
    (void)now_ms;
    safety_set_mode(s, SAFETY_DISARMED, now_ms);
}

void safety_battery_update(SafetyState *s, const SafetyConfig *cfg, uint32_t now_ms, float vbat) {
    (void)now_ms;
    s->vbat = vbat;

    if (!cfg->battery_enabled) return;

    if (vbat <= cfg->vbat_crit) {
        s->vbat_critical = true;
    }

    if (vbat <= cfg->vbat_warn) {
        s->vbat_warned = true;
    }
}

bool safety_update(SafetyState *s,
                   const SafetyConfig *cfg,
                   uint32_t now_ms,
                   float roll_deg,
                   float pitch_deg,
                   bool imu_ok,
                   bool battery_critical_now) {

    // Track IMU health
    if (imu_ok) {
        safety_notify_imu_ok(s, now_ms);
    }

    // Hard safety checks (angle)
    if (fabsf(roll_deg) > cfg->max_angle_deg || fabsf(pitch_deg) > cfg->max_angle_deg) {
        safety_set_mode(s, SAFETY_FAILSAFE, now_ms);
    }

    // IMU freeze failsafe
    if ((now_ms - s->last_imu_ok_ms) > cfg->imu_freeze_ms) {
        safety_set_mode(s, SAFETY_FAILSAFE, now_ms);
    }

    // Battery critical failsafe
    if (cfg->battery_enabled && battery_critical_now) {
        safety_set_mode(s, SAFETY_FAILSAFE, now_ms);
    }

    // State machine
    switch (s->mode) {
        case SAFETY_DISARMED:
            // waiting for calibration to finish externally (main will set CALIBRATING then READY)
            break;

        case SAFETY_CALIBRATING:
            // motors disabled
            break;

        case SAFETY_READY: {
            // allow arming only after a short settle time
            bool ready_time_ok = (now_ms - s->ready_since_ms) >= cfg->post_cal_ready_ms;

            if (s->arm_requested && ready_time_ok) {
                if ((now_ms - s->arm_request_start_ms) >= cfg->arm_hold_ms) {
                    safety_set_mode(s, SAFETY_ARMED, now_ms);
                }
            } else {
                // if not requested, reset request window
                if (!s->arm_requested) s->arm_request_start_ms = now_ms;
            }
        } break;

        case SAFETY_ARMED:
            // stays armed unless failsafe/disarm command triggers
            break;

        case SAFETY_FAILSAFE:
            // stays here until disarmed
            break;
    }

    return (s->mode == SAFETY_ARMED);
}

float safety_throttle_scale(SafetyState *s, const SafetyConfig *cfg, float dt) {
    if (s->mode != SAFETY_ARMED) return 0.0f;

    s->armed_throttle_scale += cfg->throttle_ramp_per_s * dt;
    s->armed_throttle_scale = clampf(s->armed_throttle_scale, 0.0f, 1.0f);
    return s->armed_throttle_scale;
}
