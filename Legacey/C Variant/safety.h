#ifndef SAFETY_H
#define SAFETY_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SAFETY_DISARMED = 0,
    SAFETY_CALIBRATING,
    SAFETY_READY,
    SAFETY_ARMED,
    SAFETY_FAILSAFE
} SafetyMode;

typedef struct {
    // ---- Arming ----
    uint32_t arm_hold_ms;          // time the user must hold ARM command before arming
    uint32_t post_cal_ready_ms;    // wait time after calibration before allowing arm

    // ---- IMU failsafe ----
    uint32_t imu_freeze_ms;        // if no valid IMU read for this long => failsafe

    // ---- Angle cutoff ----
    float max_angle_deg;           // if |roll| or |pitch| exceeds this => disarm/failsafe

    // ---- Throttle ramp ----
    float throttle_ramp_per_s;     // how fast throttle ramps up (0..1 per second)

    // ---- Low battery (ADC) ----
    bool battery_enabled;
    float vbat_warn;              // warning threshold (Volts)
    float vbat_crit;              // critical threshold (Volts)
    uint32_t vbat_check_ms;       // how often to check

    // ADC conversion constants (assumes a voltage divider into ADC)
    float adc_vref;               // usually 3.3V
    float divider_ratio;          // Vbat = Vadc * divider_ratio  (e.g. 11.0 for 10k/1k)
} SafetyConfig;

typedef struct {
    SafetyMode mode;

    // command + timers
    bool arm_requested;
    uint32_t arm_request_start_ms;
    uint32_t ready_since_ms;

    // IMU tracking
    uint32_t last_imu_ok_ms;

    // battery
    float vbat;
    bool vbat_warned;
    bool vbat_critical;
    uint32_t last_vbat_check_ms;

    // throttle ramp
    float armed_throttle_scale;   // ramps 0->1 when armed
} SafetyState;

void safety_init(SafetyState *s, const SafetyConfig *cfg, uint32_t now_ms);
void safety_set_mode(SafetyState *s, SafetyMode m, uint32_t now_ms);

// Call this whenever an IMU read succeeds
void safety_notify_imu_ok(SafetyState *s, uint32_t now_ms);

// Serial command handlers
void safety_cmd_arm(SafetyState *s, uint32_t now_ms);
void safety_cmd_disarm(SafetyState *s, uint32_t now_ms);

// Update the safety state machine.
// Returns true if motors should be allowed to run (ARMED).
bool safety_update(SafetyState *s,
                   const SafetyConfig *cfg,
                   uint32_t now_ms,
                   float roll_deg,
                   float pitch_deg,
                   bool imu_ok,
                   bool battery_critical_now);

// Returns a multiplier in [0..1] for throttle ramping after arming
float safety_throttle_scale(SafetyState *s, const SafetyConfig *cfg, float dt);

// Battery helpers (optional)
void safety_battery_update(SafetyState *s, const SafetyConfig *cfg, uint32_t now_ms, float vbat);

#endif
