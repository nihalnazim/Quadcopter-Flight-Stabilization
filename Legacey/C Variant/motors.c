#include "motors.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

static uint slice_m1, chan_m1;
static uint slice_m2, chan_m2;
static uint slice_m3, chan_m3;
static uint slice_m4, chan_m4;

static inline uint16_t duty_from_throttle(float throttle) {
    if (throttle < 0.0f) throttle = 0.0f;
    if (throttle > 1.0f) throttle = 1.0f;

    // Assuming 50 Hz: period ~20ms, we want 1ms–2ms (5–10% duty).
    // We'll compute based on wrap value configured in motors_init.
    // For wrap=39062: 5%≈1953, 10%≈3906.
    extern uint16_t g_pwm_wrap;   // defined below
    float min_duty = 0.05f * (float)g_pwm_wrap;
    float max_duty = 0.10f * (float)g_pwm_wrap;

    float duty = min_duty + throttle * (max_duty - min_duty);
    if (duty < 0.0f) duty = 0.0f;
    if (duty > g_pwm_wrap) duty = (float)g_pwm_wrap;
    return (uint16_t)duty;
}

uint16_t g_pwm_wrap = 39062;  // default, updated in init

void motors_init(void) {
    gpio_set_function(MOTOR1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR2_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR3_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR4_PIN, GPIO_FUNC_PWM);

    slice_m1 = pwm_gpio_to_slice_num(MOTOR1_PIN);
    slice_m2 = pwm_gpio_to_slice_num(MOTOR2_PIN);
    slice_m3 = pwm_gpio_to_slice_num(MOTOR3_PIN);
    slice_m4 = pwm_gpio_to_slice_num(MOTOR4_PIN);

    chan_m1 = pwm_gpio_to_channel(MOTOR1_PIN);
    chan_m2 = pwm_gpio_to_channel(MOTOR2_PIN);
    chan_m3 = pwm_gpio_to_channel(MOTOR3_PIN);
    chan_m4 = pwm_gpio_to_channel(MOTOR4_PIN);

    float clkdiv = 64.0f;
    g_pwm_wrap = 39062; // ~50 Hz for 125MHz / 64

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, clkdiv);
    pwm_config_set_wrap(&cfg, g_pwm_wrap);

    pwm_init(slice_m1, &cfg, true);
    pwm_init(slice_m2, &cfg, true);
    pwm_init(slice_m3, &cfg, true);
    pwm_init(slice_m4, &cfg, true);

    motors_set(0.0f, 0.0f, 0.0f, 0.0f);
}

void motors_set(float m1, float m2, float m3, float m4) {
    uint16_t d1 = duty_from_throttle(m1);
    uint16_t d2 = duty_from_throttle(m2);
    uint16_t d3 = duty_from_throttle(m3);
    uint16_t d4 = duty_from_throttle(m4);

    pwm_set_chan_level(slice_m1, chan_m1, d1);
    pwm_set_chan_level(slice_m2, chan_m2, d2);
    pwm_set_chan_level(slice_m3, chan_m3, d3);
    pwm_set_chan_level(slice_m4, chan_m4, d4);
}
