
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include <stdint.h>
#include "hardware/i2c.h"
#include "imu_i2c.h"
#include "mpu6050.h"
#include "attitude.h"
#include "calibration.h"
#include "control.h"
#include "motors.h"
#include "safety.h"
#include "battery.h"
#include "pico/stdio_usb.h"


// ---------------- Control Loop Timing ----------------
#define LOOP_HZ        250.0f
#define LOOP_PERIOD_US (1000000.0f / LOOP_HZ)


// ---------------- Battery Sense ----------------
// ADC0 on Pico is GPIO26 (must be fed through a voltage divider!)
#define VBAT_ADC_GPIO  26


static inline float clampf(float x, float lo, float hi) {
   if (x < lo) return lo;
   if (x > hi) return hi;
   return x;
}


int main(void) {
   // ---- Basic IO init ----
   // Wait for USB serial to be connected (max ~5s), so prints don't get dropped
absolute_time_t t0 = get_absolute_time();
while (!stdio_usb_connected() && absolute_time_diff_us(t0, get_absolute_time()) < 5 * 1000000) {
    sleep_ms(10);
}

printf("DRONE DEBUG BUILD V2\n");



   const uint LED_PIN = PICO_DEFAULT_LED_PIN;
   gpio_init(LED_PIN);
   gpio_set_dir(LED_PIN, GPIO_OUT);
   gpio_put(LED_PIN, 0);


   printf("DRONE DEBUG BUILD V2\n");

   printf("Commands: 'A' to ARM (hold), 'D' to DISARM\n");


   // ---- I2C + IMU init ----
   imu_i2c_init();
   printf("I2C initialized (SDA=%d, SCL=%d, %d Hz)\n",
      IMU_I2C_SDA_PIN, IMU_I2C_SCL_PIN, IMU_I2C_BAUDRATE);



   if (!imu_scan_for_mpu()) {
       printf("WARNING: MPU-6050 not detected at 0x%02X (wiring/power?)\n", MPU6050_ADDR);
   } else {
       printf("MPU-6050 detected at 0x%02X\n", MPU6050_ADDR);
   }


   if (!mpu6050_init()) {
       printf("ERROR: MPU-6050 init failed.\n");
   } else {
       printf("MPU-6050 init OK.\n");
   }


   // ---- Attitude init ----
   AttitudeState att;
   attitude_init(&att);


   // ---- Motor init (but we still keep motors OFF unless armed) ----
   motors_init();
   motors_set(0, 0, 0, 0);
   printf("Motors PWM initialized (forced OFF until armed).\n");


   // ---- Battery ADC init (safe even before LiPo arrives, but ADC pin must be valid) ----
   battery_init(VBAT_ADC_GPIO);
   printf("Battery ADC initialized on GPIO%d (ADC%d)\n", VBAT_ADC_GPIO, (int)(VBAT_ADC_GPIO - 26));


   // ---- Safety config ----
   SafetyConfig scfg = {
       // Arming behavior
       .arm_hold_ms = 800,            // must hold ARM request for 0.8s
       .post_cal_ready_ms = 500,      // wait after calibration before arming allowed


       // Failsafes
       .imu_freeze_ms = 100,          // if no good IMU read for 100ms -> FAILSAFE
       .max_angle_deg = 45.0f,        // tip beyond this -> FAILSAFE


       // Throttle ramp
       .throttle_ramp_per_s = 0.8f,   // ramps 0->1 in ~1.25s


       // Battery
       .battery_enabled = false,
       // Example thresholds for a 3S LiPo (tune later)
       .vbat_warn = 10.8f,
       .vbat_crit = 10.2f,
       .vbat_check_ms = 250,


       // ADC conversion constants
       .adc_vref = 3.3f,
       // IMPORTANT: divider_ratio depends on your resistor divider.
       // Example: 10k (top) and 1k (bottom) => ratio = (10k+1k)/1k = 11.0
       .divider_ratio = 11.0f
   };


   SafetyState safety;
   uint32_t now_ms = to_ms_since_boot(get_absolute_time());
   safety_init(&safety, &scfg, now_ms);
   safety_set_mode(&safety, SAFETY_CALIBRATING, now_ms);


   // ---- Calibration (motors OFF) ----
   motors_set(0, 0, 0, 0);

  printf("SKIPPING gyro calibration (debug mode)\n");
  safety_set_mode(&safety, SAFETY_READY, now_ms);




   // ---- Control init ----
   ControlState ctrl;
   control_init(&ctrl);


   // For hover-only demo: keep these at level hover (0 degrees)
   ctrl.setpoint_roll  = 0.0f;
   ctrl.setpoint_pitch = 0.0f;


   // NOTE: base_throttle is your “hover guess”.
   // You MUST tune this for your build. Start low.
   // ctrl.base_throttle = 0.35f;  // example: safer starting guess than 0.5
   // If your control_init already sets it, you can override here.


   printf("Safety state: READY. Hold 'A' to arm.\n");


   // ---- Loop timing ----
   absolute_time_t last_time = get_absolute_time();
   uint32_t loop_counter = 0;


   // For battery prints rate limiting
   uint32_t last_status_print_ms = now_ms;


   while (true) {
       absolute_time_t t_now = get_absolute_time();
       now_ms = to_ms_since_boot(t_now);


       int64_t dt_us = absolute_time_diff_us(last_time, t_now);
       last_time = t_now;


       float dt = dt_us / 1000000.0f;
       // dt sanity clamp (prevents one bad frame from exploding math)
       dt = clampf(dt, 0.0005f, 0.02f);


       // ---- Non-blocking serial commands ----
       int ch = getchar_timeout_us(0);
       if (ch != PICO_ERROR_TIMEOUT) {
           if (ch == 'a' || ch == 'A') {
               safety_cmd_arm(&safety, now_ms);
           } else if (ch == 'd' || ch == 'D') {
               safety_cmd_disarm(&safety, now_ms);
               motors_set(0, 0, 0, 0);
               printf("DISARMED.\n");
           }
       }


       // ---- Battery check ----
       bool battery_critical_now = false;
       if (scfg.battery_enabled && (now_ms - safety.last_vbat_check_ms) >= scfg.vbat_check_ms) {
           safety.last_vbat_check_ms = now_ms;


           float vbat = battery_read_vbat(scfg.adc_vref, scfg.divider_ratio);
           safety_battery_update(&safety, &scfg, now_ms, vbat);


           battery_critical_now = safety.vbat_critical;
       }


       // ---- IMU read + attitude update ----
       int16_t ax, ay, az, gx, gy, gz;
       bool imu_ok = mpu6050_read_raw(&ax, &ay, &az, &gx, &gy, &gz);


       if (imu_ok) {
           attitude_update(&att, ax, ay, az, gx, gy, gz, dt);
           safety_notify_imu_ok(&safety, now_ms);
       }


       // ---- Safety update (may force FAILSAFE) ----
       bool allow_motors = safety_update(&safety, &scfg, now_ms,
                                        att.roll, att.pitch,
                                        imu_ok,
                                        battery_critical_now);


       // ---- Compute control + motor commands ----
       float m1 = 0, m2 = 0, m3 = 0, m4 = 0;


       if (allow_motors) {
           control_compute(&ctrl, &att, dt, &m1, &m2, &m3, &m4);


           // Apply throttle ramp after arming
           float scale = safety_throttle_scale(&safety, &scfg, dt);
           motors_set(m1 * scale, m2 * scale, m3 * scale, m4 * scale);
       } else {
           motors_set(0, 0, 0, 0);
       }


       // ---- LED indicator ----
       // Blink patterns:
       // READY: slow blink
       // ARMED: faster blink
       // FAILSAFE: rapid blink
       static uint32_t last_led_toggle_ms = 0;
       uint32_t blink_ms = 0;


       switch (safety.mode) {
           case SAFETY_READY:     blink_ms = 400; break;
           case SAFETY_ARMED:     blink_ms = 120; break;
           case SAFETY_FAILSAFE:  blink_ms = 60;  break;
           default:               blink_ms = 800; break;
       }


       if ((now_ms - last_led_toggle_ms) >= blink_ms) {
           last_led_toggle_ms = now_ms;
           gpio_xor_mask(1u << LED_PIN);
       }


       // ---- Status print (rate-limited) ----
       if ((now_ms - last_status_print_ms) >= 20) {
           last_status_print_ms = now_ms;


           const char *mode_str =
               (safety.mode == SAFETY_DISARMED)   ? "DISARMED" :
               (safety.mode == SAFETY_CALIBRATING)? "CALIBRATING" :
               (safety.mode == SAFETY_READY)      ? "READY" :
               (safety.mode == SAFETY_ARMED)      ? "ARMED" :
                                                    "FAILSAFE";


           printf("[%s] Roll=%.2f Pitch=%.2f  VBAT=%.2fV%s%s\n",
                  mode_str,
                  att.roll, att.pitch,
                  safety.vbat,
                  (safety.vbat_warned ? " WARN" : ""),
                  (safety.vbat_critical ? " CRIT" : ""));


           if (!imu_ok) {
               printf("IMU read not OK (watch wiring/power). Motors forced OFF unless ARMED+healthy.\n");
           }
       }


       // ---- Maintain fixed loop rate ----
       absolute_time_t next_time = delayed_by_us(t_now, (uint32_t)LOOP_PERIOD_US);
       sleep_until(next_time);


       loop_counter++;
   }


   return 0;
}
