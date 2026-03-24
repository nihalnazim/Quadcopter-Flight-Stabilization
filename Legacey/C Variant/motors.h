#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

// Assign your motor pins here (must be PWM-capable)
#define MOTOR1_PIN 21
#define MOTOR2_PIN 19
#define MOTOR3_PIN 17
#define MOTOR4_PIN 16

void motors_init(void);

// Set motor outputs in range 0.0–1.0 (mapped to 1–2ms pulse at ~50 Hz)
void motors_set(float m1, float m2, float m3, float m4);

#endif // MOTORS_H
