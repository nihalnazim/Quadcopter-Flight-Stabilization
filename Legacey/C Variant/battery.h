#ifndef BATTERY_H
#define BATTERY_H

#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"   // <-- add this

void battery_init(uint adc_gpio);
float battery_read_vbat(float adc_vref, float divider_ratio);

#endif
