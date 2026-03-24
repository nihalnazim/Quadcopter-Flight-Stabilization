#include "battery.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"

static uint g_adc_input = 0;

void battery_init(uint adc_gpio) {
    adc_init();
    adc_gpio_init(adc_gpio);

    // GPIO26->ADC0, GPIO27->ADC1, GPIO28->ADC2
    g_adc_input = (adc_gpio - 26);
    adc_select_input(g_adc_input);
}

float battery_read_vbat(float adc_vref, float divider_ratio) {
    // 12-bit ADC: 0..4095
    uint16_t raw = adc_read();
    float v_adc = ((float)raw / 4095.0f) * adc_vref;
    return v_adc * divider_ratio;
}
