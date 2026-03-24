cmake_minimum_required(VERSION 3.13)

# This expects pico_sdk_import.cmake one level up from this folder
include(pico_sdk_import.cmake)

project(drone_firmware C CXX)

pico_sdk_init()

add_executable(drone
    main.c
    imu_i2c.c
    mpu6050.c
    attitude.c
    calibration.c
    control.c
    motors.c
    safety.c
    battery.c
)

target_link_libraries(drone
    pico_stdlib
    hardware_i2c
    hardware_pwm
    hardware_adc
)


pico_enable_stdio_usb(drone 1)
pico_enable_stdio_uart(drone 0)

pico_add_extra_outputs(drone)
