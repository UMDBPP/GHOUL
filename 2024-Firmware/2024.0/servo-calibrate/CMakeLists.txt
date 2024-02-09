cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(test_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(servo-calibrate
    servo-calibrate.c
    ../vent-servo.c
)

pico_enable_stdio_usb(servo-calibrate 1)

pico_add_extra_outputs(servo-calibrate)

target_link_libraries(servo-calibrate pico_stdlib hardware_pwm hardware_clocks)