cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(test_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(flight 
    flight.cpp
    ../vent-servo.c
    ../rp2040-drf1262-lib/SX1262.cpp
    ../rp2040-config/MB85RS1MT.cpp
    ../rp2040-config/config.cpp
)

pico_enable_stdio_usb(flight 1)

pico_add_extra_outputs(flight)

target_link_libraries(flight pico_stdlib hardware_sync hardware_pwm hardware_spi hardware_i2c hardware_gpio hardware_clocks pico_unique_id pico_rand hardware_watchdog pico_time)