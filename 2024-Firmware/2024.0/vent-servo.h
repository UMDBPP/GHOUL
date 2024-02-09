#ifndef VENT_SERVO_H
#define VENT_SERVO_H

#include "pico/stdlib.h"

#define WRAP_VALUE (uint16_t)19999
#define PWM_DIV 24

void setup_vent_servo(uint pwm_pin);
void vent_servo_demo();
void vent_servo_open();
void vent_servo_close();
short vent_servo_set_pulse_width(uint num_usec);

#endif