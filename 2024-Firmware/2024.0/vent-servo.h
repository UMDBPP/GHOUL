#ifndef VENT_SERVO_H
#define VENT_SERVO_H

#include "pico/stdlib.h"

// Below values assume a 48 MHz system clock, otherwise PWM frequency and pulse
// length will be incorrect
#define WRAP_VALUE (uint16_t)59999  // wrap value of PWM counter
#define PWM_DIV 8                   // PWM clock divider, divides system clock

/**
 * @brief Configures PWM pin and peripheral, system clock must be set to 48 MHz
 * before this function is called.
 *
 * @param pwm_pin
 */
void setup_vent_servo(uint pwm_pin);

/**
 * @brief Repeatedly opens and closes vent servo from the min angle to the max
 * angle.
 */
void vent_servo_demo();

/**
 * Convenience function to set servo to maximum angle (vent open).
 */
void vent_servo_open();

/**
 * Convenience function to set servo to minimum angle (vent closed).
 */
void vent_servo_close();

/**
 * Convenience function to set servo level. Checks level against maximum and
 * minimum allowable values to prevent collision.
 *
 * @param level length of servo PWM pulse length in 1/3 microseconds.
 *
 * @returns -1 if level is out of bounds, 0 on success
 */
short vent_servo_set_pulse_width(uint level);

#endif