#include "vent-servo.h"

#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

uint max_level = 1300 * 3;
uint min_level = 600 * 3;
uint slice_num = 0;
uint pwm_pin = 0;
uint current_level;

void setup_vent_servo(uint pin) {
    pwm_pin = pin;
    slice_num = pwm_gpio_to_slice_num(pwm_pin);

    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
    pwm_set_phase_correct(slice_num, true);
    pwm_set_clkdiv(slice_num, PWM_DIV);
    pwm_set_wrap(slice_num, WRAP_VALUE);
    pwm_set_gpio_level(pwm_pin, 0);
    pwm_set_enabled(slice_num, true);
}

void vent_servo_demo() {
    uint level = 2100;
    bool forward = false;
    float num_usec = 700;

    while (true) {
        if (level > max_level || level < min_level) {
            printf("PWM duty cycle out of bounds\n");
            level = 2100;
        }

        pwm_set_gpio_level(pwm_pin, level);
        current_level = level;

        printf("microseconds: %f\n", num_usec);

        if (level >= (max_level)) forward = false;

        if (level <= (min_level)) forward = true;

        if (forward) {
            num_usec = num_usec + 0.33;
            level = level + 5;
        } else {
            num_usec = num_usec - 0.33;
            level = level - 5;
        }

        tight_loop_contents();
    }
}

short vent_servo_set_pulse_width(uint level, bool low_power) {
    enable_servo();

    if (level > max_level || level < min_level) {
        printf("PWM duty cycle out of bounds\n");
        return -1;
    }

    pwm_set_gpio_level(pwm_pin, level);

    if (low_power) {
        disable_servo();
    }

    current_level = level;
    return 0;
}

void vent_servo_open(bool low_power) {
    if (current_level < min_level) {
        current_level = min_level;
        sleep_ms(1000);
    }

    while (current_level < max_level) {
        vent_servo_set_pulse_width(current_level, false);
        current_level = current_level + 1;
        if (current_level >= max_level) current_level = max_level;
        sleep_ms(2);
    }
    vent_servo_set_pulse_width(max_level, false);

    if (low_power) disable_servo();

    sleep_ms(1000);
}
void vent_servo_close(bool low_power) {
    if (current_level > max_level) {
        current_level = max_level;
        sleep_ms(1000);
    }

    while (current_level > min_level) {
        vent_servo_set_pulse_width(current_level, false);
        current_level = current_level - 1;
        if (current_level <= min_level) current_level = min_level;
        sleep_ms(2);
    }
    vent_servo_set_pulse_width(min_level, false);

    if (low_power) disable_servo();

    sleep_ms(1000);
}

void disable_servo() {
    // sleep_ms(500);  // stupid delay for now until I can figure out how to
    // ensure that this doesn't cause the servo to stop moving to it's intended
    // position GPIO_OVERRIDE_LOW GPIO_OVERRIDE_NORMAL
    gpio_set_outover(pwm_pin, GPIO_OVERRIDE_LOW);
    // pwm_set_gpio_level(pwm_pin, 0);

    pwm_set_enabled(slice_num, false);
}
void enable_servo() {
    gpio_set_outover(pwm_pin, GPIO_OVERRIDE_NORMAL);
    // pwm_set_gpio_level(pwm_pin, current_level);
    pwm_set_enabled(slice_num, true);
}