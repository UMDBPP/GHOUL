#include "vent-servo.h"

#include <stdio.h>

#include "hardware/pwm.h"
#include "pico/stdlib.h"

uint max_level = 1800 * 3;
uint min_level = 600 * 3;
uint slice_num = 0;
uint pwm_pin = 0;

void setup_vent_servo(uint pin) {
    pwm_pin = pin;
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);

    slice_num = pwm_gpio_to_slice_num(pwm_pin);

    pwm_set_phase_correct(slice_num, true);

    pwm_set_clkdiv(slice_num, PWM_DIV);

    pwm_set_wrap(slice_num, WRAP_VALUE);

    pwm_set_chan_level(slice_num, PWM_CHAN_B, min_level);

    pwm_set_enabled(slice_num, true);
}

void vent_servo_demo() {
    uint level = 2100;  // def don't go past 2000, 600-1300 for normal
                        // range of motion on UMD Vent

    bool forward = false;

    float num_usec = 700;

    while (true) {
        if (level > max_level || level < min_level) {
            printf("PWM duty cycle out of bounds\n");
            level = 2100;
        }

        pwm_set_gpio_level(pwm_pin, level);

        // pwm_set_chan_level(slice_num, PWM_CHAN_B, num_usec);

        printf("microseconds: %f\n", num_usec);

        if (level >= (1300 * 3)) forward = false;

        if (level <= (600 * 3)) forward = true;

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

// level is actually in 1/3 microseconds
short vent_servo_set_pulse_width(uint level) {
    if (level > max_level || level < min_level) {
        printf("PWM duty cycle out of bounds\n");
        return -1;
    }

    pwm_set_gpio_level(pwm_pin, level);
    return 0;
}