#include "vent-servo.h"

#include <stdio.h>

#include "hardware/pwm.h"
#include "pico/stdlib.h"

uint max_usec = 1800;
uint min_usec = 600;
uint slice_num = 0;
uint pwm_pin = 0;

void setup_vent_servo(uint pin) {
    pwm_pin = pin;
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);

    slice_num = pwm_gpio_to_slice_num(pwm_pin);

    pwm_set_phase_correct(slice_num, true);

    pwm_set_clkdiv(slice_num, PWM_DIV);

    pwm_set_wrap(slice_num, WRAP_VALUE);

    pwm_set_chan_level(slice_num, PWM_CHAN_B, min_usec);

    pwm_set_enabled(slice_num, true);
}

void vent_servo_demo() {
    uint num_usec = 700;  // def don't go past 2000, 600-1300 for normal range
                          // of motion on UMD Vent

    bool forward = false;

    while (true) {
        if (num_usec > max_usec || num_usec < min_usec) {
            printf("PWM duty cycle out of bounds\n");
            num_usec = 700;
        }

        pwm_set_gpio_level(pwm_pin, num_usec);

        // pwm_set_chan_level(slice_num, PWM_CHAN_B, num_usec);

        printf("microseconds: %d\n", num_usec);

        if (num_usec >= 1300) forward = false;

        if (num_usec <= 600) forward = true;

        if (forward)
            num_usec = num_usec + 0.1;
        else
            num_usec = num_usec - 0.1;

        tight_loop_contents();
    }
}

short vent_servo_set_pulse_width(uint num_usec) {
    if (num_usec > max_usec || num_usec < min_usec) {
        printf("PWM duty cycle out of bounds\n");
        return -1;
    }

    pwm_set_gpio_level(pwm_pin, num_usec);
    return 0;
}