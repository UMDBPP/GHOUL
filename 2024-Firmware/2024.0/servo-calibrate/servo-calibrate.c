#include <stdio.h>
#include <stdlib.h>

#include "../vent-servo.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#define PWM_PIN 1

int main() {
    stdio_init_all();

    set_sys_clock_48mhz();

    setup_vent_servo(PWM_PIN);

    uint pwm_level = 2100;
    char rx = 0;
    float num_usec = 700;

    while (true) {
        rx = getchar_timeout_us(0);

        if (rx == '.') {
            num_usec = num_usec + 0.33;
            vent_servo_set_pulse_width(pwm_level++);
        } else if (rx == ',') {
            num_usec = num_usec - 0.33;
            vent_servo_set_pulse_width(pwm_level--);
        }

        // printf("\033[H");
        printf("\n\n\n\n\n\n\ncurrent pulse width usec: %f", num_usec);
        rx = 0;
    }
}