#include <stdio.h>
#include <stdlib.h>

#include "../GHOUL_2024_0.h"
#include "../vent-servo.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

int main() {
    stdio_init_all();

    set_sys_clock_48mhz();

    setup_vent_servo(SERVO_PWM);

    uint pwm_level = 2100;
    char rx = 0;
    float num_usec = 700;

    while (true) {
        rx = getchar_timeout_us(0);

        if (rx == '.') {
            num_usec = num_usec + 0.33;
            vent_servo_set_pulse_width(pwm_level++, false);
        } else if (rx == ',') {
            num_usec = num_usec - 0.33;
            vent_servo_set_pulse_width(pwm_level--, false);
        }

        // printf("\033[H");
        printf("\n\n\n\n\n\n\ncurrent pulse width usec: %f", num_usec);
        rx = 0;
    }
}
