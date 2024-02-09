#include <stdio.h>
#include <stdlib.h>

#include "../vent-servo.h"
#include "pico/stdlib.h"

#define PWM_PIN 1

int main() {
    stdio_init_all();

    set_sys_clock_48mhz();

    setup_vent_servo(PWM_PIN);

    uint num_usec = 700;

    while (true) {
        if (getchar_timeout_us(0) == '.')
            vent_servo_set_pulse_width(num_usec++);
        else if (getchar_timeout_us(0) == ',')
            vent_servo_set_pulse_width(num_usec--);

        printf("\033[H");
        printf("current pulse width usec: %d\n", num_usec);
    }
}
