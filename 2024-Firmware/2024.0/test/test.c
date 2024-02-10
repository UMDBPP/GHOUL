#include <stdio.h>
#include <stdlib.h>

#include "../vent-servo.h"
#include "pico/stdlib.h"

#define PWM_PIN 1

int main() {
    // uint max_level = 1300 * 3;
    // uint min_level = 600 * 3;

    stdio_init_all();

    set_sys_clock_48mhz();

    sleep_ms(5000);

    setup_vent_servo(PWM_PIN);

    while (true) {
        printf("open\n");
        vent_servo_open(true);

        sleep_ms(1000);

        printf("close\n");
        vent_servo_close(true);

        sleep_ms(1000);
    }
}
