#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "../vent-servo.h"

#define PWM_PIN 1

int main() {
    stdio_init_all();

    set_sys_clock_48mhz();

    setup_vent_servo(PWM_PIN);

    vent_servo_demo();
}
