#ifndef _BITSv5_H
#define _BITSv5_H

#define SCK_PIN 26
#define MOSI_PIN 27
#define MISO_PIN 24

#define RADIO_CS 25

#define TXEN_PIN 8
#define DIO1_PIN 10
#define BUSY_PIN 11
#define SW_PIN 9
#define RADIO_RST 13

#define SDA_PIN 20
#define SCL_PIN 21

#define GPS_ADDR 0x42
#define EXTINT_PIN 17
#define TIMEPULSE_PIN 18

#define FRAM_CS 29

#define LED1_PIN 0
#define LED2_PIN 1
#define LED3_PIN 7
#define LED4_PIN 12
#define LED5_PIN 4

#define GPIO2_PIN 2
#define GPIO3_PIN 3
#define GPIO16_PIN 16
#define GPIO19_PIN 19

#define SERVO_PWM 5

#define TEMP_PIN 6

#define HEATER1_PIN 14
#define HEATER2_PIN 14

#define PGOOD_CUT_PIN 22
#define DATA_CUT 23
#define CURRENT_CUT 28

#define LOG_INIT_ADDR 8192
#define LOG_MAX_ADDR 131072

#define INCLUDE_DEBUG \
    1  // controls if debug conditionals are included at compile time

extern short debug_msgs;  // controls if debug messages are printed

#endif