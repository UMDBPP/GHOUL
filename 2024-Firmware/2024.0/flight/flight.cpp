#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../GHOUL_2024_0.h"
#include "../rp2040-config/MB85RS1MT.h"
#include "../rp2040-config/config.h"
#include "../rp2040-drf1262-lib/SX1262.h"
#include "../vent-servo.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

volatile bool vent_open = false;
volatile bool cutdown = false;
volatile bool tx_done = false;
volatile bool rx_done = false;
volatile bool send_ack = false;
volatile bool do_tx = false;

// timers
repeating_timer_t tx_timer;
repeating_timer_t log_timer;
alarm_pool_t *ack_alarm_pool;
alarm_id_t ack_alarm_id;

// radio
DRF1262 radio(spi1, RADIO_CS, SCK_PIN, MOSI_PIN, MISO_PIN, TXEN_PIN, DIO1_PIN,
              BUSY_PIN, SW_PIN, RADIO_RST);
uint8_t radio_tx_buf[100] = "hello!";
char radio_rx_buf[100] = {0};
char ack[] = "ack - GHOUL alive";

// logging
MB85RS1MT mem(spi1, FRAM_CS, SCK_PIN, MOSI_PIN, MISO_PIN);
Config test_config;
char log_str[] = "GHOUL,PRESS,TEMP,LAT,LONG,TIME\n";  // "a b c d ef"
char log_str1[] = "0,0,0,99.9999,99.9999,0000.00\n";  //"h i j k lm"
uint8_t log_buf = 0;
static uint32_t log_addr = LOG_INIT_ADDR;

// Ports
i2c_inst_t *i2c = i2c0;

void setup_led();
void led_on();
void led_off();
bool tx_timer_callback(repeating_timer_t *rt);
bool log_timer_callback(repeating_timer_t *rt);
void gpio_callback(uint gpio, uint32_t events);
static int64_t ack_timer_callback(alarm_id_t id, void *user_data);
void setup_spi();
void write_name_config();
void transmit(uint8_t *buf, size_t len);

int main() {
    stdio_init_all();

    // set_sys_clock_48mhz(); // could reduce the system clock speed if needed
    // for power reasons, would rather have the performance

    // when DIO1 goes high on the radio interrupt with gpio_callback
    gpio_set_irq_enabled_with_callback(DIO1_PIN, GPIO_IRQ_EDGE_RISE, true,
                                       &gpio_callback);

    setup_spi();  // init SPI peripheral for Radio and FRAM

    ack_alarm_pool = alarm_pool_create_with_unused_hardware_alarm(4);

    sleep_ms(5000);

    // setup devices
    radio.radio_init();
    mem.mem_init();

    setup_vent_servo(SERVO_PWM);

    // Timers: the next two blocks create timers that trigger every 60 seconds,
    // they execute the timer_callback functions specified

    // negative timeout means exact delay (rather than delay between
    // callbacks)
    if (!add_repeating_timer_us(-60000, tx_timer_callback, NULL, &tx_timer)) {
        printf("Failed to add timer\n");
        return -1;
    }

    if (!add_repeating_timer_us(-60000, log_timer_callback, NULL, &log_timer)) {
        printf("Failed to add timer\n");
        return -1;
    }

    // Enable the watchdog, requiring the watchdog to be updated every 1 minute
    // or the chip will reboot
    watchdog_enable(60000, 1);

    if (watchdog_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
    } else {
        printf("Clean boot\n");
    }

    while (true) {
        watchdog_update();

        // timed events
        //  - transmit location and some status maybe
        //  - log LAT,LONG,Time,Fix

        // get and parse any new GPS data

        // if new data in radio buffer, parse and execute

        if (rx_done) {
            // reads received packet data from the radio and places it in
            // radio_rx_buf
            radio.read_radio_buffer((uint8_t *)radio_rx_buf,
                                    sizeof(radio_rx_buf));
            printf("%s\n", radio_rx_buf);

            // This is only important for testing
            radio.get_packet_status();
            printf("RSSI: %d dBm Signal RSSI: %d SNR: %d dB\n",
                   radio.pkt_stat.rssi_pkt, radio.pkt_stat.signal_rssi_pkt,
                   radio.pkt_stat.snr_pkt);

            if (strncmp("ack", radio_rx_buf, 3) != 0) {
                ack_alarm_id = alarm_pool_add_alarm_in_ms(
                    ack_alarm_pool, 1000, ack_timer_callback, NULL, true);
            }

            rx_done = false;
        }

        if (send_ack && !tx_done) {
            printf("ACK\n");
            transmit((uint8_t *)ack, sizeof(ack));
            send_ack = false;
        }

        if (do_tx && !tx_done) {
            transmit((uint8_t *)radio_tx_buf, sizeof(radio_tx_buf));
            do_tx = false;
        }

        if (cutdown) {
            // assert some pin to initiate cut
        }

        vent_servo_open(vent_open);
    }
}

// writes the string to FRAM using the config settings in the test_config struct
void write_name_config() {
    strcpy(test_config.name, "BITSv5.2-0");
    write_config(NAME, test_config, (uint8_t *)test_config.name,
                 sizeof(test_config.name), &mem);
}

void setup_spi() {
    spi_init(spi1, 500000);
    spi_set_format(spi1,           // SPI instance
                   8,              // Number of bits per transfer
                   (spi_cpol_t)0,  // Polarity (CPOL)
                   (spi_cpha_t)0,  // Phase (CPHA)
                   SPI_MSB_FIRST);
}

// Note that the callback functions interrupt the normal program routine (i.e.
// the infinite while loop in main()) so it's important that they not take very
// long. You'll notice that for the most part they consist of modifying a bool
// flag that control a conditional in the main loop.

// when the radio finishes sending or receiving a packet it asserts DIO1 high
// and triggers this interrupt
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == DIO1_PIN) {
        radio.get_irq_status();

        if (radio.irqs.TX_DONE) {
            radio.disable_tx();
            radio.radio_receive_cont();
            led_off();
            tx_done = true;
            radio.irqs.TX_DONE = false;
        }

        if (radio.irqs.RX_DONE) {
            radio.irqs.RX_DONE = false;
            send_ack = true;
            rx_done = true;
        }

        radio.clear_irq_status();
    }
}

bool tx_timer_callback(repeating_timer_t *rt) {
    do_tx = true;
    return true;  // keep repeating
}

bool log_timer_callback(repeating_timer_t *rt) {
    // log some status string
}

static int64_t ack_timer_callback(alarm_id_t id, void *user_data) {
    send_ack = true;
    return 0;  // don't repeat
}

void setup_led() {
    gpio_init(LED1_PIN);
    gpio_set_dir(LED1_PIN, GPIO_OUT);
    gpio_put(LED1_PIN, 0);
}

void led_on() { gpio_put(LED1_PIN, true); }

void led_off() { gpio_put(LED1_PIN, false); }

void transmit(uint8_t *buf, size_t len) {
    printf("Transmit Test\n");
    led_on();
    tx_done = false;

    radio.radio_send(buf, len);

    // while (!tx_done) busy_wait_us_32(1);

    printf("%s\n", (char *)buf);
}
