#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern "C" {
#include "../vent-servo.h"
}

#include "../GHOUL_2024_0.h"
#include "../rp2040-config/MB85RS1MT.h"
#include "../rp2040-config/config.h"
#include "../rp2040-drf1262-lib/SX1262.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

volatile bool vent_open = false;
volatile bool cutdown = false;
volatile bool tx_done = true;
volatile bool rx_done = false;
volatile bool send_ack = false;
volatile bool do_tx = false;
volatile bool do_close_vent = false;

// timers
repeating_timer_t tx_timer;
repeating_timer_t log_timer;
alarm_pool_t *ack_alarm_pool;
alarm_id_t ack_alarm_id;
alarm_pool_t *vent_alarm_pool;
alarm_id_t vent_alarm_id;

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

// Misc
char c = 0;

void setup_led();
void led_on();
void led_off();
bool tx_timer_callback(repeating_timer_t *rt);
bool log_timer_callback(repeating_timer_t *rt);
void gpio_callback(uint gpio, uint32_t events);
static int64_t ack_timer_callback(alarm_id_t id, void *user_data);
static int64_t vent_timer_callback(alarm_id_t id, void *user_data);
void setup_spi();
void write_name_config();
void transmit(uint8_t *buf, size_t len);
void parse_text_radio_cmd(
    char *buf,
    uint len);  // parse received radio data and do action if applicable
void dump_fram();

int main() {
    stdio_init_all();

    // set_sys_clock_48mhz(); // could reduce the system clock speed if needed
    // for power reasons, would rather have the performance

    // when DIO1 goes high on the radio interrupt with gpio_callback
    gpio_set_irq_enabled_with_callback(DIO1_PIN, GPIO_IRQ_EDGE_RISE, true,
                                       &gpio_callback);

    // setup_spi();  // init SPI peripheral for Radio and FRAM

    gpio_init(RADIO_RST);
    gpio_set_dir(RADIO_RST, GPIO_OUT);
    gpio_put(RADIO_RST, 1);

    ack_alarm_pool = alarm_pool_create_with_unused_hardware_alarm(4);
    vent_alarm_pool = alarm_pool_create_with_unused_hardware_alarm(4);

    sleep_ms(5000);

    // setup devices
    radio.radio_init();
    mem.mem_init();

    setup_vent_servo(SERVO_PWM);

    // Timers: the next two blocks create timers that trigger every 60 seconds,
    // they execute the timer_callback functions specified

    // negative timeout means exact delay (rather than delay between
    // callbacks)
    if (!add_repeating_timer_ms(-60000, tx_timer_callback, NULL, &tx_timer)) {
        printf("Failed to add timer\n");
        return -1;
    }

    if (!add_repeating_timer_ms(-60000, log_timer_callback, NULL, &log_timer)) {
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

    radio.radio_receive_cont();

    while (true) {
        watchdog_update();

        c = getchar_timeout_us(0);

        if (c == 'd') {
            sleep_ms(500);
            write_name_config();
            printf("GHOUL Flight (Compiled %s %s)\n", __DATE__, __TIME__);
            printf("Device ID: %d\n", mem.device_id);
            read_config(NAME, test_config, (uint8_t *)test_config.name,
                        sizeof(test_config.name), &mem);
            printf("DEVICE NAME: %s\n", test_config.name);
            dump_fram();
            printf("\nDump complete, press \"d\" to dump memory\n");
        }

        // timed events
        //  - transmit location and some status maybe
        //  - log LAT,LONG,Time,Fix

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

            parse_text_radio_cmd(radio_rx_buf, sizeof(radio_rx_buf));

            if (strncmp("ack", radio_rx_buf, 3) !=
                0) {  // if the receiving packet does not start with ack, send
                      // an ack
                printf("Starting ack timer\n");
                ack_alarm_id = alarm_pool_add_alarm_in_ms(
                    ack_alarm_pool, 1000, ack_timer_callback, NULL, true);
            }

            rx_done = false;
        }

        if (send_ack && tx_done) {
            printf("Received ACK\n");
            transmit((uint8_t *)ack, sizeof(ack));
            send_ack = false;
        }

        if (do_tx && tx_done) {
            transmit((uint8_t *)radio_tx_buf, sizeof(radio_tx_buf));
            do_tx = false;
        }

        if (cutdown) {
            // assert some pin to initiate cut
        }

        if (do_close_vent) {
            printf("Closing vent\n");
            vent_servo_close(true);
            do_close_vent = false;
        };
    }
}

// writes the string to FRAM using the config settings in the test_config struct
void write_name_config() {
    strcpy(test_config.name, "GHOUL 2024.0-0");
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
    return true;
}

static int64_t ack_timer_callback(alarm_id_t id, void *user_data) {
    send_ack = true;
    return 0;  // don't repeat
}

static int64_t vent_timer_callback(alarm_id_t id, void *user_data) {
    do_close_vent = true;
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

void parse_text_radio_cmd(char *buf, uint len) {
    static char separators[] = " -\t\f\r\v\n";
    char *token;
    int device_id = 0;

    token = strtok(buf, separators);

    if (strcmp(token, "vent") == 0) {
        printf("Received Vent: ");

        // do vent things
        uint digit1 = 0;
        uint digit2 = 0;
        uint digit3 = 0;
        uint vent_time = 0;

        token = strtok(NULL, separators);

        if (strcmp(token, "open") == 0) {
            printf("open\n");
            char vent_log[] = "vent open received\n";
            mem.write_memory(log_addr, (uint8_t *)vent_log, sizeof(vent_log));
            log_addr = log_addr + sizeof(vent_log);

            vent_servo_open(true);
        } else if (strcmp(token, "close") == 0) {
            printf("close\n");
            char vent_log[] = "vent close received\n";
            mem.write_memory(log_addr, (uint8_t *)vent_log, sizeof(vent_log));
            log_addr = log_addr + sizeof(vent_log);

            vent_servo_close(true);
        } else if (strcmp(token, "time") == 0) {
            token = strtok(NULL, separators);

            digit1 = ((uint)(*(token + 0))) - 48;
            // token = token++;
            digit2 = ((uint)(*(token + 1))) - 48;
            // token = token++;
            digit3 = ((uint)(*(token + 2))) - 48;
            vent_time = (digit1 * 100) + (digit2 * 10) + digit3;

            printf("timed %d sec\n", vent_time);
            char vent_log[] = "vent timed received\n";
            mem.write_memory(log_addr, (uint8_t *)vent_log, sizeof(vent_log));
            log_addr = log_addr + sizeof(vent_log);

            vent_servo_open(true);
            vent_alarm_id =
                alarm_pool_add_alarm_in_ms(vent_alarm_pool, vent_time * 1000,
                                           vent_timer_callback, NULL, true);
        } else {
            printf("error\n");
            char vent_log[] = "vent error received\n";
            mem.write_memory(log_addr, (uint8_t *)vent_log, sizeof(vent_log));
            log_addr = log_addr + sizeof(vent_log);
        }

    } else if (strcmp(token, "ack") == 0) {
        // do nothing I guess
    } else if (strcmp(token, "cut") == 0) {
        char cut_log[] = "Cut Received\n";
        printf("Received Cut\n");
        mem.write_memory(log_addr, (uint8_t *)cut_log, sizeof(cut_log));
        log_addr = log_addr + sizeof(cut_log);
        // assert a GPIO
    } else {
    }
}

void dump_fram() {
    printf("Dumping FRAM\n");
    for (int addr = LOG_INIT_ADDR; addr <= LOG_MAX_ADDR; addr++) {
        watchdog_update();
        mem.read_memory(addr, &log_buf, 1);  // sizeof(log_buf)
        if (log_buf != 0) printf("%c", log_buf);
        // printf("%d - %c\n", log_addr, log_buf);
        // memset(log_buf, 0, sizeof(log_buf));
        log_buf = 0;
    }
}