#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern "C" {
#include "../vent-servo.h"
}

#include "../GHOUL_2024_0.h"
#include "../libnmea/src/nmea/nmea.h"
#include "../rp2040-config/MB85RS1MT.h"
#include "../rp2040-config/config.h"
#include "../rp2040-drf1262-lib/SX1262.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/time.h"

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

// GPS
uint8_t gps_buf[100] = {0};
unsigned char pos = 0;  // Keep track of current position in radio_tx_buf
char *values[10];       // Array of char pointers, for pointing to positons in
                        // radio_tx_buf

// These structs have been largely lifted from libnmea.
/* NMEA cardinal direction types */
typedef char nmea_cardinal_t;
#define NMEA_CARDINAL_DIR_NORTH (nmea_cardinal_t)'N'
#define NMEA_CARDINAL_DIR_EAST (nmea_cardinal_t)'E'
#define NMEA_CARDINAL_DIR_SOUTH (nmea_cardinal_t)'S'
#define NMEA_CARDINAL_DIR_WEST (nmea_cardinal_t)'W'
#define NMEA_CARDINAL_DIR_UNKNOWN (nmea_cardinal_t)'\0'

#define NO_GPS_DATA 0xFF

#define NMEA_TIME_FORMAT "%H%M%S"
#define NMEA_TIME_FORMAT_LEN 6

#define TM_YEAR_START 1900
#define RMC_YEAR_START 2000

/* GPS position struct */
// typedef struct _nmea_position {
//     double minutes;  // don't like that this is stored as a double
//     int degrees;
//     nmea_cardinal_t cardinal;
// } nmea_position;

struct tm {
    int tm_sec;     /* seconds after the minute [0-60] */
    int tm_min;     /* minutes after the hour [0-59] */
    int tm_hour;    /* hours since midnight [0-23] */
    int tm_mday;    /* day of the month [1-31] */
    int tm_mon;     /* months since January [0-11] */
    int tm_year;    /* years since 1900 */
    int tm_wday;    /* days since Sunday [0-6] */
    int tm_yday;    /* days since January 1 [0-365] */
    int tm_isdst;   /* Daylight Savings Time flag */
    long tm_gmtoff; /* offset from UTC in seconds */
    char *tm_zone;  /* timezone abbreviation */
};

/* Pared down struct for GGA sentence data */
typedef struct _nmea_gga {
    struct tm time;
    nmea_position longitude;
    nmea_position latitude;
    unsigned char position_fix;
} nmea_gga;

nmea_gga data;
tm date;
nmea_position lon;
nmea_position lat;

// Misc
char c = 0;
uint32_t time_since_boot = 0;
char time_since_boot_str[10] = {0};
char new_line[] = "\n";

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
void write_fram();
static int _split_string_by_comma(char *string, char **values, int max_values);
int nmea_position_parse(char *s, nmea_position *pos);
nmea_cardinal_t nmea_cardinal_direction_parse(char *s);
int nmea_date_parse(char *s, struct tm *date);
int nmea_time_parse(char *s, struct tm *time);
void get_gps_data(void);

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

    gpio_init(EXTINT_PIN);
    gpio_set_dir(EXTINT_PIN, GPIO_IN);
    gpio_init(TIMEPULSE_PIN);
    gpio_set_dir(TIMEPULSE_PIN, GPIO_IN);

    // Initialize I2C port at 100 kHz
    i2c_init(i2c, 100 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    sleep_ms(5000);

    // setup devices
    radio.radio_init();
    mem.mem_init();

    setup_vent_servo(SERVO_PWM);
    vent_servo_close(true);

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

    // time_since_boot = to_ms_since_boot(get_absolute_time());
    sprintf(time_since_boot_str, "%d\n", to_ms_since_boot(get_absolute_time()));
    mem.write_memory(log_addr, (uint8_t *)time_since_boot_str,
                     strlen(time_since_boot_str) + 1);
    log_addr = log_addr + strlen(time_since_boot_str) + 1;

    mem.write_memory(log_addr, (uint8_t *)&new_line, sizeof(new_line));
    log_addr = log_addr + sizeof(new_line);

    while (true) {
        watchdog_update();

        c = getchar_timeout_us(0);

        if (c == 'd') {
            sleep_ms(500);
            // write_name_config();
            printf("GHOUL Flight (Compiled %s %s)\n", __DATE__, __TIME__);
            printf("Device ID: %d\n", mem.device_id);
            read_config(NAME, test_config, (uint8_t *)test_config.name,
                        sizeof(test_config.name), &mem);
            printf("DEVICE NAME: %s\n", test_config.name);
            dump_fram();
            printf(
                "\nDump complete, press \"d\" to dump memory, \"w\" to write "
                "memory\n");
        } else if (c == 'w') {
            sleep_ms(500);
            write_name_config();
            write_fram();
            printf(
                "\nWrite complete, press \"d\" to dump memory, \"w\" to write "
                "memory\n");
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
            mem.write_memory(log_addr, (uint8_t *)radio_rx_buf,
                             strlen(radio_rx_buf) + 1);
            log_addr = log_addr + sizeof(radio_rx_buf);

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

        get_gps_data();
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
    sprintf(time_since_boot_str, "%d:%d.%d\n", date.tm_hour, date.tm_min,
            date.tm_sec);
    mem.write_memory(log_addr, (uint8_t *)time_since_boot_str,
                     strlen(time_since_boot_str) + 1);
    log_addr = log_addr + strlen(time_since_boot_str) + 1;
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

void write_fram() {
    uint8_t empty = 0;
    printf("\nClearing FRAM\n");
    for (int addr = LOG_INIT_ADDR; addr <= LOG_MAX_ADDR; addr++) {
        watchdog_update();
        mem.write_memory(addr, &empty, 1);
    }
}

/**
 * Splits a string by comma.
 *
 * string is the string to split, will be manipulated. Needs to be
 *        null-terminated.
 * values is a char pointer array that will be filled with pointers to the
 *        splitted values in the string.
 * max_values is the maximum number of values to be parsed.
 *
 * Returns the number of values found in string.
 *
 * Copied from libnmea/src/nmea/nmea.c.
 */
static int _split_string_by_comma(char *string, char **values, int max_values) {
    int i = 0;

    values[i++] = string;
    while (i < max_values && NULL != (string = strchr(string, ','))) {
        *string = '\0';
        values[i++] = ++string;
    }

    return i;
}

/**
 * Parses a position (latitude or longitude). TODO: Need to modify if want
 * minutes as int.
 *
 * s is the string containing the data in string form.
 * pos is a pointer to an nmea_position struct which will be filled with the
 *      data.
 *
 * Copied from libnmea/src/parsers/parse.c.
 */
int nmea_position_parse(char *s, nmea_position *pos) {
    char *cursor;

    pos->degrees = 0;
    pos->minutes = 0;

    if (s == NULL || *s == '\0') {
        return -1;
    }

    /* decimal minutes */
    if (NULL == (cursor = strchr(s, '.'))) {
        return -1;
    }

    /* minutes starts 2 digits before dot */
    cursor -= 2;
    pos->minutes = atof(cursor);
    *cursor = '\0';

    /* integer degrees */
    cursor = s;
    pos->degrees = atoi(cursor);

    return 0;
}

/**
 * Parses a cardinal direction.
 *
 * s is a pointer to a char, which contains the data to be parsed.
 *
 * Returns an nmea_cardinal_t (basically an enum).
 *
 * Copied from libnmea/src/parsers/parse.c.
 */
nmea_cardinal_t nmea_cardinal_direction_parse(char *s) {
    if (NULL == s || '\0' == *s) {
        return NMEA_CARDINAL_DIR_UNKNOWN;
    }

    switch (*s) {
        case NMEA_CARDINAL_DIR_NORTH:
            return NMEA_CARDINAL_DIR_NORTH;
        case NMEA_CARDINAL_DIR_EAST:
            return NMEA_CARDINAL_DIR_EAST;
        case NMEA_CARDINAL_DIR_SOUTH:
            return NMEA_CARDINAL_DIR_SOUTH;
        case NMEA_CARDINAL_DIR_WEST:
            return NMEA_CARDINAL_DIR_WEST;
        default:
            break;
    }

    return NMEA_CARDINAL_DIR_UNKNOWN;
}

/**
 * Parses a date.
 *
 * s is the string containng the date.
 * date is a pointer to a struct that gets filled with data.
 *
 * Returns 0 on success and -1 on failure.
 *
 * Copied from libnmea/src/parsers/parse.c.
 */
int nmea_date_parse(char *s, struct tm *date) {
    char *rv;
    uint32_t x;

    if (s == NULL || *s == '\0') {
        return -1;
    }

    x = strtoul(s, &rv, 10);
    date->tm_mday = x / 10000;
    date->tm_mon = ((x % 10000) / 100) - 1;
    date->tm_year = x % 100;

    // Normalize tm_year according to C standard library
    if (date->tm_year > 1900) {  // ZDA message case
        date->tm_year -= TM_YEAR_START;
    } else {  // RMC message case
        date->tm_year += (RMC_YEAR_START - TM_YEAR_START);
    }

    return 0;
}

int nmea_time_parse(char *s, struct tm *time) {
    char *rv;
    uint32_t x;

    if (s == NULL || *s == '\0') {
        return -1;
    }

    x = strtoul(s, &rv, 10);
    time->tm_hour = x / 10000;
    time->tm_min = (x % 10000) / 100;
    time->tm_sec = x % 100;
    if (time->tm_hour > 23 || time->tm_min > 59 || time->tm_sec > 59 ||
        (int)(rv - s) < NMEA_TIME_FORMAT_LEN) {
        return -1;
    }
    if (*rv == '.') {
        /* TODO There is a sub-second field. */
    }

    return 0;
}

void get_gps_data() {
    int result = PICO_ERROR_GENERIC;
    uint8_t rx_msg = 0;
    result = i2c_read_blocking(i2c, GPS_ADDR, &rx_msg, 1, false);
    if (result == PICO_ERROR_GENERIC)
        printf("\ni2c error occurred %x\n\n", result);
    else {
        if (rx_msg != NO_GPS_DATA) {
            // printf("%c", rx_msg);
            // End sequence is "\r\n"
            gps_buf[pos++] = rx_msg;
            if (rx_msg == '\n') {
                // We've reached the end of the sentence/line.
                gps_buf[pos++] = '\0';  // NULL-terminate for safety
                // Parse NMEA sentence, only if GNGGA for now
                if (strlen((char *)gps_buf) >= 6 &&
                    strncmp((char *)(&gps_buf[3]), "GGA", 3) == 0) {
                    // printf("%s\n", gps_buf);
                    // Split msg into values, parse each value, then assign
                    // each result to appropriate place in struct
                    _split_string_by_comma((char *)gps_buf, values, 7);
                    // nmea_gga data;
                    // tm date;
                    // nmea_position lon;
                    // nmea_position lat;
                    // nmea_date_parse(values[1], &date);
                    nmea_time_parse(values[1], &date);
                    data.time = date;
                    nmea_position_parse(values[4], &lon);
                    lon.cardinal = nmea_cardinal_direction_parse(values[5]);
                    data.longitude = lon;
                    nmea_position_parse(values[2], &lat);
                    lon.cardinal = nmea_cardinal_direction_parse(values[3]);
                    data.latitude = lat;
                    data.position_fix = atoi(values[6]);
                }
                pos = 0;  // Reset pos for reading in the next sentence
            }
        }
    }
}