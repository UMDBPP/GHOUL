#include <stdio.h>
#include <string.h>

#include "../GHOUL_2024_0.h"
#include "../rp2040-drf1262-lib/SX1262.h"
#include "hardware/i2c.h"
// #include "nmea/gpgga.h"
// #include "nmea/gpgll.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#define NO_GPS_DATA 0xFF

// Registers
static const uint8_t REG_NUM_BYTES_MSB = 0xFD;
static const uint8_t REG_NUM_BYTES_LSB = 0xFE;
static const uint8_t REG_DATA = 0xFF;

volatile bool transmit;

repeating_timer_t tx_timer;

uint8_t radio_tx_buf[100] = "hello!";

// Ports
i2c_inst_t *i2c = i2c0;

DRF1262 radio(spi1, RADIO_CS, SCK_PIN, MOSI_PIN, MISO_PIN, TXEN_PIN, DIO1_PIN,
              BUSY_PIN, SW_PIN, RADIO_RST);

bool tx_timer_callback(repeating_timer_t *rt);
void setup_led();
void led_on();
void led_off();
void transmit_test(uint8_t *buf, size_t len);

int main() {
    // Pins
    const uint sda_pin = SDA_PIN;
    const uint scl_pin = SCL_PIN;

    uint result1 = 0;
    uint result2 = 0;

    stdio_init_all();

    setup_led();
    led_off();

    gpio_init(EXTINT_PIN);
    gpio_set_dir(EXTINT_PIN, GPIO_IN);
    gpio_init(TIMEPULSE_PIN);
    gpio_set_dir(TIMEPULSE_PIN, GPIO_IN);

    gpio_init(RADIO_RST);
    gpio_set_dir(RADIO_RST, GPIO_OUT);
    gpio_put(RADIO_RST, 1);

    sleep_ms(5000);

    radio.debug_msg_en = 0;
    radio.radio_init();

    // negative timeout means exact delay (rather than delay between
    // callbacks)
    if (!add_repeating_timer_us(-20000000, tx_timer_callback, NULL,
                                &tx_timer)) {
        printf("Failed to add timer\n");
        return 1;
    }

    // Initialize I2C port at 100 kHz
    i2c_init(i2c, 100 * 1000);

    // Initialize I2C pins
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    uint8_t rx_msg = 0;

    while (true) {
        // printf("%c", uart_getc(uart1));

        if (transmit) {
            transmit_test((uint8_t *)radio_tx_buf, sizeof(radio_tx_buf));
            transmit = false;
        }

        result2 = i2c_read_blocking(i2c, GPS_ADDR, &rx_msg, 1, false);
        if (result2 == PICO_ERROR_GENERIC)
            printf("\ni2c error occurred %x\n\n", result2);
        else {
            if (rx_msg != NO_GPS_DATA) printf("%c", rx_msg);
        }
    }
}

bool tx_timer_callback(repeating_timer_t *rt) {
    transmit = true;

    return true;  // keep repeating
}

void transmit_test(uint8_t *buf, size_t len) {
    printf("Transmit Test\n");

    led_on();

    // tx_done = false;

    // buf[0] = (char)get_rand_32();

    radio.radio_send(buf, len);

    while (gpio_get(BUSY_PIN) && !gpio_get(DIO1_PIN))
        ;

    printf("Starting wait\n");

    sleep_ms(10000);

    led_off();
    printf("%s\n", (char *)buf);
    // radio.disable_tx();
    // radio.radio_receive_single();

    radio.get_radio_errors();

    radio.clear_irq_status();

    radio.radio_receive_single();

    radio.get_radio_errors();
}

void setup_led() {
    gpio_init(LED1_PIN);
    gpio_set_dir(0, GPIO_OUT);
    gpio_put(LED1_PIN, 0);
}

void led_on() { gpio_put(LED1_PIN, true); }

void led_off() { gpio_put(LED1_PIN, false); }