/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"

#define VERSION 0x0001

/* configuration

UART0 (GP0, GP1) => modbus server RTU
USB CDC => console
UART1 (GP4, GP5) => modbus client RTU
I2C1 (GP14, GP15) => oled display

GP10 => zero crossing input
GP11 => dimmer output

GP25 => led

*/
#define UART0_TX_PIN 0
#define UART0_RX_PIN 1

#define UART1_TX_PIN 4
#define UART1_RX_PIN 5

#define I2C1_SDA_PIN 14
#define I2C1_SCL_PIN 15

#define LED_PIN 25


void hardware_init()
{
    // LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);


    // Set up our UART1
    uart_init(uart1, 3800);
    gpio_set_function(UART1_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART1_RX_PIN, GPIO_FUNC_UART);
    
    // Set up our UART1
    uart_init(uart1, 3800);
    gpio_set_function(UART1_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART1_RX_PIN, GPIO_FUNC_UART);
}

void blink_led(void) {
    static bool b_led_state = false;
    static absolute_time_t blink_time = 0;

    // blink LED
    absolute_time_t cur_time = get_absolute_time();
    int64_t blink_diff_us = absolute_time_diff_us (blink_time, cur_time);
    if( blink_diff_us > 500000 ) {
        gpio_put(LED_PIN, b_led_state);
        b_led_state = !b_led_state;
        blink_time = cur_time;
    }
}

int main() {
    stdio_init_all();
    printf("Routeur solaire v%d.%d (%s %s)\n", VERSION>>8, VERSION&0xFF, __DATE__, __TIME__);
    hardware_init();

    
    while (true) {
        blink_led();
        
    }
}
