/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"

#include "modbus.h"

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


    // Set up UART0
    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);
    uart_init(uart0, 115200);
    uart_set_hw_flow(uart0, false, false);
    uart_set_fifo_enabled (uart0, true);
    
    // Set up UART1
    gpio_set_function(UART1_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART1_RX_PIN, GPIO_FUNC_UART);
    uart_init(uart1, 3800);
    uart_set_hw_flow(uart1, false, false);
    uart_set_fifo_enabled (uart1, true);
    
}

void blink_led(void) {
    static bool b_led_state = false;
    static absolute_time_t blink_time = 0;
    int64_t blink_period_us = 500*1000; // 500ms

    // blink LED
    absolute_time_t cur_time = get_absolute_time();
    int64_t blink_diff_us = absolute_time_diff_us(blink_time, cur_time);
    if( blink_diff_us > blink_period_us ) {
        blink_time = cur_time;
        gpio_put(LED_PIN, b_led_state);
        b_led_state = !b_led_state;
    }
}



int main() {
    stdio_init_all();
    printf("Routeur solaire v%d.%d (%s %s)\n", VERSION>>8, VERSION&0xFF, __DATE__, __TIME__);
    
    hardware_init();
    modbus_client_init();
    modbus_server_init();
    
    while (true) {
        blink_led();
        modbus_server_loop();
        modbus_client_loop();
        // update_dimmer_output_loop
    }
}
