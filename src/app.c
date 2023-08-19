/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "hardware/rtc.h"
#include "hardware/watchdog.h"
#include "hardware/i2c.h"
#include "pico/util/datetime.h"

#include "modbus.h"
#include "data.h"
#include "ssd1306_i2c.h"

#define VERSION 0x0001

/* configuration

UART0 (GP0, GP1) => modbus client RTU
USB CDC => console
I2C1 (GP14, GP15) => oled display
GP25 => led

*/

#define UART0_TX_PIN 0
#define UART0_RX_PIN 1


#define I2C1_SDA_PIN 14
#define I2C1_SCL_PIN 15
// 400 is usual, but often these can be overclocked to improve display response.
// Tested at 1000 on both 32 and 84 pixel height devices and it worked.
#define SSD1306_I2C_CLK             400
//#define SSD1306_I2C_CLK             1000

#define LED_PIN 25


char cmd_buf[512];
uint32_t u32_char_count = 0;

void hardware_init()
{
    // LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);


    // Set up UART0
    gpio_set_function(UART0_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX_PIN, GPIO_FUNC_UART);
    uart_init(uart0, 4800);
    uart_set_hw_flow(uart0, false, false);
    uart_set_fifo_enabled (uart0, true);

    // set up I2C
    // I2C is "open drain", pull ups to keep signal high when no data is being
    // sent
    gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA_PIN);
    gpio_pull_up(I2C1_SCL_PIN);
    i2c_init(i2c1, SSD1306_I2C_CLK * 1000);
    
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

    // default date
    datetime_t t = {
            .year  = 2023,
            .month = 01,
            .day   = 01,
            .dotw  = 0, // 0 is Sunday, so 5 is Friday
            .hour  = 00,
            .min   = 00,
            .sec   = 00
    };
 
    // Start the RTC
    rtc_init();
    rtc_set_datetime(&t);

    // wait for uart connection
    int16_t i16Timeout = 5000;
    while (!stdio_usb_connected() && (i16Timeout > 0)) {
        sleep_ms(100);
        i16Timeout -= 100;
    }
    
    printf("Routeur solaire v%d.%d (%s %s)\n", VERSION>>8, VERSION&0xFF, __DATE__, __TIME__);
    
    hardware_init();
    SSD1306_init();
    modbus_client_init();
    
    while (true) {
        blink_led();
        modbus_client_loop();
        data_loop();
        SSD1306_loop();

        // console
        int val = getchar_timeout_us(0);
        if( val != PICO_ERROR_TIMEOUT ) {
            char c = (char)val;
            if (c == '\r' ) {
                char* p_cmd = cmd_buf;
                // execute command
                cmd_buf[u32_char_count] = '\0';
                // get command
                char * p_first_space = strchr(cmd_buf, ' ');
                if(p_first_space != NULL) {
                    *p_first_space = '\0';
                }
                if( 0 == strcmp("reset", cmd_buf)) {
                    printf("Enable watchdog\n");
                    watchdog_enable(100,1);
                } else if( 0 == strcmp("simu", cmd_buf)) {
                    data_toggle_simu();
                } else if( 0 == strcmp("datetime", cmd_buf)) {
                    // no argument print datetime
                    if(NULL == p_first_space) {
                        datetime_t t;
                        char datetime_buf[256];
                        rtc_get_datetime(&t);
                        datetime_to_str(datetime_buf, sizeof(datetime_buf), &t);
                        printf("%s\n", datetime_buf);
                    } else {
                        // first argument is datetime in format YYYY-MM-DD-HH-MM-SS-dotw
                        char* p_datetime = p_first_space+1;
                        // trim end space
                        char * p_end_space = strchr(p_datetime, ' ');
                        if(p_end_space != NULL) {
                            *p_end_space = '\0';
                        }
                        // parse datetime
                        // newlib-nano doesn't support %hhd so use intermediate int
                        datetime_t t;
                        int val[7];
                        int nb_found = sscanf(p_datetime, "%d-%d-%d-%d-%d-%d-%d", &val[0], &val[1], &val[2], &val[3], &val[4], &val[5], &val[6]);
                        if( 7 == nb_found ) {
                            t.year=val[0];
                            t.month=val[1];
                            t.day=val[2];
                            t.hour=val[3];
                            t.min=val[4];
                            t.sec=val[5];
                            t.dotw=val[6];
                            // Set time
                            rtc_set_datetime(&t);
                            // clk_sys is >2000x faster than clk_rtc, so datetime is not updated immediately when rtc_get_datetime() is called.
                            // tbe delay is up to 3 RTC clock cycles (which is 64us with the default clock settings)
                            sleep_us(64);
                            // get and print new time
                            datetime_t newt;
                            char datetime_buf[256];
                            rtc_get_datetime(&newt);
                            datetime_to_str(datetime_buf, sizeof(datetime_buf), &newt);
                            printf("datetime set to %s\n", datetime_buf);
                        } else {
                            printf("Invalid format [%s] nb_found=%d\n", p_datetime, nb_found);
                            printf("%d-%d-%d-%d-%d-%d-%d\n", t.year, t.month, t.day, t.hour, t.min, t.sec, t.dotw);
                        }
                    }


                } else {
                    printf("Unknown command [%s]\n", cmd_buf);
                }
                // reset command
                u32_char_count = 0;
            } else {
                // add char to command
                cmd_buf[u32_char_count] = c;
                u32_char_count++;
                if( u32_char_count >= (sizeof(cmd_buf)-1) ) {
                    // command too long
                    printf("Command too long\n");
                    u32_char_count = 0;
                }
            }
        }
    }
}
