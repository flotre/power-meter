#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/rtc.h"
#include "pico/util/datetime.h"
#include "modbus.h"

uint32_t u32_LastSendIndex = 0;
bool b_simu = false;

void data_loop(void) {
 
    // get power data 
    t_power_data* p_power_data = modbus_get_power_data();

    // send data if new recent data
    absolute_time_t cur_time = get_absolute_time();
    int64_t diff_us = absolute_time_diff_us(p_power_data->time, cur_time);
    // if data has less than one second
    if( ((diff_us < (1*1000*1000)) && (u32_LastSendIndex != p_power_data->u32_index)) || (b_simu) ) {
        u32_LastSendIndex = p_power_data->u32_index;
        // send data on stdout
        datetime_t t;
        char datetime_buf[256];
        rtc_get_datetime(&t);
        // format to iso 8601 YYYY-MM-DDTHH:MM:SS
        snprintf(datetime_buf, sizeof(datetime_buf), "%d-%02d-%02dT%02d:%02d:%02d", t.year, t.month, t.day, t.hour, t.min, t.sec);
        // use json format
        printf("{");
        printf("\"idx\":%u,", p_power_data->u32_index);
        printf("\"time\":\"%s\",", datetime_buf); //p_power_data->time);
        printf("\"V\":%u.%03u,", p_power_data->tension_mv/1000, p_power_data->tension_mv%1000);
        printf("\"F\":%u.%03u", p_power_data->frequence_mhz/1000, p_power_data->frequence_mhz%1000);
        for(int8_t i=0; i<2; i++) {
            printf(",\"I%d\":%u.%03u", i+1, p_power_data->voie[i].courant_ma/1000, p_power_data->voie[i].courant_ma%1000);
            printf(",\"P%d\":%u.%03u", i+1, p_power_data->voie[i].puissance_active_mw/1000, p_power_data->voie[i].puissance_active_mw%1000);
            printf(",\"E%d\":%u", i+1, p_power_data->voie[i].energie_wh);
            printf(",\"fp%d\":%u.%03u", i+1, p_power_data->voie[i].facteur_puissance/1000, p_power_data->voie[i].facteur_puissance%1000);
        }
        printf("}\n");
    }
}

void data_toggle_simu(void) {
    b_simu = !b_simu;
}