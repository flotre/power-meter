#ifndef MODBUS_H__
#define MODBUS_H__
#include "pico/stdlib.h"


#define MODBUS_SERVER_UART uart0
#define MODBUS_CLIENT_UART uart1


typedef struct
{
    uint32_t courant_ma;
    int32_t puissance_active_mw;
    uint32_t energie_wh;
    uint32_t facteur_puissance;
}t_channel_data;
typedef struct
{
    absolute_time_t time;
    uint32_t u32_index;
    uint32_t tension_mv;
    uint32_t frequence_mhz;
    t_channel_data voie[2];
    
}t_power_data;

void modbus_client_init(void);
void modbus_client_loop(void);


t_power_data* modbus_get_power_data(void);

#endif // MODBUS_H__
