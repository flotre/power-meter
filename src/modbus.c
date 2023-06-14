#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "modbus.h"


/*
Modbus frame format
A Modbus "frame" consists of an Application Data Unit (ADU), which encapsulates a Protocol Data Unit (PDU):[8]

    ADU = Address + PDU + Error check.
    PDU = Function code + Data.

In Modbus data frames, the most significant byte of a multi-byte value is sent before the others.


Address 	1 	Station address
Function 	1 	Indicates the function code e.g. "read coils"
Data 	n × 1 	Data + length will be filled depending on the message type
CRC 	    2 	Cyclic redundancy check

*/

/*****************************************************************************/
/*            CONST                                                          */
/*****************************************************************************/
#define MODBUS_FRAME_SIZE 256
#define NB_POWER_DATA 60
/*****************************************************************************/
/*            PRIVATE TYPE                                                   */
/*****************************************************************************/

enum mb_state {
    MODBUS_WAIT_SOF=0,
    MODBUS_WAIT_FUNCTION,
    MODBUS_WAIT_DATA_SIZE,
    MODBUS_WAIT_DATA,
    MODBUS_WAIT_CRC
};

typedef void (*t_rx_cb)(uint8_t*, uint8_t);

typedef struct
{
    enum mb_state state;
    uint8_t u8_function;
    absolute_time_t sof_time;
    uint8_t mb_frame[MODBUS_FRAME_SIZE];
    uint8_t u8_frame_size;
    uint8_t u8_frame_expected_size;
    uart_inst_t *uart;
    t_rx_cb rx_cb;
}t_mb_ctx;


/*****************************************************************************/
/*            PRIVATE FUNCTION                                               */
/*****************************************************************************/
uint16_t modbus_crc16(uint8_t *buffer, uint16_t buffer_length);
uint32_t bytes_to_uint32(uint8_t* pbuf);

void modbus_client_rx_cb(uint8_t * pbuf, uint8_t size);
void modbus_rx_loop(t_mb_ctx* ctx);

/*****************************************************************************/
/*            PRIVATE VAR                                                    */
/*****************************************************************************/
static t_mb_ctx mb_ctx_client;
static t_power_data power_data[NB_POWER_DATA];
static uint32_t u32_power_data_idx = 0;
static absolute_time_t send_time = 0;

/*****************************************************************************/
/*            FUNCTION DEFINITION                                            */
/*****************************************************************************/
void modbus_print_frame(uint8_t* p_frame, uint8_t u8_size) {
    // print frame
    for(int i=0;i<u8_size;i++) {
        printf("%02X ", p_frame[i]);
    }
    printf("\n");
}


t_power_data* modbus_get_power_data(void) {
    uint8_t u8_power_data_idx = (u32_power_data_idx-1) % NB_POWER_DATA;
    
    return &power_data[u8_power_data_idx];
}


void modbus_send_blocking(t_mb_ctx *ctx, uint8_t* buf, uint8_t size) {
    uint16_t u16_crc = modbus_crc16(buf, size-2);
    buf[size-2] = (uint8_t)(u16_crc >> 8);
    buf[size-1] = (uint8_t)u16_crc;
    uart_write_blocking (ctx->uart, buf, size);
}


void modbus_ctx_init(t_mb_ctx *ctx, uart_inst_t *uart, t_rx_cb rx_cb) {
    memset(ctx, 0, sizeof(ctx));
    ctx->state = MODBUS_WAIT_SOF;
    ctx->uart = uart;
    ctx->rx_cb = rx_cb;
}

void modbus_client_init(void) {
    memset(&power_data, 0, sizeof(power_data));
    modbus_ctx_init(&mb_ctx_client, MODBUS_CLIENT_UART, modbus_client_rx_cb);

    // read sensor properties
    uint8_t request[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x04, 0, 0};
    modbus_send_blocking(&mb_ctx_client, request, sizeof(request));
    send_time = get_absolute_time();
}


void modbus_client_loop(void) {
    const int64_t SEND_PERIOD_US = 1000*1000; // 1000ms
    // read register 0x0048 -> 0x0048+0x000E
    const uint8_t request[] = {0x01, 0x03, 0x00, 0x48, 0x00, 0x0E, 0x44, 0x18};

    // blink LED
    absolute_time_t cur_time = get_absolute_time();
    int64_t diff_us = absolute_time_diff_us(send_time, cur_time);
    if( diff_us > SEND_PERIOD_US ) {
        send_time = cur_time;
        // read current, power, ...
        uart_write_blocking (MODBUS_CLIENT_UART, request, sizeof(request));
    }

    // RX
    modbus_rx_loop(&mb_ctx_client);

}

void modbus_client_rx_cb(uint8_t * pbuf, uint8_t size) {
    // print frame
    printf("CMB RX ");
    modbus_print_frame(pbuf, size);
    uint8_t u8_address = pbuf[0];
    uint8_t u8_function_code = pbuf[1];
    // check if frame match the request we send, ignore other frame
    if((u8_function_code == 3) && (size==5+4*0xE)) {
        //uint8_t u8_data_size = pbuf[2];

        // build data struct
        uint8_t u8_power_data_idx = u32_power_data_idx % NB_POWER_DATA;
        t_power_data* p_data = &power_data[u8_power_data_idx];
        
        //common value
        p_data->u32_index = u32_power_data_idx++;
        p_data->time = get_absolute_time();
        p_data->tension_mv = bytes_to_uint32(&pbuf[3]) / 10;
        p_data->frequence_mhz = bytes_to_uint32(&pbuf[31]) * 10;
        // voie 1
        p_data->voie[0].courant_ma = bytes_to_uint32(&pbuf[7]) / 10;
        p_data->voie[0].puissance_active_mw = bytes_to_uint32(&pbuf[11]) / 10;
        if( pbuf[27] ) {
            p_data->voie[0].puissance_active_mw = -p_data->voie[0].puissance_active_mw;
        }
        p_data->voie[0].energie_wh = bytes_to_uint32(&pbuf[15]) / 10;
        p_data->voie[0].facteur_puissance = bytes_to_uint32(&pbuf[19]);
        // voie 2
        p_data->voie[1].courant_ma = bytes_to_uint32(&pbuf[39]) / 10;
        p_data->voie[1].puissance_active_mw = bytes_to_uint32(&pbuf[43]) / 10;
        if( pbuf[28] ) {
            p_data->voie[1].puissance_active_mw = -p_data->voie[1].puissance_active_mw;
        }
        p_data->voie[1].energie_wh = bytes_to_uint32(&pbuf[47]) / 10;
        p_data->voie[1].facteur_puissance = bytes_to_uint32(&pbuf[51]);
    }
}


void modbus_rx_loop(t_mb_ctx* ctx) {

    while( uart_is_readable(ctx->uart) ) {
        uint8_t byte = (uint8_t) uart_get_hw(ctx->uart)->dr;
        // check max size
        if(ctx->u8_frame_size >= MODBUS_FRAME_SIZE) {
            // frame too long, timeout will reset comm
            break;
        }

        ctx->mb_frame[ctx->u8_frame_size++] = byte;

        switch(ctx->state) {
            case MODBUS_WAIT_SOF:
                ctx->sof_time = get_absolute_time();
                ctx->state = MODBUS_WAIT_FUNCTION;
                break;

            case MODBUS_WAIT_FUNCTION:
            {
                ctx->u8_function = byte;
                ctx->state = MODBUS_WAIT_DATA_SIZE;
                // check for error
                if(byte&0x80) {
                    // exception
                    ctx->u8_frame_expected_size = 7;
                } else {
                    // we need to read one more byte
                    ctx->u8_frame_expected_size = 3;
                }
                break;
            }

            case MODBUS_WAIT_DATA_SIZE:
                // TODO adjust expected size if fc > 10
                // header(3) + crc(2)
                ctx->u8_frame_expected_size = byte + 5;
                ctx->state = MODBUS_WAIT_DATA;
                break;

            case MODBUS_WAIT_DATA:
                // TODO adjust expected size if fc > 10
                if( ctx->u8_frame_size >= (ctx->u8_frame_expected_size-2)) {
                    ctx->state = MODBUS_WAIT_CRC;
                }
                
                break;
            
            case MODBUS_WAIT_CRC:
            {
                if( ctx->u8_frame_size >= ctx->u8_frame_expected_size ) {
                    // compute CRC16
                    uint16_t crc = modbus_crc16(ctx->mb_frame, ctx->u8_frame_size-2);
                    if( (ctx->mb_frame[ctx->u8_frame_size-2] == (crc >> 8)) && (ctx->mb_frame[ctx->u8_frame_size-1] == (crc & 0x00ff)) ) {
                        if(ctx->u8_function&0x80) {
                            // exception
                            printf("MB RX Error code=%02X Exception code=%02X\n", ctx->mb_frame[1], ctx->mb_frame[2]);
                        }
                        // callback
                        ctx->rx_cb(ctx->mb_frame, ctx->u8_frame_size);
                    } else {
                        printf("modbus bad crc %04X\n", crc);
                        modbus_print_frame(ctx->mb_frame, ctx->u8_frame_size);
                        return;
                    }
                    ctx->state = MODBUS_WAIT_SOF;
                    ctx->u8_frame_size = 0;
                }
                break;
            }
        }
    }

    // 1s timeout
    if( ctx->state != MODBUS_WAIT_SOF ) {
        absolute_time_t cur_time = get_absolute_time();
        int64_t frame_diff_us = absolute_time_diff_us(ctx->sof_time, cur_time);
        if( frame_diff_us > (1*1000*1000) ) {
            // cancel frame
            ctx->state = MODBUS_WAIT_SOF;
            ctx->u8_frame_size = 0;
        }
    }
}

static const uint8_t table_crc_hi[] = {

    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};




uint16_t modbus_crc16(uint8_t *buffer, uint16_t buffer_length)
{
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    unsigned int i; /* will index into CRC lookup */

    /* pass through message buffer */
    while (buffer_length--) {
        i = crc_hi ^ *buffer++; /* calculate the CRC  */
        crc_hi = crc_lo ^ table_crc_hi[i];
        crc_lo = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}

uint32_t bytes_to_uint32(uint8_t* pbuf) {
    return (((uint32_t)pbuf[0])<<24) + (((uint32_t)pbuf[1])<<16)  + (((uint32_t)pbuf[2])<<8) + pbuf[3];
}