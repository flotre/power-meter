#ifndef MODBUS_H__
#define MODBUS_H__

#define MODBUS_SERVER_UART uart0
#define MODBUS_CLIENT_UART uart1

void modbus_client_init(void);
void modbus_client_loop(void);

void modbus_server_init(void);
void modbus_server_loop(void);

#endif // MODBUS_H__
