#ifndef _UART_COMM_H_
#define _UART_COMM_H_

#include "ringbuffer.h"
#include "type.h"


int uart_send_data(uint8_t *data, int len);

int uartComm_init(void);
void uartComm_deinit(void);


#endif  //_UART_COMM_H_