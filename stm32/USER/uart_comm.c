#include <stdio.h>
#include <string.h>
#include "delay.h"
#include "usart3.h"
#include "ringbuffer.h"
#include "uart_comm.h"


// uart receive data buffer
unsigned char ring_buf[512];
struct ringbuffer recv_ringbuf;


int uart_send_data(uint8_t *data, int len)
{
    // send data
    uart3_send(data, len);

    return 0;
}

int uartComm_init(void)
{
    ringbuf_init(&recv_ringbuf, ring_buf, sizeof(ring_buf));
	usart3_init(115200);

    printf("%s ok\r\n", __FUNCTION__);
    return 0;
}

void uartComm_deinit(void)
{

}
