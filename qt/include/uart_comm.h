#ifndef _UART_COMM_H_
#define _UART_COMM_H_

#include "type.h"

/* C++ include C */
#ifdef __cplusplus
extern "C" {
#endif
#include "ringbuffer.h"
#include "protocol.h"
#ifdef __cplusplus
}
#endif


#define CLI_SENDBUF_SIZE    1024
#define CLI_RECVBUF_SIZE	(CLI_SENDBUF_SIZE*2)


#define HEARTBEAT_INTERVAL_S		10


typedef enum
{
	STATE_OPENED,		//打开成功
	STATE_LOGIN,		//登陆成功
	STATE_CLOSE,		//关闭
}uart_state_e;



struct uartCommInfo
{
	int fd;
	pthread_mutex_t	send_mutex;
	struct ringbuffer recv_ringbuf;			// socket receive data ring buffer
	proto_detect_t proto_detect;
	uart_state_e state;		// client state
};


int uart_comm_start(char *dev_name);

void uart_comm_stop(void);


#endif	// _UART_COMM_H_
