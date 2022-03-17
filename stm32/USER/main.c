#include <stdio.h>
#include <string.h>
#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "usart3.h"
#include "ringbuffer.h"
#include "uart_comm.h"
#include "cJSON.h"
#include "protocol.h"
#include "config.h"


extern unsigned char ring_buf[512];
extern struct ringbuffer recv_ringbuf;


int main(void)
{

	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 //���ڳ�ʼ��Ϊ115200

	uartComm_init();
	proto_init(uart_send_data);

	printf("system init ok\r\n");

	while(1)
	{

		proto_recv_handel(&recv_ringbuf);

	}	 
}

