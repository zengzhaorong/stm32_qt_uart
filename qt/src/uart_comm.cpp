#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include "uart_comm.h"
#include "mainwindow.h"
#include "protocol.h"
#include "config.h"


/* C++ include C */
#ifdef __cplusplus
extern "C" {
#endif
#include "ringbuffer.h"
#ifdef __cplusplus
}
#endif

uint8_t proto_recv_buf[PROTO_PACK_MAX_LEN] = {0};
uint8_t proto_ack_buf[PROTO_PACK_MAX_LEN] = {0};

struct uartCommInfo uart_info = {0};

int uart_0x03_heartbeat(struct uartCommInfo *client, uint8_t *data, int len, uint8_t *ack_data, int size, int *ack_len)
{
	printf("%s enter ++\n", __FUNCTION__);
	return 0;
}

int uart_send_data(void *arg, uint8_t *data, int len)
{
	struct uartCommInfo *client = (struct uartCommInfo *)arg;
	int total = 0;
	int ret;

	if(data==NULL || client->state==STATE_CLOSE)
		return -1;

	// lock
	pthread_mutex_lock(&client->send_mutex);
	do{
		ret = write(client->fd, data +total, len -total);
		total += ret;
	}while(total < len);
	// unlock
	pthread_mutex_unlock(&client->send_mutex);

	return total;
}

int uart_recv_data(struct uartCommInfo *client)
{
	uint8_t tmpBuf[128];
	int len, space;
	int ret = 0;

	space = ringbuf_space(&client->recv_ringbuf);

	memset(tmpBuf, 0, sizeof(tmpBuf));
	len = read(client->fd, tmpBuf, (int)sizeof(tmpBuf)>space ? space:(int)sizeof(tmpBuf));
	if(len > 0)
	{
		ret = ringbuf_write(&client->recv_ringbuf, tmpBuf, len);
	}

	return ret;
}

static int proto_packet_handle(struct uartCommInfo *client, uint8_t *pack, uint32_t len)
{
	uint8_t seq = 0, cmd = 0;
	uint8_t *data = NULL;
	int data_len = 0;
    int ack_len = 0;
    int ret;

    ret = proto_packet_analy(pack, len, &seq, &cmd, &data_len, &data);
	if(ret != 0)
		return -1;

	printf("ptoto cmd: 0x%02x, seq: %d, pack_len: %d, data_len: %d\n", cmd, seq, len, data_len);

    switch(cmd)
    {
        case 0x01:
            break;

        case 0x02:
            break;

        case 0x03:
			ret = uart_0x03_heartbeat(client, data, data_len, proto_ack_buf, sizeof(proto_ack_buf), &ack_len);
            break;
        
        default:
            break;
    }
	
	/* send ack data */
	if(ret==0 && ack_len>0)
	{
		proto_makeup_packet(seq, cmd, ack_len, proto_ack_buf, proto_recv_buf, PROTO_PACK_MAX_LEN, &data_len);
		uart_send_data(client, proto_recv_buf, data_len);
	}

    return 0;
}

int client_proto_handel(struct uartCommInfo *client)
{
    int proto_len;
	int recv_ret;
	int det_ret;

	// ????????????
	recv_ret = uart_recv_data(client);

    // ?????????????????????
    det_ret = proto_packet_detect(&client->recv_ringbuf, &client->proto_detect, proto_recv_buf, sizeof(proto_recv_buf), &proto_len);
    if(det_ret == 0)
    {
        //?????????????????????
        proto_packet_handle(client, proto_recv_buf, proto_len);
    }

	if(recv_ret<=0 && det_ret!=0)
	{
		usleep(30*1000);
	}

    return 0;
}

/**
*@brief  ????????????????????????
*@param  fd     ?????? int  ???????????????????????????
*@param  speed  ?????? int  ????????????
*@return  void
*/
const int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
          		   B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300, };
const int name_arr[] = {115200, 38400, 19200, 9600, 4800, 2400, 1200,  300, 
		  		  115200, 38400, 19200, 9600, 4800, 2400, 1200,  300, };

void set_speed(int fd, int speed)
{
  int   i; 
  int   status; 
  struct termios   Opt;
  tcgetattr(fd, &Opt); //??????????????????
  for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) { 
    if  (speed == name_arr[i]) {     
      tcflush(fd, TCIOFLUSH);     
      cfsetispeed(&Opt, speed_arr[i]); //????????????????????? 
      cfsetospeed(&Opt, speed_arr[i]); //?????????????????????  
      status = tcsetattr(fd, TCSANOW, &Opt);  //??????????????????
      if  (status != 0) {        
        perror("tcsetattr fd1");  
        return;     
      }    
      tcflush(fd,TCIOFLUSH);   
    }  
  }
}

/**
*@brief   ?????????????????????????????????????????????
*@param  fd     ??????  int  ???????????????????????????
*@param  databits ??????  int ?????????   ?????? ??? 7 ??????8
*@param  stopbits ??????  int ?????????   ????????? 1 ??????2
*@param  parity  ??????  int  ???????????? ?????????N,E,O,,S
*/
int set_parity(int fd,int databits,int stopbits,int parity)
{ 
	struct termios options; 
	if(tcgetattr(fd,&options)  !=  0) { 
		perror("SetupSerial 1");     
		return -1;  
	}

	options.c_cflag &= ~CSIZE; 
	switch (databits) /*??????????????????*/
	{   
	case 7:		
		options.c_cflag |= CS7; 
		break;
	case 8:
		options.c_cflag |= CS8;
		break;
	default:
		fprintf(stderr,"Unsupported data size\n"); return -1;  
	}
	
	switch (parity) 
	{
		case 'n':
		case 'N':    
			options.c_cflag &= ~PARENB;   /* Clear parity enable */
			options.c_iflag &= ~INPCK;     /* Enable parity checking */ 
			break;  
		case 'o':   
		case 'O':     
			options.c_cflag |= (PARODD | PARENB); /* ??????????????????*/  
			options.c_iflag |= INPCK;             /* Disnable parity checking */ 
			break;  
		case 'e':  
		case 'E':   
			options.c_cflag |= PARENB;     /* Enable parity */    
			options.c_cflag &= ~PARODD;   /* ??????????????????*/     
			options.c_iflag |= INPCK;       /* Disnable parity checking */
			break;
		case 'S': 
		case 's':  /*as no parity*/   
			options.c_cflag &= ~PARENB;
			options.c_cflag &= ~CSTOPB;break;  
		default:   
			fprintf(stderr,"Unsupported parity\n");    
			return -1;  
	}
	
	/* ???????????????*/  
	switch (stopbits)
	{   
		case 1:    
			options.c_cflag &= ~CSTOPB;  
			break;  
		case 2:    
			options.c_cflag |= CSTOPB;  
		   break;
		default:    
			 fprintf(stderr,"Unsupported stop bits\n");  
			 return -1; 
	}
	
	/* Set input parity option */ 
	if (parity != 'n')   
		options.c_iflag |= INPCK; 
	tcflush(fd,TCIFLUSH);

    //?????????????????????????????????
	options.c_cc[VTIME] = 3*10; /* ????????????10 seconds(??????1/10s 100ms)*/   
	options.c_cc[VMIN] = 0; 

//  options.c_cc[VMIN] = FRAME_MAXSIZE;   //?????????????????????
    //?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????(Raw Mode)??????????????????~ICANON???????????????????????????,?????????????????????
    options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
    options.c_oflag  &= ~OPOST;   /*Output*/

	options.c_iflag &= ~(INLCR | ICRNL); //???????????????????????????
	options.c_iflag &= ~(IXON | IXOFF | IXANY); //?????????????????????

    /* Update the options and do it NOW */
	if (tcsetattr(fd,TCSANOW,&options) != 0)   
	{ 
		perror("SetupSerial 3");   
		return -1;  
	} 

	return 0;
}

int uart_dev_init(struct uartCommInfo *uartCom, char *uart_dev)
{
	int ret;

	uartCom->fd = open(uart_dev, O_RDWR|O_NOCTTY);
	if(uartCom->fd < 0)
	{
		perror(uart_dev);
		return -1;
	}

	set_speed(uartCom->fd, DEFAULT_UART_BAUDRATE);
	ret = set_parity(uartCom->fd, 8, 1, 'N');
	if(ret != 0)
	{
		perror(uart_dev);
		return -1;
	}

	ringbuf_init(&uartCom->recv_ringbuf, PROTO_PACK_MAX_LEN *2);

	printf("%s: fd=%d, success\n", __FUNCTION__, uartCom->fd);

	return 0;
}

void *uart_comm_task(void *arg)
{
    struct uartCommInfo *client = &uart_info;
	char uart_dev[32] = {0};
	time_t heartbeat_time = 0;
	time_t init_time = 0;
	time_t curTime;
    int ret;

    printf("%s enter +_+\n", __FUNCTION__);

	strcpy(uart_dev, (char *)arg);
	uart_dev_init(client, uart_dev);

	proto_init(uart_send_data, client);

	init_time = time(NULL);
    while(client->state != STATE_CLOSE)
    {
		curTime = time(NULL);
		switch (client->state)
		{
			case STATE_OPENED:
				if(abs(curTime - heartbeat_time) >= HEARTBEAT_INTERVAL_S)
				{
					proto_0x03_sendHeartBeat();
					printf("heartbear ...\n");
					heartbeat_time = curTime;
				}
				break;

			case STATE_LOGIN:
				break;
			
			default:
				break;
		}

		if(client->state == STATE_OPENED)
		{
			client_proto_handel(client);
		}

		usleep(200*1000);

    }

	mainwin_set_disconnect();
    printf("%s exit -_-\n", __FUNCTION__);

    return NULL;
}

/* satrt uart commnuicate - ??????uart??????????????????*/
int uart_comm_start(char *dev_name)
{
	static char uart_dev[32] = {0};
	pthread_t tid;
	int ret;

	strcpy(uart_dev, dev_name);
	ret = pthread_create(&tid, NULL, uart_comm_task, uart_dev);
	if(ret != 0)
	{
		return -1;
	}

	return 0;
}

/* stop uart commnuicate - ??????uart??????????????????*/
void uart_comm_stop(void)
{
    struct uartCommInfo *client = &uart_info;
	client->state = STATE_CLOSE;
	printf("stop uart.\n");
}
