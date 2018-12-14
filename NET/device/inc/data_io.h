#ifndef _DATA_IO_H_
#define _DATA_IO_H_

#define UART_MAX_IDX 100
#define UART_BUF_MAX_LEN 0x10

#define REV_OK		0	//接收完成标志
#define REV_WAIT	1	//接收未完成标志

#define DATA_IO_UP		USART1
#define DATA_IO_DOWM	USART2

typedef struct
{
	
	unsigned short dataLen;			//接收数据长度
	unsigned short dataLenPre;		//上一次的长度数据，用于比较
	
	unsigned short dataLenRead;		//已读取数据位置
	
	unsigned char buf[1024];			//接收缓存

} DATA_IO_INFO;


typedef struct
{
	
	unsigned short dataLen;			//接收数据长度
	unsigned short dataLenPre;		//上一次的长度数据，用于比较
	
	unsigned short dataLenRead;		//已读取数据位置
	
	unsigned char buf[256];			//接收缓存

} UART_DATA_INFO;

typedef struct
{
	unsigned char dataLen;			//接收数据长度
	unsigned char dataLenPre;		//上一次的长度数据，用于比较
	unsigned char status;			//buffer 状态，0表示无数据或者数据未接收完，1表示数据已结束完
	unsigned char buf[UART_BUF_MAX_LEN];			//接收缓存
} UART_DATA;



extern DATA_IO_INFO up_recv_data_info;

extern UART_DATA_INFO down_recv_data_info;

extern UART_DATA uasrt2_recv_data[UART_MAX_IDX];

extern int uart2_enqueue_idx;
extern int uart2_dequeue_idx;

#endif
