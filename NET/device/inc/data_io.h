#ifndef _DATA_IO_H_
#define _DATA_IO_H_


typedef struct
{
	
	unsigned short dataLen;			//接收数据长度
	unsigned short dataLenPre;		//上一次的长度数据，用于比较
	
	unsigned short dataLenRead;		//已读取数据位置
	
	unsigned char buf[256];			//接收缓存

} DATA_IO_INFO;

#define REV_OK		0	//接收完成标志
#define REV_WAIT	1	//接收未完成标志

#define DATA_IO_UP		USART1
#define DATA_IO_DOWM	USART2



extern DATA_IO_INFO down_recv_data_info;
extern DATA_IO_INFO down_send_data_info;


extern DATA_IO_INFO up_recv_data_info;
extern DATA_IO_INFO up_send_data_info;







#endif
