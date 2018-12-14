#ifndef _DATA_IO_H_
#define _DATA_IO_H_

#define UART_MAX_IDX 100
#define UART_BUF_MAX_LEN 0x10

#define REV_OK		0	//������ɱ�־
#define REV_WAIT	1	//����δ��ɱ�־

#define DATA_IO_UP		USART1
#define DATA_IO_DOWM	USART2

typedef struct
{
	
	unsigned short dataLen;			//�������ݳ���
	unsigned short dataLenPre;		//��һ�εĳ������ݣ����ڱȽ�
	
	unsigned short dataLenRead;		//�Ѷ�ȡ����λ��
	
	unsigned char buf[1024];			//���ջ���

} DATA_IO_INFO;


typedef struct
{
	
	unsigned short dataLen;			//�������ݳ���
	unsigned short dataLenPre;		//��һ�εĳ������ݣ����ڱȽ�
	
	unsigned short dataLenRead;		//�Ѷ�ȡ����λ��
	
	unsigned char buf[256];			//���ջ���

} UART_DATA_INFO;

typedef struct
{
	unsigned char dataLen;			//�������ݳ���
	unsigned char dataLenPre;		//��һ�εĳ������ݣ����ڱȽ�
	unsigned char status;			//buffer ״̬��0��ʾ�����ݻ�������δ�����꣬1��ʾ�����ѽ�����
	unsigned char buf[UART_BUF_MAX_LEN];			//���ջ���
} UART_DATA;



extern DATA_IO_INFO up_recv_data_info;

extern UART_DATA_INFO down_recv_data_info;

extern UART_DATA uasrt2_recv_data[UART_MAX_IDX];

extern int uart2_enqueue_idx;
extern int uart2_dequeue_idx;

#endif
