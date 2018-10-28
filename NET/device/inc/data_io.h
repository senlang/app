#ifndef _DATA_IO_H_
#define _DATA_IO_H_


typedef struct
{
	
	unsigned short dataLen;			//�������ݳ���
	unsigned short dataLenPre;		//��һ�εĳ������ݣ����ڱȽ�
	
	unsigned short dataLenRead;		//�Ѷ�ȡ����λ��
	
	unsigned char buf[256];			//���ջ���

} DATA_IO_INFO;

#define REV_OK		0	//������ɱ�־
#define REV_WAIT	1	//����δ��ɱ�־

#define DATA_IO_UP		USART1
#define DATA_IO_DOWM	USART2



extern DATA_IO_INFO down_recv_data_info;
extern DATA_IO_INFO down_send_data_info;


extern DATA_IO_INFO up_recv_data_info;
extern DATA_IO_INFO up_send_data_info;







#endif
