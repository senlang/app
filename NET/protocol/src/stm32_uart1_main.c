/**
	************************************************************
	************************************************************
	************************************************************
	*	�ļ����� 	
	*
	*	���ߣ� 		
	*
	*	���ڣ� 		
	*
	*	�汾�� 		
	*
	*	˵���� ���uartЭ�飬���С�����������Android����Ϣͬʱ��������Ƭ��������Ϣ
	*
	*	�޸ļ�¼��	
	************************************************************
	************************************************************
	************************************************************
**/

#include "stm32f10x.h"	//��Ƭ��ͷ�ļ�

#include "stm32_protocol.h"
#include "data_io.h"
#include "stm32_uart1.h"
#include "stm32_uart2.h"
#include "usart.h"

//Ӳ������
#include "delay.h"
#include "usart.h"

//C��
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "ucos_ii.h"

extern OS_EVENT *SemOfUart1RecvData;          



int up_data_parse(void)
{
	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
	parse_up_rx_info();
	return 0;
	
}


int uart1_receive_data(void)
{
	int retval = -1;
	int i = 0;

	//UsartPrintf(USART_DEBUG, "%s[%d]BDSIOInfo.dataLen = %d\r\n", __FUNCTION__, __LINE__, BDSIOInfo.dataLen);
	
	if(UART1_IO_Receive() == 0)
	return retval;


	UsartPrintf(USART_DEBUG, "uart1 receive[%d]\r\n",up_recv_data_info.dataLen);
	for(i = 0; i < up_recv_data_info.dataLen; i++)
	{
		UsartPrintf(USART_DEBUG, "0x%02x,", up_recv_data_info.buf[i]);
	}
	
	UsartPrintf(USART_DEBUG, "\r\n");
	
	//UART2_IO_Send(up_recv_data_info.buf, up_recv_data_info.dataLen);

#if 0
	uart1_shared_buf_preparse(up_recv_data_info.buf, up_recv_data_info.dataLen);

	
	UART1_IO_ClearRecive();
	
	up_data_parse();
#else
	packet_parser(up_recv_data_info.buf, up_recv_data_info.dataLen);
	UART1_IO_ClearRecive();
#endif		
	return 0;
}


