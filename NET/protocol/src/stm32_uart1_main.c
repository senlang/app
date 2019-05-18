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

extern OS_EVENT *SemOfDataParse;          
extern UART_DATA uart1_recv_data[UART_MAX_IDX];
extern int uart1_enqueue_idx;
extern int uart1_dequeue_idx;

int uart1_receive_data(void)
{
	int retval = -1;
	int i = 0;
	
	//UsartPrintf(USART_DEBUG, " enq idx[%d] len[%d]", uart1_enqueue_idx, uart1_recv_data[uart1_enqueue_idx].dataLen);
	if(UART1_IO_Receive() == 0)
	return retval;

	UsartPrintf(USART_DEBUG, "uart1 enqueue idx[%d] len[%d]", uart1_enqueue_idx, uart1_recv_data[uart1_enqueue_idx].dataLen);
	for(i = 0; i < uart1_recv_data[uart1_enqueue_idx].dataLen; i++)
	{
		UsartPrintf(USART_DEBUG, "0x%02x,", uart1_recv_data[uart1_enqueue_idx].buf[i]);
	}
	UsartPrintf(USART_DEBUG, "\r\n");

	uart1_enqueue_idx++;
	if(uart1_enqueue_idx >= UART_MAX_IDX)
	uart1_enqueue_idx = 0;

	UART1_IO_ClearRecive();

	OSSemPost(SemOfDataParse);
	return 0;
}

int parse_protocol(void)
{
	unsigned char *src;
	int len;
	int count = 0;
	//unsigned char position = 0;

	//UsartPrintf(USART_DEBUG, "00:en idx = %d, de idx = %d\r\n", uart1_enqueue_idx, uart1_dequeue_idx);

	if (uart1_enqueue_idx != uart1_dequeue_idx)
	{
		// 1. ������д��
		// 2. д���ڶ���
		count = (uart1_dequeue_idx > uart1_enqueue_idx) ?
			(UART_MAX_IDX - uart1_dequeue_idx + uart1_enqueue_idx) // ������д
			: (uart1_enqueue_idx - uart1_dequeue_idx ); // д���ڶ�
	}
	else
	{
		//count = UART_MAX_IDX; 
		return 0;
	}
	
	//UsartPrintf(USART_DEBUG, "count = %d\r\n", count);

	while(count > 0)
	{
		//UsartPrintf(USART_DEBUG, "11:en idx = %d, de idx = %d\r\n", uart1_enqueue_idx, uart1_dequeue_idx);
		src = uart1_recv_data[uart1_dequeue_idx].buf;
		len = uart1_recv_data[uart1_dequeue_idx].dataLen;
		packet_parser(src, len);
		
		uart1_dequeue_idx++;
		if(uart1_dequeue_idx >= UART_MAX_IDX - 1)
		uart1_dequeue_idx = 0;
		count--;
	};	
	return 0;
}

