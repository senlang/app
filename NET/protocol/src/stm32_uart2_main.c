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
	*	˵���� ���uartЭ�飬���С�����������������Ƭ����Ϣͬʱ��Android�巢����Ϣ
	*
	*	�޸ļ�¼��	
	************************************************************
	************************************************************
	************************************************************
**/


#include "stm32f10x.h"	//��Ƭ��ͷ�ļ�

#include "data_io.h"
#include "stm32_uart1.h"
#include "stm32_uart2.h"
#include "stm32_protocol.h"
#include "usart.h"

//Ӳ������
#include "delay.h"
#include "usart.h"

//C��
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "ucos_ii.h"


int down_data_parse(void)
{
	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
	//parse_bd_rx_info();
	return 0;
	
}

  
extern uint8_t  g_src_board_id;
int uart2_receive_data(void)
{
	int retval = -1;
	
	
	//UsartPrintf(USART_DEBUG, "UART2 Data[%d]=0x%02x\r\n", 
	//				uasrt2_recv_data[uart2_enqueue_idx].dataLen, 
	//				uasrt2_recv_data[uart2_dequeue_idx].buf[uasrt2_recv_data[uart2_enqueue_idx].dataLen]);

	if(UART2_IO_Receive() == 0)
	return retval;

	uart2_enqueue_idx++;
	if(uart2_enqueue_idx >= UART_MAX_IDX)
	uart2_enqueue_idx = 0;
	
	uart2_parse_protocol();
	
	UART2_IO_ClearRecive();
	
	return 0;
}
int down_shared_buf_copy(unsigned char *src, int len)
{  	
	return 0;
}  







int uart2_parse_protocol(void)
{
	unsigned char *src;
	int len;
	int count = 0;

	UsartPrintf(USART_DEBUG, "uart2:en idx = %d, de idx = %d\r\n", uart2_enqueue_idx, uart2_dequeue_idx);

	if (uart2_enqueue_idx != uart2_dequeue_idx)
	{
		// 1. ������д��
		// 2. д���ڶ���
		count = (uart2_dequeue_idx > uart2_enqueue_idx) ?
			(UART_MAX_IDX - uart2_dequeue_idx + uart2_enqueue_idx) // ������д
			: (uart2_enqueue_idx - uart2_dequeue_idx ); // д���ڶ�
	}
	else
	{
		//count = UART_MAX_IDX; 
		return 0;
	}
	
	UsartPrintf(USART_DEBUG, "count = %d\r\n", count);

	while(count > 0)
	{
		//UsartPrintf(USART_DEBUG, "uart2:en idx = %d, de idx = %d\r\n", uart2_enqueue_idx, uart2_dequeue_idx);
		src = uasrt2_recv_data[uart2_dequeue_idx].buf;
		len = uasrt2_recv_data[uart2_dequeue_idx].dataLen;
		
		UART1_IO_Send(src, len);
		up_packet_parser(src, len);
		
		uart2_dequeue_idx++;
		if(uart2_dequeue_idx >= UART_MAX_IDX - 1)
		uart2_dequeue_idx = 0;
		count--;
	};	
	return 0;
}














