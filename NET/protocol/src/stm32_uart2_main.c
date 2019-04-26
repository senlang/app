/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	
	*
	*	作者： 		
	*
	*	日期： 		
	*
	*	版本： 		
	*
	*	说明： 相对uart协议，上行。即接收来自其他单片机消息同时向Android板发送消息
	*
	*	修改记录：	
	************************************************************
	************************************************************
	************************************************************
**/


#include "stm32f10x.h"	//单片机头文件

#include "data_io.h"
#include "stm32_uart1.h"
#include "stm32_uart2.h"
#include "stm32_protocol.h"
#include "usart.h"

//硬件驱动
#include "delay.h"
#include "usart.h"

//C库
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

  
int down_shared_buf_copy(unsigned char *src, int len)
{  	
	return 0;
}  

extern uint8_t  g_src_board_id;
int uart2_receive_data(void)
{
	int retval = -1;
	int i = 0;

	if(UART2_IO_Receive() == 0)
	return retval;

	UsartPrintf(USART_DEBUG, "uart2 receive[%d]", uasrt2_recv_data[uart2_enqueue_idx].dataLen);
	for(i = 0; i < uasrt2_recv_data[uart2_enqueue_idx].dataLen; i++)
	{
		UsartPrintf(USART_DEBUG, "0x%02x,", uasrt2_recv_data[uart2_enqueue_idx].buf[i]);
	}
	UsartPrintf(USART_DEBUG, "\r\n");
	UART1_IO_Send(uasrt2_recv_data[uart2_enqueue_idx].buf, uasrt2_recv_data[uart2_enqueue_idx].dataLen);
	
	if(g_src_board_id == 1)
	up_packet_parser(uasrt2_recv_data[uart2_enqueue_idx].buf, uasrt2_recv_data[uart2_enqueue_idx].dataLen);
	
	UART2_IO_ClearRecive();
	return 0;
}

