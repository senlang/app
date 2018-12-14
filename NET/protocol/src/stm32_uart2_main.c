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

  
int down_shared_buf_copy(unsigned char *src, int len)
{  
	#if 0
	do
	{
		UsartPrintf(USART_DEBUG, "recevied msg header[%s]\r\n", src); 
		
		memcpy(bd_shared_rx_buf, src + chk_offset, len);
		if ((bd_shared_rx_buf[1] == 'D') && (bd_shared_rx_buf[2] == 'W')) //�յ���λ��Ϣ$DWXX  
		{  
			bd_buf_bitmap |= DWXX_BUF;  
			memcpy(dwxx_buf, bd_shared_rx_buf, sizeof(dwxx_buf));  
		}  
		else if ((bd_shared_rx_buf[1] == 'T') && (bd_shared_rx_buf[2] == 'X')) //�յ�ͨ����Ϣ$TXXX  
		{  
			bd_buf_bitmap |= TXXX_BUF;  
			memcpy(txxx_buf, bd_shared_rx_buf, sizeof(txxx_buf));  
		}  
		else if ((bd_shared_rx_buf[1] == 'I') && (bd_shared_rx_buf[2] == 'C')) //�յ�IC��Ϣ$ICXX  
		{  
			bd_buf_bitmap |= ICXX_BUF;  
			memcpy(icxx_buf, bd_shared_rx_buf, sizeof(icxx_buf));  
		}  
		else if ((bd_shared_rx_buf[1] == 'Z') && (bd_shared_rx_buf[2] == 'J')) //�յ��Լ���Ϣ$ZJXX  
		{  
			bd_buf_bitmap |= ZJXX_BUF;  
			memcpy(zjxx_buf, bd_shared_rx_buf, sizeof(zjxx_buf));  
		}  
		else if ((bd_shared_rx_buf[1] == 'S') && (bd_shared_rx_buf[2] == 'J')) //�յ�ʱ����Ϣ$SJXX  
		{  
			bd_buf_bitmap |= SJXX_BUF;  
			memcpy(sjxx_buf, bd_shared_rx_buf, sizeof(sjxx_buf));  
		}  
		else if ((bd_shared_rx_buf[1] == 'B') && (bd_shared_rx_buf[2] == 'B')) //�յ��汾��Ϣ$BBXX  
		{  
			bd_buf_bitmap |= BBXX_BUF;  
			memcpy(bbxx_buf, bd_shared_rx_buf, sizeof(bbxx_buf));  
		}  
		else if ((bd_shared_rx_buf[1] == 'F') && (bd_shared_rx_buf[2] == 'K')) //�յ�������Ϣ$FKXX  
		{  
			bd_buf_bitmap |= FKXX_BUF;  
			memcpy(fkxx_buf, bd_shared_rx_buf, sizeof(fkxx_buf));  
		}  
		else if ((bd_shared_rx_buf[1] == 'B') && (bd_shared_rx_buf[2] == 'D') && (bd_shared_rx_buf[3] == 'T')) //�յ�BDTXR
		{  
			bd_buf_bitmap |= BDTXR_BUF;  
			memcpy(bdtxr_buf, bd_shared_rx_buf, sizeof(bd_shared_rx_buf));  
		}  
		
		else
		{
			UsartPrintf(USART_DEBUG, "Received Data is Error, drop!!\r\n");
			break;
		}
		
		
		chk_offset = bd_shared_rx_buf[5]<<8|bd_shared_rx_buf[6];
		UsartPrintf(USART_DEBUG, "chk_offset = %d, len = %d!!\r\n", chk_offset, len);
		
	}while(chk_offset >0 && chk_offset < len);
	#endif
	
	return 0;
}  

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
		
	UART2_IO_ClearRecive();
	return 0;
}

