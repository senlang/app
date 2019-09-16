#ifndef _STM32_UART2_H_
#define _STM32_UART2_H_

#include "stm32f10x.h"
#include <stdio.h>
#include "sys.h"	 								  

//ģʽ����
#define RS485_TX_EN		PBout(1)	//485ģʽ����.0,����;1,����.
//����봮���жϽ��գ��벻Ҫע�����º궨��
#define EN_USART2_RX 	1			//0,������;1,����.

void RS485_Init(u32 bound);
void RS485_Send_Data(u8 *buf,u8 len);

#endif
