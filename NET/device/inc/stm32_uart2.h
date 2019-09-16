#ifndef _STM32_UART2_H_
#define _STM32_UART2_H_

#include "stm32f10x.h"
#include <stdio.h>
#include "sys.h"	 								  

//模式控制
#define RS485_TX_EN		PBout(1)	//485模式控制.0,接收;1,发送.
//如果想串口中断接收，请不要注释以下宏定义
#define EN_USART2_RX 	1			//0,不接收;1,接收.

void RS485_Init(u32 bound);
void RS485_Send_Data(u8 *buf,u8 len);

#endif
