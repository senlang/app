/**
  ******************************************************************************
  * @file    bsp_usart.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   ???c?printf???usart??
  ******************************************************************************
  * @attention
  *
  * ????:??STM32 F103-?? ???  
  * ??    :http://www.firebbs.cn
  * ??    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 
	
#include "usart.h"


#include <stdarg.h>
#include <string.h>
#include "delay.h"		//Ó²¼þÇý¶¯



/*****************  ?????? **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
  /* ?????????USART */
  USART_SendData(pUSARTx,ch);
	  
  /* ??????????? */
  while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);  
}

/****************** ??8???? ************************/
void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num)
{
uint8_t i;
  
  for(i=0; i<num; i++)
{
	  /* ?????????USART */
	  Usart_SendByte(pUSARTx,array[i]);   

}
  /* ?????? */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET);
}

/*****************  ????? **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
  unsigned int k=0;
do 
{
	Usart_SendByte( pUSARTx, *(str + k) );
	k++;
} while(*(str + k)!='\0');

/* ?????? */
while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
{}
}

/*****************  ????16?? **********************/
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
  uint8_t temp_h, temp_l;
  
  /* ????? */
  temp_h = (ch&0XFF00)>>8;
  /* ????? */
  temp_l = ch&0XFF;
  
  /* ????? */
  USART_SendData(pUSARTx,temp_h); 
  while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
  
  /* ????? */
  USART_SendData(pUSARTx,temp_l); 
  while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);  
}

 
  /**
   * @brief  ???????????NVIC
   * @param  ?
   * @retval ?
   */
 static void Debug_NVIC_Configuration(void)
 {
   NVIC_InitTypeDef NVIC_InitStructure;
   
   /* ???????????? */
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
   
   /* ??USART???? */
   NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
   /* ?????*/
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
   /* ???? */
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
   /* ???? */
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   /* ?????NVIC */
   NVIC_Init(&NVIC_InitStructure);
 }


 /**
  * @brief  USART GPIO ??,??????
  * @param  ?
  * @retval ?
  */
void Debug_USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// ????GPIO???
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);
	
	// ?????????
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

	// ?USART Tx?GPIO?????????
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  // ?USART Rx?GPIO?????????
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	
	// ?????????
	// ?????
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	// ?? ?????
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// ?????
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// ?????
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// ???????
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// ??????,????
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// ??????????
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
	
	// ?????????
	Debug_NVIC_Configuration();
	
	// ????????
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);	
	
	// ????
	USART_Cmd(DEBUG_USARTx, ENABLE);	
	
	USART_GetFlagStatus(USART1, USART_FLAG_TC);

  // ????????
	USART_ClearFlag(USART1, USART_FLAG_TC);     
}


void USART3_IRQHandler(void) 
{
	RTOS_EnterInt();

	if(USART_GetITStatus(DEBUG_USARTx,USART_IT_RXNE)!=RESET)
	{		
		USART_ClearFlag(DEBUG_USARTx, USART_FLAG_RXNE);
	}	 
	RTOS_ExitInt();
}


void UsartPrintf(USART_TypeDef *USARTx, char *fmt,...)
{
	unsigned char UsartPrintfBuf[500];
	va_list ap;
	unsigned char *pStr = UsartPrintfBuf;
	
	va_start(ap, fmt);
	vsprintf((char *)UsartPrintfBuf, fmt, ap);							//???
	va_end(ap);
	
	while(*pStr != 0)
	{
		USART_SendData(USARTx, *pStr++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
	}
}

