/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	uasrt1_up.c
	*
	*	作者： 		
	*
	*	日期： 		2016-11-23
	*
	*	版本： 		V1.0
	*
	*	说明： 		网络设备数据IO层
	*
	*	修改记录：	
	************************************************************
	************************************************************
	************************************************************
**/

#include "stm32f10x.h"  //单片机头文件

#include "data_io.h"		//网络设备数据IO
#include "stm32_uart1.h"

#include "delay.h"		//硬件驱动


#include "usart.h"


#include <stdarg.h>	//C库
#include <string.h>


#include "ucos_ii.h"


DATA_IO_INFO up_recv_data_info;
UART_DATA uart1_recv_data[UART_MAX_IDX];

int uart1_enqueue_idx = 0;
int uart1_dequeue_idx = 0;

extern OS_EVENT *SemOfUart1RecvData;          //


void UART1_IO_ClearRecive(void);
 /**
  * @brief  配置嵌套向量中断控制器NVIC
  * @param  无
  * @retval 无
  */
static void Up_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 嵌套向量中断控制器组选择 */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* 配置USART为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = UP_USART_IRQ;
  /* 抢断优先级*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  /* 子优先级 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  /* 使能中断 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* 初始化配置NVIC */
  NVIC_Init(&NVIC_InitStructure);
}



 
  /**
   * @brief  USART GPIO 配置,工作参数配置
   * @param  无
   * @retval 无
   */
 void Up_USART_Config(void)
 {
	 GPIO_InitTypeDef GPIO_InitStructure;
	 USART_InitTypeDef USART_InitStructure;
 
	 // 打开串口GPIO的时钟
	 UP_USART_GPIO_APBxClkCmd(UP_USART_GPIO_CLK, ENABLE);
	 
	 // 打开串口外设的时钟
	 UP_USART_APBxClkCmd(UP_USART_CLK, ENABLE);
 
	 // 将USART Tx的GPIO配置为推挽复用模式
	 GPIO_InitStructure.GPIO_Pin = UP_USART_TX_GPIO_PIN;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(UP_USART_TX_GPIO_PORT, &GPIO_InitStructure);
 
   // 将USART Rx的GPIO配置为浮空输入模式
	 GPIO_InitStructure.GPIO_Pin = UP_USART_RX_GPIO_PIN;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	 GPIO_Init(UP_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	 
	 // 配置串口的工作参数
	 // 配置波特率
	 USART_InitStructure.USART_BaudRate = UP_USART_BAUDRATE;
	 // 配置 针数据字长
	 USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	 // 配置停止位
	 USART_InitStructure.USART_StopBits = USART_StopBits_1;
	 // 配置校验位
	 USART_InitStructure.USART_Parity = USART_Parity_No ;
	 // 配置硬件流控制
	 USART_InitStructure.USART_HardwareFlowControl = 
	 USART_HardwareFlowControl_None;
	 // 配置工作模式，收发一起
	 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	 // 完成串口的初始化配置
	 USART_Init(UP_USARTx, &USART_InitStructure);
	 
	 // 串口中断优先级配置
	 Up_NVIC_Configuration();
	 
	 // 使能串口接收中断
	 USART_ITConfig(UP_USARTx, USART_IT_RXNE, ENABLE);	 
	 
	 // 使能串口
	 USART_Cmd(UP_USARTx, ENABLE);		 
 }








/*
************************************************************
*	函数名称：	Usart1_Init
*
*	函数功能：	串口1初始化
*
*	入口参数：	baud：设定的波特率
*
*	返回参数：	无
*
*	说明：		TX-PA9		RX-PA10
************************************************************
*/
void Usart1_Init(unsigned int baud)
{

	GPIO_InitTypeDef gpioInitStruct;
	USART_InitTypeDef usartInitStruct;
	NVIC_InitTypeDef nvicInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	//PA9	TXD
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioInitStruct.GPIO_Pin = GPIO_Pin_9;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpioInitStruct);
	
	//PA10	RXD
	gpioInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpioInitStruct.GPIO_Pin = GPIO_Pin_10;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpioInitStruct);
	
	usartInitStruct.USART_BaudRate = baud;
	usartInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		//无硬件流控
	usartInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;						//接收和发送
	usartInitStruct.USART_Parity = USART_Parity_No;									//无校验
	usartInitStruct.USART_StopBits = USART_StopBits_1;								//1位停止位
	usartInitStruct.USART_WordLength = USART_WordLength_8b;							//8位数据位
	USART_Init(USART1, &usartInitStruct);
	
	USART_Cmd(USART1, ENABLE);														//使能串口
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);									//使能接收中断
	
	nvicInitStruct.NVIC_IRQChannel = USART1_IRQn;
	nvicInitStruct.NVIC_IRQChannelCmd = ENABLE;
	nvicInitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	nvicInitStruct.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&nvicInitStruct);
	
	USART_GetFlagStatus(USART1, USART_FLAG_TC);

	memset(&uart1_recv_data[uart1_enqueue_idx], 0, sizeof(UART_DATA));

	//UART1_IO_ClearRecive();
}










/*
************************************************************
*	函数名称：	NET_IO_Send
*
*	函数功能：	发送数据
*
*	入口参数：	str：需要发送的数据
*				len：数据长度
*
*	返回参数：	无
*
*	说明：		底层的数据发送驱动
*
************************************************************
*/
void UART1_IO_Send(unsigned char *str, unsigned short len)
{

	unsigned short count = 0;
	
	for(; count < len; count++)											//发送一帧数据
	{
		USART_SendData(USART1, *str++);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	}

}

/*
************************************************************
*	函数名称：	NET_IO_WaitRecive
*
*	函数功能：	等待接收完成
*
*	入口参数：	无
*
*	返回参数：	REV_OK-接收完成		REV_WAIT-接收超时未完成
*
*	说明：		循环调用检测是否接收完成
************************************************************
*/
_Bool UART1_IO_WaitRecive(void)
{

	if(uart1_recv_data[uart1_enqueue_idx].dataLen == 0)						//如果接收计数为0 则说明没有处于接收数据中，所以直接跳出，结束函数
		return REV_WAIT;
		
	if(uart1_recv_data[uart1_enqueue_idx].dataLen == uart1_recv_data[uart1_enqueue_idx].dataLenPre)	//如果上一次的值和这次相同，则说明接收完毕
	{
		//down_recv_data_info.dataLen = 0;						//清0接收计数
		return REV_OK;								//返回接收完成标志
	}
		
	uart1_recv_data[uart1_enqueue_idx].dataLenPre = uart1_recv_data[uart1_enqueue_idx].dataLen;		//置为相同
	return REV_WAIT;								//返回接收未完成标志

}


/*
************************************************************
*	函数名称：	NET_IO_ClearRecive
*
*	函数功能：	清空缓存
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		
************************************************************
*/
void UART1_IO_ClearRecive(void)
{	
	memset(&uart1_recv_data[uart1_enqueue_idx], 0, sizeof(UART_DATA));
}


/*
************************************************************
*	函数名称：	USART2_IRQHandler
*
*	函数功能：	接收中断
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		
************************************************************
*/
void USART1_IRQHandler(void)
{
	uint8_t len;
	RTOS_EnterInt();

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //接收中断
	{		
		len = uart1_recv_data[uart1_enqueue_idx].dataLen;
		uart1_recv_data[uart1_enqueue_idx].buf[len] = USART1->DR; //USART_ReceiveData(USART2);//(USART2->DR);	//读取接收到的数据
		if(len < UART_BUF_MAX_LEN - 1)
		{
			len++;
			uart1_recv_data[uart1_enqueue_idx].dataLen = len;
		}
		
		USART_ClearFlag(USART1, USART_FLAG_RXNE);
		
		OSSemPost(SemOfUart1RecvData);
	}
	RTOS_ExitInt();
}

int UART1_IO_Receive(void)
{
	if(UART1_IO_WaitRecive() != REV_OK)
	{
		return 0;
	}

	uart1_recv_data[uart1_enqueue_idx].status = 1;
	return uart1_recv_data[uart1_enqueue_idx].dataLen;
}





