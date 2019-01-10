/**
	************************************************************
	************************************************************
	************************************************************
	*	�ļ����� 	uasrt1_up.c
	*
	*	���ߣ� 		
	*
	*	���ڣ� 		2016-11-23
	*
	*	�汾�� 		V1.0
	*
	*	˵���� 		�����豸����IO��
	*
	*	�޸ļ�¼��	
	************************************************************
	************************************************************
	************************************************************
**/

#include "stm32f10x.h"  //��Ƭ��ͷ�ļ�

#include "data_io.h"		//�����豸����IO
#include "stm32_uart1.h"

#include "delay.h"		//Ӳ������


#include "usart.h"


#include <stdarg.h>	//C��
#include <string.h>


#include "ucos_ii.h"


DATA_IO_INFO up_recv_data_info;
UART_DATA uart1_recv_data[UART_MAX_IDX];

int uart1_enqueue_idx = 0;
int uart1_dequeue_idx = 0;

extern OS_EVENT *SemOfUart1RecvData;          //


void UART1_IO_ClearRecive(void);
 /**
  * @brief  ����Ƕ�������жϿ�����NVIC
  * @param  ��
  * @retval ��
  */
static void Up_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Ƕ�������жϿ�������ѡ�� */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* ����USARTΪ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = UP_USART_IRQ;
  /* �������ȼ�*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  /* �����ȼ� */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  /* ʹ���ж� */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* ��ʼ������NVIC */
  NVIC_Init(&NVIC_InitStructure);
}



 
  /**
   * @brief  USART GPIO ����,������������
   * @param  ��
   * @retval ��
   */
 void Up_USART_Config(void)
 {
	 GPIO_InitTypeDef GPIO_InitStructure;
	 USART_InitTypeDef USART_InitStructure;
 
	 // �򿪴���GPIO��ʱ��
	 UP_USART_GPIO_APBxClkCmd(UP_USART_GPIO_CLK, ENABLE);
	 
	 // �򿪴��������ʱ��
	 UP_USART_APBxClkCmd(UP_USART_CLK, ENABLE);
 
	 // ��USART Tx��GPIO����Ϊ���츴��ģʽ
	 GPIO_InitStructure.GPIO_Pin = UP_USART_TX_GPIO_PIN;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(UP_USART_TX_GPIO_PORT, &GPIO_InitStructure);
 
   // ��USART Rx��GPIO����Ϊ��������ģʽ
	 GPIO_InitStructure.GPIO_Pin = UP_USART_RX_GPIO_PIN;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	 GPIO_Init(UP_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	 
	 // ���ô��ڵĹ�������
	 // ���ò�����
	 USART_InitStructure.USART_BaudRate = UP_USART_BAUDRATE;
	 // ���� �������ֳ�
	 USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	 // ����ֹͣλ
	 USART_InitStructure.USART_StopBits = USART_StopBits_1;
	 // ����У��λ
	 USART_InitStructure.USART_Parity = USART_Parity_No ;
	 // ����Ӳ��������
	 USART_InitStructure.USART_HardwareFlowControl = 
	 USART_HardwareFlowControl_None;
	 // ���ù���ģʽ���շ�һ��
	 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	 // ��ɴ��ڵĳ�ʼ������
	 USART_Init(UP_USARTx, &USART_InitStructure);
	 
	 // �����ж����ȼ�����
	 Up_NVIC_Configuration();
	 
	 // ʹ�ܴ��ڽ����ж�
	 USART_ITConfig(UP_USARTx, USART_IT_RXNE, ENABLE);	 
	 
	 // ʹ�ܴ���
	 USART_Cmd(UP_USARTx, ENABLE);		 
 }








/*
************************************************************
*	�������ƣ�	Usart1_Init
*
*	�������ܣ�	����1��ʼ��
*
*	��ڲ�����	baud���趨�Ĳ�����
*
*	���ز�����	��
*
*	˵����		TX-PA9		RX-PA10
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
	usartInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		//��Ӳ������
	usartInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;						//���պͷ���
	usartInitStruct.USART_Parity = USART_Parity_No;									//��У��
	usartInitStruct.USART_StopBits = USART_StopBits_1;								//1λֹͣλ
	usartInitStruct.USART_WordLength = USART_WordLength_8b;							//8λ����λ
	USART_Init(USART1, &usartInitStruct);
	
	USART_Cmd(USART1, ENABLE);														//ʹ�ܴ���
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);									//ʹ�ܽ����ж�
	
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
*	�������ƣ�	NET_IO_Send
*
*	�������ܣ�	��������
*
*	��ڲ�����	str����Ҫ���͵�����
*				len�����ݳ���
*
*	���ز�����	��
*
*	˵����		�ײ�����ݷ�������
*
************************************************************
*/
void UART1_IO_Send(unsigned char *str, unsigned short len)
{

	unsigned short count = 0;
	
	for(; count < len; count++)											//����һ֡����
	{
		USART_SendData(USART1, *str++);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	}

}

/*
************************************************************
*	�������ƣ�	NET_IO_WaitRecive
*
*	�������ܣ�	�ȴ��������
*
*	��ڲ�����	��
*
*	���ز�����	REV_OK-�������		REV_WAIT-���ճ�ʱδ���
*
*	˵����		ѭ�����ü���Ƿ�������
************************************************************
*/
_Bool UART1_IO_WaitRecive(void)
{

	if(uart1_recv_data[uart1_enqueue_idx].dataLen == 0)						//������ռ���Ϊ0 ��˵��û�д��ڽ��������У�����ֱ����������������
		return REV_WAIT;
		
	if(uart1_recv_data[uart1_enqueue_idx].dataLen == uart1_recv_data[uart1_enqueue_idx].dataLenPre)	//�����һ�ε�ֵ�������ͬ����˵���������
	{
		//down_recv_data_info.dataLen = 0;						//��0���ռ���
		return REV_OK;								//���ؽ�����ɱ�־
	}
		
	uart1_recv_data[uart1_enqueue_idx].dataLenPre = uart1_recv_data[uart1_enqueue_idx].dataLen;		//��Ϊ��ͬ
	return REV_WAIT;								//���ؽ���δ��ɱ�־

}


/*
************************************************************
*	�������ƣ�	NET_IO_ClearRecive
*
*	�������ܣ�	��ջ���
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		
************************************************************
*/
void UART1_IO_ClearRecive(void)
{	
	memset(&uart1_recv_data[uart1_enqueue_idx], 0, sizeof(UART_DATA));
}


/*
************************************************************
*	�������ƣ�	USART2_IRQHandler
*
*	�������ܣ�	�����ж�
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		
************************************************************
*/
void USART1_IRQHandler(void)
{
	uint8_t len;
	RTOS_EnterInt();

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //�����ж�
	{		
		len = uart1_recv_data[uart1_enqueue_idx].dataLen;
		uart1_recv_data[uart1_enqueue_idx].buf[len] = USART1->DR; //USART_ReceiveData(USART2);//(USART2->DR);	//��ȡ���յ�������
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





