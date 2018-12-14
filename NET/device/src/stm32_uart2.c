/**
	************************************************************
	************************************************************
	************************************************************
	*	�ļ����� 	uasrt_down.c
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
#include "stm32_uart2.h"

#include "delay.h"		//Ӳ������


#include "usart.h"


#include <stdarg.h>	//C��
#include <string.h>

UART_DATA_INFO down_recv_data_info;
UART_DATA uasrt2_recv_data[UART_MAX_IDX];

int uart2_enqueue_idx = 0;
int uart2_dequeue_idx = 0;
uint8_t uart_rece_flag = 1;



void UART2_IO_ClearRecive(void);

/**
   * @brief  ����Ƕ�������жϿ�����NVIC
   * @param  ��
   * @retval ��
   */
 static void Down_NVIC_Configuration(void)
 {
   NVIC_InitTypeDef NVIC_InitStructure;
   
   /* Ƕ�������жϿ�������ѡ�� */
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
   
   /* ����USARTΪ�ж�Դ */
   NVIC_InitStructure.NVIC_IRQChannel = DOWN_USART_IRQ;
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
 void Down_USART_Config(void)
 {
	 GPIO_InitTypeDef GPIO_InitStructure;
	 USART_InitTypeDef USART_InitStructure;
 
	 // �򿪴���GPIO��ʱ��
	 DOWN_USART_GPIO_APBxClkCmd(DOWN_USART_GPIO_CLK, ENABLE);
	 
	 // �򿪴��������ʱ��
	 UP_USART_APBxClkCmd(DOWN_USART_CLK, ENABLE);
 
	 // ��USART Tx��GPIO����Ϊ���츴��ģʽ
	 GPIO_InitStructure.GPIO_Pin = DOWN_USART_TX_GPIO_PIN;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(DOWN_USART_TX_GPIO_PORT, &GPIO_InitStructure);
 
   // ��USART Rx��GPIO����Ϊ��������ģʽ
	 GPIO_InitStructure.GPIO_Pin = DOWN_USART_RX_GPIO_PIN;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	 GPIO_Init(DOWN_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	 
	 // ���ô��ڵĹ�������
	 // ���ò�����
	 USART_InitStructure.USART_BaudRate = DOWN_USART_BAUDRATE;
	 // ���� �������ֳ�
	 USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	 // ����ֹͣλ
	 USART_InitStructure.USART_StopBits = USART_StopBits_1;
	 // ����У��λ
	 USART_InitStructure.USART_Parity = USART_Parity_No ;
	 // ����Ӳ��������
	 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	 // ���ù���ģʽ���շ�һ��
	 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	 // ��ɴ��ڵĳ�ʼ������
	 USART_Init(DOWN_USARTx, &USART_InitStructure);
	 
	 // �����ж����ȼ�����
	 Down_NVIC_Configuration();
	 
	 // ʹ�ܴ��ڽ����ж�
	 USART_ITConfig(DOWN_USARTx, USART_IT_RXNE, ENABLE);	 
	 
	 // ʹ�ܴ���
	 USART_Cmd(DOWN_USARTx, ENABLE);		 
 
   // ���������ɱ�־
	 //USART_ClearFlag(USART1, USART_FLAG_TC);	   
 }
 
 
 
 /*
 ************************************************************
 *	 �������ƣ�  Usart2_Init
 *
 *	 �������ܣ�  ����2��ʼ��
 *
 *	 ��ڲ�����  baud���趨�Ĳ�����
 *
 *	 ���ز�����  ��
 *
 *	 ˵���� 	 TX-PA2 	 RX-PA3
 ************************************************************
 */
 void Usart2_Init(unsigned int baud)
 {
 
	 GPIO_InitTypeDef gpioInitStruct;
	 USART_InitTypeDef usartInitStruct;
	 NVIC_InitTypeDef nvicInitStruct;
	 
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	 
	 //PA2	 TXD
	 gpioInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	 gpioInitStruct.GPIO_Pin = GPIO_Pin_2;
	 gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOA, &gpioInitStruct);
	 
	 //PA3	 RXD
	 gpioInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	 gpioInitStruct.GPIO_Pin = GPIO_Pin_3;
	 gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOA, &gpioInitStruct);
	 
	 usartInitStruct.USART_BaudRate = baud;
	 usartInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	 //��Ӳ������
	 usartInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					 //���պͷ���
	 usartInitStruct.USART_Parity = USART_Parity_No;								 //��У��
	 usartInitStruct.USART_StopBits = USART_StopBits_1; 							 //1λֹͣλ
	 usartInitStruct.USART_WordLength = USART_WordLength_8b;						 //8λ����λ
	 USART_Init(USART2, &usartInitStruct);
	 
	 USART_Cmd(USART2, ENABLE); 													 //ʹ�ܴ���
	 
	 USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 								 //ʹ�ܽ����ж�
	 
	 nvicInitStruct.NVIC_IRQChannel = USART2_IRQn;
	 nvicInitStruct.NVIC_IRQChannelCmd = ENABLE;
	 nvicInitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	 nvicInitStruct.NVIC_IRQChannelSubPriority = 3;
	 NVIC_Init(&nvicInitStruct);

	 
	 USART_GetFlagStatus(USART2, USART_FLAG_TC);
	 
	 UART2_IO_ClearRecive();
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
void UART2_IO_Send(unsigned char *str, unsigned short len)
{

	unsigned short count = 0;
	
	for(; count < len; count++)											//����һ֡����
	{
		USART_SendData(USART2, *str++);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
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
_Bool UART2_IO_WaitRecive(void)
{

	if(uasrt2_recv_data[uart2_enqueue_idx].dataLen == 0) 						//������ռ���Ϊ0 ��˵��û�д��ڽ��������У�����ֱ����������������
		return REV_WAIT;
		
	if(uasrt2_recv_data[uart2_enqueue_idx].dataLen == uasrt2_recv_data[uart2_enqueue_idx].dataLenPre)	//�����һ�ε�ֵ�������ͬ����˵���������
	{
		//down_recv_data_info.dataLen = 0;						//��0���ռ���
		return REV_OK;								//���ؽ�����ɱ�־
	}
		
	uasrt2_recv_data[uart2_enqueue_idx].dataLenPre = uasrt2_recv_data[uart2_enqueue_idx].dataLen;		//��Ϊ��ͬ
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
void UART2_IO_ClearRecive(void)
{
	uart2_enqueue_idx++;
	if(uart2_enqueue_idx >= UART_MAX_IDX)
	uart2_enqueue_idx = 0;
	
	memset(&uasrt2_recv_data[uart2_enqueue_idx], 0, sizeof(UART_DATA));
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
#if 0
void USART2_IRQHandler(void)
{
	
	RTOS_EnterInt();

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //�����ж�
	{
		if(down_recv_data_info.dataLen >= sizeof(down_recv_data_info.buf))	
		down_recv_data_info.dataLen = 0; //��ֹ���ڱ�ˢ��
		
		//UsartPrintf(USART_DEBUG, "irq,0x%02x\r\n", USART_ReceiveData(USART2));
		down_recv_data_info.buf[down_recv_data_info.dataLen] = USART_ReceiveData(USART2);

		down_recv_data_info.dataLen++;
		USART_ClearFlag(USART2, USART_FLAG_RXNE);
	}
	
	RTOS_ExitInt();

}

int UART2_IO_Receive(void)
{
	unsigned short len = 0;
	len = down_recv_data_info.dataLen;	
	if(UART2_IO_WaitRecive() != REV_OK)
	{
		//UsartPrintf(USART_DEBUG, "UART2 No Data!!!!!!!!!!!\r\n");
		return 0;
	}

	return len;
}

#else
void USART2_IRQHandler(void)
{
	uint8_t len;
	
	RTOS_EnterInt();

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //�����ж�
	{
		len = uasrt2_recv_data[uart2_enqueue_idx].dataLen;
		uasrt2_recv_data[uart2_enqueue_idx].buf[len] = USART2->DR;	//��ȡ���յ�������
		if(len < UART_BUF_MAX_LEN - 1)
		{
			len++;
			uasrt2_recv_data[uart2_enqueue_idx].dataLen = len;
		}
		
		USART_ClearFlag(USART2, USART_FLAG_RXNE);
	}
	
	RTOS_ExitInt();

}

int UART2_IO_Receive(void)
{
	if(uart2_enqueue_idx == uart2_dequeue_idx - 1)//��������δ����
	uart_rece_flag = 0;

	if(UART2_IO_WaitRecive() != REV_OK)
	{
		return 0;
	}

	uasrt2_recv_data[uart2_enqueue_idx].status = 1;
	return uasrt2_recv_data[uart2_enqueue_idx].dataLen;
}
#endif


