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

extern OS_EVENT *SemOfUart2RecvData;	//uart2 ���ڽ��������ź���


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
	nvicInitStruct.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&nvicInitStruct);


	USART_GetFlagStatus(USART2, USART_FLAG_TC);
	memset(&uasrt2_recv_data[uart2_enqueue_idx], 0, sizeof(UART_DATA));
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
	
	UsartPrintf(USART_DEBUG, "UART2 Send[%d] : ", len);
	for(; count < len; count++)											//����һ֡����
	{
		UsartPrintf(USART_DEBUG, "0x%02x,", *(str + count));
		USART_SendData(USART2, *(str + count));
		
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	}
	UsartPrintf(USART_DEBUG, "\r\n\r\n");
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
	//UsartPrintf(USART_DEBUG, "WaitReceive:%d,%d\r\n", uasrt2_recv_data[uart2_enqueue_idx].dataLen, uasrt2_recv_data[uart2_enqueue_idx].dataLenPre);

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
		OSSemPost(SemOfUart2RecvData);
	}
	
	RTOS_ExitInt();

}

int UART2_IO_Receive(void)
{

	if(UART2_IO_WaitRecive() != REV_OK)
	{
		return 0;
	}

	uasrt2_recv_data[uart2_enqueue_idx].status = 1;
	return uasrt2_recv_data[uart2_enqueue_idx].dataLen;
}




//��ʼ��IO ����2
//pclk1:PCLK1ʱ��Ƶ��(Mhz)
//bound:������	  
void RS485_Init(u32 bound)
{  
    GPIO_InitTypeDef GPIO_InitStructure;
  	USART_InitTypeDef USART_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE);//ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;				 //PA7�˿�����
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	//PA2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);  

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,ENABLE);//��λ����2
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2,DISABLE);//ֹͣ��λ
 
	
 	#ifdef EN_USART2_RX		  	//���ʹ���˽���
	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ���ݳ���
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;///��żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�ģʽ

    USART_Init(USART2, &USART_InitStructure); ; //��ʼ������
  
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //ʹ�ܴ���2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //�����ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure); //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
 
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�����ж�
   
    USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ��� 

 	#endif

	RS485_TX_EN=0;			//Ĭ��Ϊ����ģʽ
}



//RS485����len���ֽ�.
//buf:�������׵�ַ
//len:���͵��ֽ���
void RS485_Send_Data(u8 *buf,u8 len)
{
	u8 t;
	
	UsartPrintf(USART_DEBUG, "RS485 Send[%d] : ", len);
	for(t=0;t<len;t++)	
	{
		UsartPrintf(USART_DEBUG, "0x%02x,", *(buf + t));
	}
	UsartPrintf(USART_DEBUG, "\r\n\r\n");
		
	RS485_TX_EN=1;			//����Ϊ����ģʽ
  	for(t=0;t<len;t++)		//ѭ����������
	{		   
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	  
		USART_SendData(USART2,buf[t]);
	}	 
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	
	
	RS485_TX_EN=0;				//����Ϊ����ģʽ	
}


