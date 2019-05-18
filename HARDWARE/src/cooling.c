/**
	************************************************************
	************************************************************
	************************************************************
	*	�ļ����� 	cooling_compressor.c
	*
	*	���ߣ� 		
	*
	*	���ڣ� 		
	*
	*	�汾�� 		V1.0
	*
	*	˵���� 		�����ʼ��������
	*
	*	�޸ļ�¼��	
	************************************************************
	************************************************************
	************************************************************
**/

//��Ƭ��ͷ�ļ�
#include "stm32f10x.h"

//Ӳ������
#include "cooling.h"


/*
************************************************************
*	�������ƣ�	cooling_Init
*
*	�������ܣ�	�����豸��ʼ��
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		
************************************************************
*/
void Cooling_Init(void)
{

	GPIO_InitTypeDef gpioInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//��GPIOA��ʱ��
	
	gpioInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;				//����Ϊ���
	gpioInitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;						//����ʼ����Pin��
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;				//�ɳ��ص����Ƶ��
	
	GPIO_Init(GPIOA, &gpioInitStruct);							//��ʼ��GPIO

	
	Coolingcompressor_Set(COOLING_OFF);
	Coolingfan_Set(COOLING_OFF);
}

/*
************************************************************
*	�������ƣ�	coolingcompressor_Set
*
*	�������ܣ�	ѹ��������
*
*	��ڲ�����	status������ѹ����
*
*	���ز�����	��
*
*	˵����		��-		��-
************************************************************
*/
void Coolingcompressor_Set(_Bool status)
{
	
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, status == COOLING_ON ? Bit_SET : Bit_RESET);		//���status����BEEP_ON���򷵻�Bit_SET�����򷵻�Bit_RESET
}

void Coolingfan_Set(_Bool status)
{
	
	GPIO_WriteBit(GPIOA, GPIO_Pin_5, status == COOLING_ON ? Bit_SET : Bit_RESET);		//���status����BEEP_ON���򷵻�Bit_SET�����򷵻�Bit_RESET
}



