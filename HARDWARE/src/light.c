

//��Ƭ��ͷ�ļ�
#include "stm32f10x.h"

//Ӳ������
#include "light.h"


/*
************************************************************
*	�������ƣ�	Light_Init
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
void Light_Init(void)
{

	GPIO_InitTypeDef gpioInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//��GPIOA��ʱ��
	
	gpioInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;				//����Ϊ���
	gpioInitStruct.GPIO_Pin = GPIO_Pin_6;						//����ʼ����Pin��
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;				//�ɳ��ص����Ƶ��
	
	GPIO_Init(GPIOA, &gpioInitStruct);							//��ʼ��GPIO
}

/*
************************************************************
*	�������ƣ�	Light_Set
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
void Light_Set(_Bool status)
{
	
	GPIO_WriteBit(GPIOA, GPIO_Pin_6, status == LIGHT_ON ? Bit_SET : Bit_RESET);		//���status����BEEP_ON���򷵻�Bit_SET�����򷵻�Bit_RESET
}


