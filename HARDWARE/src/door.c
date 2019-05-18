
//��Ƭ��ͷ�ļ�
#include "stm32f10x.h"

//Ӳ������
#include "door.h"




/*
************************************************************
*	�������ƣ�	Door_Init
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
void Door_Init(void)
{

	GPIO_InitTypeDef gpioInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//��GPIOA��ʱ��
	
	gpioInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;				//����Ϊ���
	gpioInitStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;						//����ʼ����Pin��
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;				//�ɳ��ص����Ƶ��
	
	GPIO_Init(GPIOA, &gpioInitStruct);							//��ʼ��GPIO
	
	FrontDoor_Set(BOX_DOOR_CLOSE);
	
	BackDoor_Set(BOX_DOOR_CLOSE);
}

/*
************************************************************
*	�������ƣ�	FrontDoor_Set
*
*	�������ܣ�	ǰ���ſ���
*
*	��ڲ�����	status������ѹ����
*
*	���ز�����	��
*
*	˵����		��-		��-
************************************************************
*/

void FrontDoor_Set(_Bool status)
{
	GPIO_WriteBit(GPIOA, GPIO_Pin_7, status == BOX_DOOR_OPEN ? Bit_SET : Bit_RESET);		//���status����DOOR_OPEN���򷵻�Bit_SET�����򷵻�Bit_RESET
}

void BackDoor_Set(_Bool status)
{
	GPIO_WriteBit(GPIOA, GPIO_Pin_6, status == BOX_DOOR_OPEN ? Bit_SET : Bit_RESET);		//���status����DOOR_OPEN���򷵻�Bit_SET�����򷵻�Bit_RESET
}









