/**
	************************************************************
	************************************************************
	************************************************************
	*	�ļ����� 	led.c
	*
	*	���ߣ� 		
	*
	*	���ڣ� 		2016-11-23
	*
	*	�汾�� 		V1.0
	*
	*	˵���� 		LED��ʼ��������LED
	*
	*	�޸ļ�¼��	
	************************************************************
	************************************************************
	************************************************************
**/

//��Ƭ��ͷ�ļ�
#include "stm32f10x.h"

//LEDͷ�ļ�
#include "led.h"




LED_STATUS ledStatus;


/*
************************************************************
*	�������ƣ�	Led_Init
*
*	�������ܣ�	LED��ʼ��
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		LED4-PB6	LED5-PB7	LED6-PB8	LED7-PB9
				�ߵ�ƽ�ص�		�͵�ƽ����
************************************************************
*/
void Led_Init(void)
{
	GPIO_InitTypeDef gpioInitStrcut;

	//ʹ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD, ENABLE);
	
	//IO����
	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_3;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	//IO��ʼ��
	GPIO_Init(GPIOC, &gpioInitStrcut);


	gpioInitStrcut.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOD, &gpioInitStrcut);

	
    
    Led_Set(LED_1, LED_OFF);
    Led_Set(LED_2, LED_OFF);
}

/*
************************************************************
*	�������ƣ�	Led2_Set
*
*	�������ܣ�	LED4����
*
*	��ڲ�����	status��LED_ON-����	LED_OFF-�ص�
*
*	���ز�����	��
*
*	˵����		
************************************************************
*/
void Led_Set(LED_SELECT_ENUM gpio, LED_STATUS_ENUM status)
{
	if(gpio == LED_1)
	{
		GPIO_WriteBit(GPIOC, GPIO_Pin_3, status != LED_ON ? Bit_SET : Bit_RESET);
		ledStatus.Led1Sta = status;
	}
	else
	{
		GPIO_WriteBit(GPIOD, GPIO_Pin_12, status != LED_ON ? Bit_SET : Bit_RESET);
		ledStatus.Led2Sta = status;
	}
}
