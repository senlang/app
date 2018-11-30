/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	led.c
	*
	*	作者： 		
	*
	*	日期： 		2016-11-23
	*
	*	版本： 		V1.0
	*
	*	说明： 		LED初始化，亮灭LED
	*
	*	修改记录：	
	************************************************************
	************************************************************
	************************************************************
**/

//单片机头文件
#include "stm32f10x.h"

//LED头文件
#include "led.h"




LED_STATUS ledStatus;


/*
************************************************************
*	函数名称：	Led_Init
*
*	函数功能：	LED初始化
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		LED4-PB6	LED5-PB7	LED6-PB8	LED7-PB9
				高电平关灯		低电平开灯
************************************************************
*/
void Led_Init(void)
{
	GPIO_InitTypeDef gpioInitStrcut;

	//使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD, ENABLE);
	
	//IO配置
	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_3;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	//IO初始化
	GPIO_Init(GPIOC, &gpioInitStrcut);


	gpioInitStrcut.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOD, &gpioInitStrcut);

	
    
    Led_Set(LED_1, LED_OFF);
    Led_Set(LED_2, LED_OFF);
}

/*
************************************************************
*	函数名称：	Led2_Set
*
*	函数功能：	LED4控制
*
*	入口参数：	status：LED_ON-开灯	LED_OFF-关灯
*
*	返回参数：	无
*
*	说明：		
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
