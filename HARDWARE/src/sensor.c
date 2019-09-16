/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	sensor.c
	*
	*	作者： 		
	*
	*	日期： 		
	*
	*	版本： 		V1.0
	*
	*	说明： 		灯箱初始化、控制
	*
	*	修改记录：	
	************************************************************
	************************************************************
	************************************************************
**/

//单片机头文件
#include "stm32f10x.h"

#include "delay.h"

#include "sensor.h"



/*人体检查PB4*/
void Sensor_Init(void)
{
	GPIO_InitTypeDef gpioInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	gpioInitStructure.GPIO_Mode = GPIO_Mode_IPU;
	gpioInitStructure.GPIO_Pin = GPIO_Pin_4;
	gpioInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioInitStructure);
}

_Bool Sensor_Status(void)
{
	if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4))//未发现接入为低
	{
		return SENSOR_DETECT;
	}
	else									//发现接入为高
	{
		return SENSOR_NO_DETECT;
	}
}


unsigned char Sensor_Detect(void)
{
	unsigned char ret_val = 0;
	
	if(Sensor_Status() == SENSOR_DETECT)
	{
		RTOS_TimeDly(20);					
		if(Sensor_Status() == SENSOR_DETECT)
		ret_val = SENSOR_DETECT;			
	}
	else
	{
		ret_val = SENSOR_NO_DETECT;
	}
	
	//UsartPrintf(USART_DEBUG, "Sensor_Detect = %d\r\n", ret_val);		//提示任务开始执行
	return ret_val;
}


/*?a1???°′?￥?ì2é*/
void Door_Key_Init(void)
{
	GPIO_InitTypeDef gpioInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/*1????ì2a*/
	gpioInitStructure.GPIO_Mode = GPIO_Mode_IPU;
	gpioInitStructure.GPIO_Pin = GPIO_Pin_3;
	gpioInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioInitStructure);

	/*?a???ì2a*/
	gpioInitStructure.GPIO_Mode = GPIO_Mode_IPU;
	gpioInitStructure.GPIO_Pin = GPIO_Pin_7;
	gpioInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &gpioInitStructure);

	
}

_Bool Door_Key_Status(unsigned char door_detect)
{
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	
	if(door_detect == DOOR_OPEN)
	{
		GPIOx = GPIOD;
		GPIO_Pin = GPIO_Pin_7;
	}
	else
	{
		GPIOx = GPIOB;
		GPIO_Pin = GPIO_Pin_3;
	}

	if(!GPIO_ReadInputDataBit(GPIOx, GPIO_Pin))//?′·￠???óè??aμí
	{
		return SENSOR_DETECT;
	}
	else									//·￠???óè??a??
	{
		return SENSOR_NO_DETECT;
	}
}


unsigned char Door_Key_Detect(unsigned char door_detect)
{
	unsigned char ret_val = 0;
	
	if(Door_Key_Status(door_detect) == SENSOR_DETECT)
	{
		RTOS_TimeDly(20);					
		if(Door_Key_Status(door_detect) == SENSOR_DETECT)
		{
			ret_val = SENSOR_DETECT;
		}			
	}
	else
	{
		ret_val = SENSOR_NO_DETECT;
	}
	//UsartPrintf(USART_DEBUG, "%s:%s\r\n", DOOR_OPEN?"DOOR_OPEN":"DOOR_CLOSE", ret_val?"SENSOR_DETECT":"SENSOR_NO_DETECT");		//
	//UsartPrintf(USART_DEBUG, "%s:%d,%d\r\n", __FUNCTION__,  door_detect, ret_val);		//

	return ret_val;
}

