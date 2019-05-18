

//单片机头文件
#include "stm32f10x.h"

//硬件驱动
#include "light.h"


/*
************************************************************
*	函数名称：	Light_Init
*
*	函数功能：	灯箱控制，2号板PA7
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		
************************************************************
*/
void Light_Init(void)
{

	GPIO_InitTypeDef gpioInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//打开GPIOA的时钟
	
	gpioInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;				//设置为输出
	gpioInitStruct.GPIO_Pin = GPIO_Pin_7;						//将初始化的Pin脚
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;				//可承载的最大频率
	
	GPIO_Init(GPIOA, &gpioInitStruct);							//初始化GPIO

	
	Light_Set(LIGHT_OFF);
}

/*
************************************************************
*	函数名称：	Light_Set
*
*	函数功能：	压缩机控制
*
*	入口参数：	status：灯箱状态
*
*	返回参数：	无
*
*	说明：		开-		关-
************************************************************
*/
void Light_Set(_Bool status)
{
	GPIO_WriteBit(GPIOA, GPIO_Pin_7, status == LIGHT_ON ? Bit_SET : Bit_RESET);		//如果status等于BEEP_ON，则返回Bit_SET，否则返回Bit_RESET
}


