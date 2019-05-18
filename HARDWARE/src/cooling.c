/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	cooling_compressor.c
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

//硬件驱动
#include "cooling.h"


/*
************************************************************
*	函数名称：	cooling_Init
*
*	函数功能：	制冷设备初始化
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		
************************************************************
*/
void Cooling_Init(void)
{

	GPIO_InitTypeDef gpioInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//打开GPIOA的时钟
	
	gpioInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;				//设置为输出
	gpioInitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;						//将初始化的Pin脚
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;				//可承载的最大频率
	
	GPIO_Init(GPIOA, &gpioInitStruct);							//初始化GPIO

	
	Coolingcompressor_Set(COOLING_OFF);
	Coolingfan_Set(COOLING_OFF);
}

/*
************************************************************
*	函数名称：	coolingcompressor_Set
*
*	函数功能：	压缩机控制
*
*	入口参数：	status：开关压缩机
*
*	返回参数：	无
*
*	说明：		开-		关-
************************************************************
*/
void Coolingcompressor_Set(_Bool status)
{
	
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, status == COOLING_ON ? Bit_SET : Bit_RESET);		//如果status等于BEEP_ON，则返回Bit_SET，否则返回Bit_RESET
}

void Coolingfan_Set(_Bool status)
{
	
	GPIO_WriteBit(GPIOA, GPIO_Pin_5, status == COOLING_ON ? Bit_SET : Bit_RESET);		//如果status等于BEEP_ON，则返回Bit_SET，否则返回Bit_RESET
}



