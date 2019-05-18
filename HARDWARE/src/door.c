
//单片机头文件
#include "stm32f10x.h"

//硬件驱动
#include "door.h"




/*
************************************************************
*	函数名称：	Door_Init
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
void Door_Init(void)
{

	GPIO_InitTypeDef gpioInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//打开GPIOA的时钟
	
	gpioInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;				//设置为输出
	gpioInitStruct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;						//将初始化的Pin脚
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;				//可承载的最大频率
	
	GPIO_Init(GPIOA, &gpioInitStruct);							//初始化GPIO
	
	FrontDoor_Set(BOX_DOOR_CLOSE);
	
	BackDoor_Set(BOX_DOOR_CLOSE);
}

/*
************************************************************
*	函数名称：	FrontDoor_Set
*
*	函数功能：	前大门控制
*
*	入口参数：	status：开关压缩机
*
*	返回参数：	无
*
*	说明：		开-		关-
************************************************************
*/

void FrontDoor_Set(_Bool status)
{
	GPIO_WriteBit(GPIOA, GPIO_Pin_7, status == BOX_DOOR_OPEN ? Bit_SET : Bit_RESET);		//如果status等于DOOR_OPEN，则返回Bit_SET，否则返回Bit_RESET
}

void BackDoor_Set(_Bool status)
{
	GPIO_WriteBit(GPIOA, GPIO_Pin_6, status == BOX_DOOR_OPEN ? Bit_SET : Bit_RESET);		//如果status等于DOOR_OPEN，则返回Bit_SET，否则返回Bit_RESET
}









