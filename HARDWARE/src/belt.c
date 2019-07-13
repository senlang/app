/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	push_belt.c
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
//通讯协议
#include "stm32_protocol.h"
//硬件驱动
#include "belt.h"
#include "motor.h"
#include "usart.h"
#include "delay.h"

extern uint16_t drag_push_time[BOARD_ID_MAX];  
extern uint16_t drag_push_time_calc_pre;
extern uint16_t drag_push_time_calc;

extern MOTOR_STATUS MotorStatus;


/*
************************************************************
*	函数名称：	Belt_Init
*
*	函数功能：	传送带电机初始化
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		
************************************************************
*/

void Belt_Init(void)
{
	
	GPIO_InitTypeDef gpioInitStrcut;

	//使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	//IO配置
	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	//IO初始化
	GPIO_Init(GPIOA, &gpioInitStrcut);
	
	Belt_Set(PUSH_BELT, BELT_STOP);
	Belt_Set(COLLECT_BELT, BELT_STOP);

	memset(&drag_push_time[0], 0x00, sizeof(drag_push_time));
}

//针对以前单传送带
void Belt_Set(BELT_ENUM belt,BELT_WORK_ENUM status)
{
	if(BELT_STOP == status)
	{	
		if(PUSH_BELT == belt)
		GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
		else if(COLLECT_BELT == belt)
		GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
	}
	else if(BELT_RUN == status)
	{	
		if(PUSH_BELT == belt)
		GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);
	
		else if(COLLECT_BELT == belt)
		GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
	}
	MotorStatus.ConveyoeSta = status;
}


void Push_Belt_Set(BELT_WORK_ENUM status)
{

	if(BELT_STOP == status)
	{	
		GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
	}
	else if(BELT_RUN == status)
	{	
		GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);
	}
	MotorStatus.ConveyoeSta = status;
}



void Collect_Belt_Set(BELT_WORK_ENUM status)
{

	if(BELT_STOP == status)
	{	
		GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
	}
	else if(BELT_RUN == status)
	{	
		GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
	}
	MotorStatus.ConveyoeSta = status;
}

int Collect_Belt_Run(void)
{
	Belt_Set(COLLECT_BELT, BELT_RUN);
	
	RTOS_TimeDlyHMSM(0, 0, BELT_RUN_TIME, 0);
	
	Belt_Set(COLLECT_BELT, BELT_STOP);

	return 1;
}

uint8_t Push_Belt_Check(void)
{
	if((0 == drag_push_time_calc_pre && 0 == drag_push_time_calc)||(drag_push_time_calc_pre == 0))
	return 0;//不必启动传送带

	if(drag_push_time_calc == drag_push_time_calc_pre)
	return 1;
	
	drag_push_time_calc = drag_push_time_calc_pre;
	return 0;
}



void PushBeltControl(uint8_t status)
{
	Belt_Set(PUSH_BELT, status);
}



