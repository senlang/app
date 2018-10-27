/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	motor.c
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


//LED头文件
#include "motor.h"
#include "usart.h"
#include "stm32_protocol.h"
#include "delay.h"
#include "key.h"


MOTOR_STATUS MotorStatus;
extern struct motor_control_struct  motor_struct[TOTAL_PUSH_CNT];
extern int motor_enqueue_idx;
extern int motor_dequeue_idx;

extern unsigned char calibrate_track_selected;
extern KEY_STATUS key_status;

/*
************************************************************
*	函数名称：	Motor_Init
*
*	函数功能：	Motor初始化
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		LED4-PB6	LED5-PB7	LED6-PB8	LED7-PB9
				高电平关灯		低电平开灯
************************************************************
*/
void Motor_Init(void)
{
	
	GPIO_InitTypeDef gpioInitStrcut;

	//使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	//IO配置
	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	//IO初始化
	GPIO_Init(GPIOC, &gpioInitStrcut);
    
    Motor_Set(MOTOR_STOP);

	motor_enqueue_idx = 0;
	motor_dequeue_idx = 0;
}


void Conveyor_Init()
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
	
	Conveyor_set(CONVEYOR_STOP);
}




/*
************************************************************
*	函数名称：	Motor_Set
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
void Motor_Set(MOTOR_ENUM status)
{

	if(MOTOR_STOP == status)
	{	
		GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_RESET);
		GPIO_WriteBit(GPIOC, GPIO_Pin_5, Bit_RESET);
	}
	else if(MOTOR_RUN_FORWARD== status)
	{	
		GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_SET);
		GPIO_WriteBit(GPIOC, GPIO_Pin_5, Bit_RESET);
	}
	else if(MOTOR_RUN_BACKWARD== status)
	{	
		GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_RESET);
		GPIO_WriteBit(GPIOC, GPIO_Pin_5, Bit_SET);
	}
	MotorStatus.MotorSta = status;
}


void Conveyor_set(CONVEYOR_ENUM status)
{

	if(CONVEYOR_STOP == status)
	{	
		GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
		GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
	}
	else if(CONVEYOR_RUN== status)
	{	
		GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);
		GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
	}
	MotorStatus.ConveyoeSta = status;
}





void Motor_Start(void)
{
	int push_cnt = 0;
	int i = 0;
	uint16_t last_keep_time = 0;
	uint8_t delay_s = 0;
	uint16_t delay_ms = 0;

	
	UsartPrintf(USART_DEBUG, "Motor_Start,motor_dequeue_idx[%d]-------------\r\n", motor_dequeue_idx);
	push_cnt = (motor_enqueue_idx > motor_dequeue_idx) ? (motor_enqueue_idx - motor_dequeue_idx):(MAX_PUSH_CNT - motor_dequeue_idx + motor_enqueue_idx + 1);

	
	UsartPrintf(USART_DEBUG, "push_cnt[%d]-------------\r\n", push_cnt);
	
	for(i = 0; i < push_cnt; i++)
	{
		UsartPrintf(USART_DEBUG, "motor_struct[%d] board_id = 0x%04x\r\n", motor_dequeue_idx, motor_struct[motor_dequeue_idx].info.board_id);
		UsartPrintf(USART_DEBUG, "motor_struct[%d] medicine_track_number = 0x%04x\r\n", motor_dequeue_idx, motor_struct[motor_dequeue_idx].info.medicine_track_number);
		UsartPrintf(USART_DEBUG, "motor_struct[%d] push_time = 0x%04x\r\n", motor_dequeue_idx, motor_struct[motor_dequeue_idx].info.push_time);

		if(motor_struct[motor_dequeue_idx].motor_run == MOTOR_RUN_FORWARD)
		{
			Motor_Set(MOTOR_RUN_FORWARD);
		}
		else if(motor_struct[motor_dequeue_idx].motor_run == MOTOR_RUN_BACKWARD)
		{
			Motor_Set(MOTOR_RUN_BACKWARD);
		}
		
		Conveyor_set(CONVEYOR_RUN);
		
		set_track(motor_struct[motor_dequeue_idx].info.medicine_track_number, MOTOR_RUN_FORWARD);
		

		delay_s = motor_struct[motor_dequeue_idx].info.push_time/10;
		delay_ms = (motor_struct[motor_dequeue_idx].info.push_time%10) * 100;

		UsartPrintf(USART_DEBUG, "delay_s[%d]delay_ms[%d]\r\n", delay_s, delay_ms);
		last_keep_time = delay_s;
			
		RTOS_TimeDlyHMSM(0, 0, delay_s, delay_ms);
		

		
		set_track(motor_struct[motor_dequeue_idx].info.medicine_track_number, MOTOR_STOP);
		Motor_Set(MOTOR_STOP);


		motor_dequeue_idx++;

		if (motor_dequeue_idx == MAX_PUSH_CNT)
		{
			motor_dequeue_idx = 0;
		}
	}
	
	
	//if(MotorStatus.ConveyoeSta == CONVEYOR_RUN)
	//{
	//	RTOS_TimeDlyHMSM(0, 0, 30, 0);
	//	Conveyor_set(CONVEYOR_STOP);
	//}

}

void track_calibrate()
{
#if 0
	if()
	Motor_Set(MOTOR_RUN_FORWARD);
	set_track(calibrate_track_selected, MOTOR_RUN_FORWARD);
	
	
	
	set_track(push_srtuct[motor_dequeue_idx].medicine_track_number, MOTOR_STOP);
	#endif
}


