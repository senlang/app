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
#include "box.h"

#include "stm32_protocol.h"


#include "stdlib.h"

MOTOR_STATUS MotorStatus;
extern struct motor_control_struct  motor_struct[TOTAL_PUSH_CNT];
extern int motor_enqueue_idx;
extern int motor_dequeue_idx;

extern KEY_STATUS key_status;
extern unsigned char calibrate_enable;
extern struct status_report_request_info_struct  heart_info;


extern uint16_t drag_push_time[BOARD_ID_MAX];  
extern uint16_t drag_push_time_calc_pre;
extern uint16_t drag_push_time_calc;
extern uint8_t motor_run_detect_flag;
extern uint8_t motor_run_detect_track_num;


/*
************************************************************
*	函数名称：	Motor_Init
*
*	函数功能：	货道控制初始化
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		
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




/*
************************************************************
*	函数名称：	Motor_Set
*
*	函数功能：	电机正反转控制
*
*	入口参数：	status：
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
		motor_run_direction = status;
	}
	else if(MOTOR_RUN_FORWARD== status)
	{	
		GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_SET);
		GPIO_WriteBit(GPIOC, GPIO_Pin_5, Bit_RESET);
		motor_run_direction = status;
	}
	else if(MOTOR_RUN_BACKWARD== status)
	{	
		GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_RESET);
		GPIO_WriteBit(GPIOC, GPIO_Pin_5, Bit_SET);
		motor_run_direction = status;
	}
	MotorStatus.MotorSta = status;
}


void Motor_Start(void)
{
	int push_cnt = 0;
	int i = 0;
	//uint16_t last_keep_time = 0;
	uint16_t delay_s = 0;
	uint16_t delay_ms = 0;
	struct push_medicine_complete_request_info_struct  push_complete_info;

	
	UsartPrintf(USART_DEBUG, "Motor_Start,motor_enqueue_idx[%d]motor_dequeue_idx[%d]-------------\r\n", motor_enqueue_idx, motor_dequeue_idx);
	push_cnt = (motor_dequeue_idx > motor_enqueue_idx) ? (motor_enqueue_idx - motor_dequeue_idx + TOTAL_PUSH_CNT):(motor_enqueue_idx - motor_dequeue_idx);

	
	UsartPrintf(USART_DEBUG, "push_cnt[%d]-------------\r\n", push_cnt);
	calibrate_track_selected = 255;
	
	for(i = 0; i < push_cnt; i++)
	{
		UsartPrintf(USART_DEBUG, "motor_struct[%d] board_id = 0x%04x\r\n", motor_dequeue_idx, motor_struct[motor_dequeue_idx].info.board_id);
		UsartPrintf(USART_DEBUG, "motor_struct[%d] medicine_track_number = 0x%04x\r\n", motor_dequeue_idx, motor_struct[motor_dequeue_idx].info.medicine_track_number);
		UsartPrintf(USART_DEBUG, "motor_struct[%d] push_time = 0x%04x\r\n", motor_dequeue_idx, motor_struct[motor_dequeue_idx].info.push_time);


		memset(&push_complete_info, 0x00, sizeof(push_complete_info));
		if(motor_struct[motor_dequeue_idx].motor_run == MOTOR_RUN_FORWARD)
		{
			Motor_Set(MOTOR_RUN_FORWARD);
			heart_info.board_id = motor_struct[motor_dequeue_idx].info.board_id;
			heart_info.board_status = TESTING_STATUS;
			heart_info.medicine_track_number = motor_struct[motor_dequeue_idx].info.medicine_track_number;
		}
		else if(motor_struct[motor_dequeue_idx].motor_run == MOTOR_RUN_BACKWARD)
		{
			Motor_Set(MOTOR_RUN_BACKWARD);
			heart_info.board_id = motor_struct[motor_dequeue_idx].info.board_id;
			heart_info.board_status = TESTING_STATUS;
			heart_info.medicine_track_number = motor_struct[motor_dequeue_idx].info.medicine_track_number;
		}
		
		//Push_Belt_Set(BELT_RUN);

		motor_run_detect_track_num = motor_struct[motor_dequeue_idx].info.medicine_track_number;
		set_track((uint16_t)motor_struct[motor_dequeue_idx].info.medicine_track_number, MOTOR_RUN_FORWARD);

		RTOS_TimeDlyHMSM(0, 0, 0, KEY_DELAY_MS * 100);

		motor_run_detect_flag = 1;

		delay_s = (motor_struct[motor_dequeue_idx].info.push_time - KEY_DELAY_MS)/10;
		delay_ms = ((motor_struct[motor_dequeue_idx].info.push_time - KEY_DELAY_MS)%10) * 100;

		UsartPrintf(USART_DEBUG, "delay_s[%d]delay_ms[%d]\r\n", delay_s, delay_ms);
		//last_keep_time = delay_s;
			
		RTOS_TimeDlyHMSM(0, 0, delay_s, delay_ms);
		

		
		set_track(motor_struct[motor_dequeue_idx].info.medicine_track_number, MOTOR_STOP);
		Motor_Set(MOTOR_STOP);
		motor_run_detect_flag = 0;

		
		heart_info.board_id = motor_struct[motor_dequeue_idx].info.board_id;
		heart_info.board_status = STANDBY_STATUS;
		heart_info.medicine_track_number = 0;

		UsartPrintf(USART_DEBUG, "motor_struct[motor_enqueue_idx].motor_work_mode[0x%02x]\r\n", motor_struct[motor_dequeue_idx].motor_work_mode);
		#if 1
		if(motor_struct[motor_dequeue_idx].motor_work_mode == CMD_PUSH_MEDICINE_REQUEST)
		{
			push_complete_info.board_id = motor_struct[motor_dequeue_idx].info.board_id;
			push_complete_info.medicine_track_number = motor_struct[motor_dequeue_idx].info.medicine_track_number;
			board_send_message(PUSH_MEDICINE_COMPLETE_REQUEST, &push_complete_info);
		}
		#endif
		
		motor_dequeue_idx++;

		if (motor_dequeue_idx == TOTAL_PUSH_CNT)
		{
			motor_dequeue_idx = 0;
		}
	}
	
	
	//if(MotorStatus.ConveyoeSta == BELT_RUN)
	//{
	//	RTOS_TimeDlyHMSM(0, 0, 30, 0);
	//	Push_Belt_Set(BELT_STOP);
	//}

}


#if 0
void track_calibrate(void)
{
	//UsartPrintf(USART_DEBUG, "calibrate_track_selected[%d]calibrate_enable[%d]\r\n", calibrate_track_selected, calibrate_enable);

	if((calibrate_enable == 1) && (calibrate_track_selected == 255))
	{
		Motor_Set(MOTOR_STOP);
		set_track(calibrate_track_selected, MOTOR_STOP);
		calibrate_enable = 0;
		return;
	}

	calibrate_enable = 1;	
	if((key_status.Key0StatChange == KEY_UP2DOWN)&&(key_status.Key1StatChange == KEY_UP2DOWN))
	{
		Motor_Set(MOTOR_STOP);
		set_track(calibrate_track_selected, MOTOR_STOP);
	}
	
	if((key_status.Key0StatChange == KEY_UP2DOWN))
	{
		Motor_Set(MOTOR_RUN_FORWARD);
		set_track(calibrate_track_selected, MOTOR_RUN_FORWARD);
	}

	if((key_status.Key1StatChange == KEY_UP2DOWN))
	{
		Motor_Set(MOTOR_RUN_BACKWARD);
		set_track(calibrate_track_selected, MOTOR_RUN_BACKWARD);
	}
	
	if((key_status.Key0StatChange == KEY_DOWN2UP) 
		|| (key_status.Key1StatChange == KEY_DOWN2UP))
	{
		Motor_Set(MOTOR_STOP);
		set_track(calibrate_track_selected, MOTOR_STOP);
	}

}
#else

void track_calibrate(void)
{

}


#endif



/*取货门电机控制初始化*/
void Door_Control_Init(void)
{
	return;
	/*
	GPIO_InitTypeDef gpioInitStrcut;

	//使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC, ENABLE);
	
	//IO配置
	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_13;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	//IO初始化
	GPIO_Init(GPIOD, &gpioInitStrcut);


	//IO配置
	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_11;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	//IO初始化
	GPIO_Init(GPIOC, &gpioInitStrcut);
	
	Door_Control_Set(MOTOR_STOP);
	*/
}



void Door_Control_Set(MOTOR_ENUM status)
{
	Motor_Set(status);
	set_track(98, status);
	#if 0
	if(MOTOR_STOP == status)
	{	
		GPIO_WriteBit(GPIOD, GPIO_Pin_13, Bit_RESET);
		GPIO_WriteBit(GPIOC, GPIO_Pin_11, Bit_RESET);
	}
	else if(MOTOR_RUN_FORWARD== status)
	{	
		GPIO_WriteBit(GPIOD, GPIO_Pin_13, Bit_SET);
		GPIO_WriteBit(GPIOC, GPIO_Pin_11, Bit_RESET);
	}
	else if(MOTOR_RUN_BACKWARD== status)
	{	
		GPIO_WriteBit(GPIOD, GPIO_Pin_13, Bit_RESET);
		GPIO_WriteBit(GPIOC, GPIO_Pin_11, Bit_SET);
	}
	#endif
	MotorStatus.DoorSta = status;
}



