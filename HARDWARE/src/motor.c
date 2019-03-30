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
#include "track.h"

#include "stdlib.h"

#define CONVEYOR_DELAY 20

MOTOR_STATUS MotorStatus;
extern struct motor_control_struct  motor_struct[TOTAL_PUSH_CNT];
extern int motor_enqueue_idx;
extern int motor_dequeue_idx;

extern unsigned char calibrate_track_selected;
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
*	函数名称：	Conveyor_Init
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

void Conveyor_Init(void)
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

	
	memset(&drag_push_time[0], 0x00, sizeof(drag_push_time));
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
	else if(CONVEYOR_RUN == status)
	{	
		GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);
		GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
	}
	MotorStatus.ConveyoeSta = status;
}

int cmp(const void *a,const void *b)
{
    return *(uint16_t *)b - *(uint16_t *)a;
}


int Conveyor_run(void)
{
	uint8_t delay_s = 0;

	qsort(drag_push_time, BOARD_ID_MAX, sizeof(drag_push_time[0]),cmp);

	delay_s = drag_push_time[0]/10;

	drag_push_time_calc_pre = drag_push_time_calc_pre = 0;
	memset(&drag_push_time[0], 0x00, sizeof(drag_push_time));

	
	//UsartPrintf(USART_DEBUG, "Conveyor_run %ds-------------\r\n", delay_s);
	//if(delay_s == 0)
	//return delay_s;
	
	Conveyor_set(CONVEYOR_RUN);
	
	RTOS_TimeDlyHMSM(0, 0, CONVEYOR_DELAY, 0);
	//RTOS_TimeDlyHMSM(0, 0, delay_s + CONVEYOR_DELAY, 0);
	
	Conveyor_set(CONVEYOR_STOP);

	return delay_s;
}

uint8_t Conveyor_check(void)
{
	if((0 == drag_push_time_calc_pre && 0 == drag_push_time_calc)||(drag_push_time_calc_pre == 0))
	return 0;//不必启动传送带

	if(drag_push_time_calc == drag_push_time_calc_pre)
	return 1;
	
	drag_push_time_calc = drag_push_time_calc_pre;
	return 0;
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
		
		//Conveyor_set(CONVEYOR_RUN);

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
	
	
	//if(MotorStatus.ConveyoeSta == CONVEYOR_RUN)
	//{
	//	RTOS_TimeDlyHMSM(0, 0, 30, 0);
	//	Conveyor_set(CONVEYOR_STOP);
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
	UsartPrintf(USART_DEBUG, "calibrate_track_selected[%d]calibrate_enable[%d]\r\n", calibrate_track_selected, calibrate_enable);

	if((calibrate_track_selected == 255))
	{
		//Motor_Set(MOTOR_STOP);
		//set_track(calibrate_track_selected, MOTOR_STOP);
		calibrate_enable = 0;
		return;
	}

	calibrate_enable = 1;
	if((key_status.Key0Stat == KEYUP)&&(key_status.Key1Stat == KEYUP))
	{
		Motor_Set(MOTOR_STOP);
		set_track(calibrate_track_selected, MOTOR_STOP);
	}
	
	if((key_status.Key0Stat == KEYDOWN))
	{
		Motor_Set(MOTOR_RUN_FORWARD);
		set_track(calibrate_track_selected, MOTOR_RUN_FORWARD);
	}

	if((key_status.Key1Stat== KEYDOWN))
	{
		Motor_Set(MOTOR_RUN_BACKWARD);
		set_track(calibrate_track_selected, MOTOR_RUN_BACKWARD);
	}
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

/*人体检查PB4*/
void Sensor_Init(void)
{
	GPIO_InitTypeDef gpioInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	gpioInitStructure.GPIO_Mode = GPIO_Mode_IPU;
	gpioInitStructure.GPIO_Pin = GPIO_Pin_1;
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

/*开关门按钮检查*/
void Door_Key_Init(void)
{
	GPIO_InitTypeDef gpioInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/*关门检测*/
	gpioInitStructure.GPIO_Mode = GPIO_Mode_IPU;
	gpioInitStructure.GPIO_Pin = GPIO_Pin_3;
	gpioInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioInitStructure);

	/*开门检测*/
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

	if(!GPIO_ReadInputDataBit(GPIOx, GPIO_Pin))//未发现接入为低
	{
		return SENSOR_DETECT;
	}
	else									//发现接入为高
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


