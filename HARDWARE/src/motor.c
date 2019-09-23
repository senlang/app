/**
	************************************************************
	************************************************************
	************************************************************
	*	�ļ����� 	motor.c
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

extern uint8_t g_track_state;

/*
************************************************************
*	�������ƣ�	Motor_Init
*
*	�������ܣ�	�������Ƴ�ʼ��
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		
************************************************************
*/
void Motor_Init(void)
{
	
	GPIO_InitTypeDef gpioInitStrcut;

	//ʹ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	//IO����
	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	//IO��ʼ��
	GPIO_Init(GPIOC, &gpioInitStrcut);
    
    Motor_Set(MOTOR_STOP);

	motor_enqueue_idx = 0;
	motor_dequeue_idx = 0;
}




/*
************************************************************
*	�������ƣ�	Motor_Set
*
*	�������ܣ�	�������ת����
*
*	��ڲ�����	status��
*
*	���ز�����	��
*
*	˵����		
************************************************************
*/
void Motor_Set(uint8_t status)
{
	UsartPrintf(USART_DEBUG, "Motor_Set:%d\r\n", status);
	
	if(MOTOR_STOP == status)
	{	
		GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_RESET);
		GPIO_WriteBit(GPIOC, GPIO_Pin_5, Bit_RESET);
		motor_run_direction = status;
		g_track_state = TRACK_STANDBY;
	}
	else if(MOTOR_RUN_FORWARD== status)
	{	
		GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_SET);
		GPIO_WriteBit(GPIOC, GPIO_Pin_5, Bit_RESET);
		motor_run_direction = status;
		
		g_track_state = TRACK_WORKING;
	}
	else if(MOTOR_RUN_BACKWARD== status)
	{	
		GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_RESET);
		GPIO_WriteBit(GPIOC, GPIO_Pin_5, Bit_SET);
		motor_run_direction = status;
		
		g_track_state = TRACK_WORKING;
	}
	MotorStatus.MotorSta = status;
}




/*ȡ���ŵ�����Ƴ�ʼ��*/
void Door_Control_Init(void)
{
	return;
	/*
	GPIO_InitTypeDef gpioInitStrcut;

	//ʹ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC, ENABLE);
	
	//IO����
	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_13;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	//IO��ʼ��
	GPIO_Init(GPIOD, &gpioInitStrcut);


	//IO����
	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_11;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	//IO��ʼ��
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



