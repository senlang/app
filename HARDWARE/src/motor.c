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


//LEDͷ�ļ�
#include "motor.h"
#include "usart.h"
#include "stm32_protocol.h"
#include "delay.h"



MOTOR_STATUS MotorStatus;
extern struct push_medicine_request_info_struct push_srtuct[TOTAL_PUSH_CNT];
extern int enqueue_push_index;
extern int dequeue_push_index;


/*
************************************************************
*	�������ƣ�	Motor_Init
*
*	�������ܣ�	Motor��ʼ��
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		LED4-PB6	LED5-PB7	LED6-PB8	LED7-PB9
				�ߵ�ƽ�ص�		�͵�ƽ����
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
}


void Conveyor_Init()
{
	
	GPIO_InitTypeDef gpioInitStrcut;

	//ʹ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	//IO����
	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	//IO��ʼ��
	GPIO_Init(GPIOA, &gpioInitStrcut);
	
	Conveyor_set(CONVEYOR_STOP);
}




/*
************************************************************
*	�������ƣ�	Motor_Set
*
*	�������ܣ�	LED4����
*
*	��ڲ�����	status��LED_ON-����	LED_OFF-�ص�
*
*	���ز�����	��
*
*	˵����		
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



void Push_Medicine_Start(void)
{
	int push_cnt = 0;
	int i = 0;
	uint16_t last_keep_time = 0;
	uint8_t delay_s = 0;
	uint16_t delay_ms = 0;
	
	
	if(dequeue_push_index < enqueue_push_index)
	{
		push_cnt = enqueue_push_index - dequeue_push_index;
		for(i = 0; i < push_cnt; i++)
		{
			UsartPrintf(USART_DEBUG, "push_srtuct[%d].board_id = 0x%04x\r\n", dequeue_push_index, push_srtuct[dequeue_push_index].board_id);
			UsartPrintf(USART_DEBUG, "push_srtuct[%d].medicine_track_number = 0x%04x\r\n", dequeue_push_index, push_srtuct[dequeue_push_index].medicine_track_number);
			UsartPrintf(USART_DEBUG, "push_srtuct[%d].push_time = 0x%04x\r\n", dequeue_push_index, push_srtuct[dequeue_push_index].push_time);

			Motor_Set(MOTOR_RUN_BACKWARD);
			Conveyor_set(CONVEYOR_RUN);
			
			set_track(push_srtuct[dequeue_push_index].medicine_track_number, MOTOR_RUN_FORWARD);
			

			delay_s = push_srtuct[dequeue_push_index].push_time/10;
			delay_ms = (push_srtuct[dequeue_push_index].push_time%10) * 100;

			UsartPrintf(USART_DEBUG, "delay_s[%d]delay_ms[%d]\r\n", delay_s, delay_ms);
			last_keep_time = delay_s;
				
			RTOS_TimeDlyHMSM(0, 0, delay_s, delay_ms);
			

			
			set_track(push_srtuct[dequeue_push_index].medicine_track_number, MOTOR_STOP);
			Motor_Set(MOTOR_STOP);
			
			dequeue_push_index++;
		}
		


	}
	else if(dequeue_push_index > enqueue_push_index)
	{
		push_cnt = TOTAL_PUSH_CNT - dequeue_push_index + enqueue_push_index;
	}
	
	if(MotorStatus.ConveyoeSta == CONVEYOR_RUN)
	{
		RTOS_TimeDlyHMSM(0, 0, last_keep_time, 0);
		Conveyor_set(CONVEYOR_STOP);
	}

}


