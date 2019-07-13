/**
	************************************************************
	************************************************************
	************************************************************
	*	�ļ����� 	push_belt.c
	*
	*	���ߣ� 		
	*
	*	���ڣ� 		
	*
	*	�汾�� 		V1.0
	*
	*	˵���� 		�����ʼ��������
	*
	*	�޸ļ�¼��	
	************************************************************
	************************************************************
	************************************************************
**/

//��Ƭ��ͷ�ļ�
#include "stm32f10x.h"
//ͨѶЭ��
#include "stm32_protocol.h"
//Ӳ������
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
*	�������ƣ�	Belt_Init
*
*	�������ܣ�	���ʹ������ʼ��
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		
************************************************************
*/

void Belt_Init(void)
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
	
	Belt_Set(PUSH_BELT, BELT_STOP);
	Belt_Set(COLLECT_BELT, BELT_STOP);

	memset(&drag_push_time[0], 0x00, sizeof(drag_push_time));
}

//�����ǰ�����ʹ�
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
	return 0;//�����������ʹ�

	if(drag_push_time_calc == drag_push_time_calc_pre)
	return 1;
	
	drag_push_time_calc = drag_push_time_calc_pre;
	return 0;
}



void PushBeltControl(uint8_t status)
{
	Belt_Set(PUSH_BELT, status);
}



