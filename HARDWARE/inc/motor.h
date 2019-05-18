#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stm32f10x.h"


typedef struct
{
	_Bool MotorSta;
	_Bool ConveyoeSta;
	_Bool DoorSta;
} MOTOR_STATUS;

extern MOTOR_STATUS MotorStatus;


/*���״̬*/
typedef enum
{
	MOTOR_STOP = 0,		//�������ֹͣ
	MOTOR_RUN_FORWARD,	//���������ת
	MOTOR_RUN_BACKWARD,	//���������ת
} MOTOR_ENUM;


/*������Ƴ�ʼ��*/
void Motor_Init(void);

/*�������*/
void  Motor_Set(MOTOR_ENUM status);

void Motor_Start(void);

void track_calibrate(void);

void Door_Control_Init(void);

void Door_Control_Set(MOTOR_ENUM status);



#endif
