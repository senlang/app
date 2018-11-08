#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stm32f10x.h"


typedef struct
{
	_Bool MotorSta;
	_Bool ConveyoeSta;
} MOTOR_STATUS;

extern MOTOR_STATUS MotorStatus;


/*���״̬*/
typedef enum
{
	MOTOR_STOP = 0,		//�������ֹͣ
	MOTOR_RUN_FORWARD,	//���������ת
	MOTOR_RUN_BACKWARD,	//���������ת
} MOTOR_ENUM;


typedef enum
{
	CONVEYOR_STOP = 0,	//���ʹ�ֹͣ
	CONVEYOR_RUN,		//���ʹ�����
}CONVEYOR_ENUM;





/*������Ƴ�ʼ��*/
void Motor_Init(void);

/*�������*/
void  Motor_Set(MOTOR_ENUM status);

/*���ʹ����Ƴ�ʼ��*/
void Conveyor_Init(void);

void Conveyor_set(CONVEYOR_ENUM status);

void Motor_Start(void);
void track_calibrate(void);


void Conveyor_run(void);

uint8_t Conveyor_check(void);


#endif
