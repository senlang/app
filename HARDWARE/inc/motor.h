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


typedef enum
{
	CONVEYOR_STOP = 0,	//���ʹ�ֹͣ
	CONVEYOR_RUN,		//���ʹ�����
}CONVEYOR_ENUM;


typedef enum
{
	DOOR_OPEN = 0,	//��
	DOOR_CLOSE,		//��
}Door_Detect_ENUM;


typedef enum
{
	SENSOR_NO_DETECT = 0,	//��
	SENSOR_DETECT,		//��
}SENSOR_DETECT_ENUM;




/*������Ƴ�ʼ��*/
void Motor_Init(void);

/*�������*/
void  Motor_Set(MOTOR_ENUM status);

/*���ʹ����Ƴ�ʼ��*/
void Conveyor_Init(void);

void Conveyor_set(CONVEYOR_ENUM status);

void Motor_Start(void);

void track_calibrate(void);

int Conveyor_run(void);

uint8_t Conveyor_check(void);

void Door_Control_Init(void);

void Door_Control_Set(MOTOR_ENUM status);

void Sensor_Init(void);

unsigned char Sensor_Detect(void);

void Door_Key_Init(void);

unsigned char Door_Key_Detect(unsigned char door_detect);


#endif
