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


/*电机状态*/
typedef enum
{
	MOTOR_STOP = 0,		//货道电机停止
	MOTOR_RUN_FORWARD,	//货道电机正转
	MOTOR_RUN_BACKWARD,	//货道电机反转
} MOTOR_ENUM;


typedef enum
{
	CONVEYOR_STOP = 0,	//传送带停止
	CONVEYOR_RUN,		//传送带运行
}CONVEYOR_ENUM;


typedef enum
{
	DOOR_OPEN = 0,	//开
	DOOR_CLOSE,		//关
}Door_Detect_ENUM;


typedef enum
{
	SENSOR_NO_DETECT = 0,	//开
	SENSOR_DETECT,		//关
}SENSOR_DETECT_ENUM;




/*电机控制初始化*/
void Motor_Init(void);

/*电机控制*/
void  Motor_Set(MOTOR_ENUM status);

/*传送带控制初始化*/
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
