#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stm32f10x.h"


typedef struct
{
	_Bool MotorSta;
	_Bool ConveyoeSta;
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





/*电机控制初始化*/
void Motor_Init(void);

/*电机控制*/
void  Motor_Set(MOTOR_ENUM status);

/*传送带控制初始化*/
void Conveyor_Init(void);

void Conveyor_set(CONVEYOR_ENUM status);

void Motor_Start(void);
void track_calibrate(void);


void Conveyor_run(void);

uint8_t Conveyor_check(void);


#endif
