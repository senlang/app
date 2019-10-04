#ifndef _BELT_H_
#define _BELT_H_

#include "stm32f10x.h"

#define BELT_RUN_TIME 10


typedef enum
{
	BELT_STOP = 0,	//传送带停止
	BELT_RUN,		//传送带运行
}BELT_WORK_ENUM;

typedef enum
{
	PUSH_BELT = 0,	//传送带停止
	COLLECT_BELT,		//传送带运行
}BELT_ENUM;



/*传送带控制初始化*/
void Belt_Init(void);

void Push_Belt_Set(BELT_WORK_ENUM status);

int Collect_Belt_Run(void);

uint8_t Push_Belt_Check(void);

void Belt_Set(BELT_ENUM belt,BELT_WORK_ENUM status);

void PushBeltControl(BELT_WORK_ENUM status);

#endif

