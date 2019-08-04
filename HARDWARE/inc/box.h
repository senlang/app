#ifndef _BOX_H_
#define _BOX_H_

#include "sys.h"

#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "motor.h"
#include "iwdg.h"
#include "motor.h"
#include "track.h"
#include "timer.h"
#include "belt.h"
#include "cooling.h"
#include "door.h"
#include "light.h"
#include "sensor.h"
#include "lifter.h"

#include "queue.h"


#define BOX_TEXT_MODE_CLOSE 0
#define BOX_TEXT_MODE_OPEN 1 


//定义药柜相关的结构体
typedef struct
{
	_Bool PushBeltStatus;
	_Bool collectBeltStatus;
} belt_status;

typedef struct
{
	_Bool FrontDoorStatus;
	_Bool BackDoorStatus;
} door_status;

typedef struct
{
	_Bool LightStatus;
} ligth_status;


typedef struct
{
	float tempreture;
	float humidity;
}temp_hum;


typedef struct
{
	belt_status belt;
	door_status door;
	ligth_status light;
	temp_hum th;
} drug_box, *pdrug_box;


extern uint8_t key_init;
extern uint8_t key_stat;
extern uint8_t trigger_calc_runtime;

extern uint8_t motor_run_detect_flag;
extern uint8_t motor_run_detect_track_num;
extern uint8_t motor_run_direction;

extern uint8_t calc_track_start_idx;
extern uint8_t calc_track_count;

extern uint8_t trigger_calc_flag; //0;1;2

extern uint8_t  g_src_board_id;

extern struct node* UartMsgNode;

extern OS_EVENT *SemOfMotor;        	//Motor控制信号量
extern OS_EVENT *SemOfUart1RecvData;	//uart1 串口接收数据信号量
extern OS_EVENT *SemOfUart2RecvData;	//uart2 串口接收数据信号量
extern OS_EVENT *SemOfDataParse;	//数据解析线程信号量
extern OS_EVENT *SemOfKey;				// 按键控制信号量
extern OS_EVENT *SemOfConveyor;        	//Motor控制信号量
extern OS_EVENT *SemOfTrack;        	//track 控制信号量
extern OS_EVENT *SemOfCalcTime;        	//触发货道时间统计信号量
extern OS_EVENT *SemOfOverCurrent;				//过流保护信号量
extern OS_EVENT *MsgMutex;
extern OS_EVENT *SemOfFactoryTest;				//出厂测试


#endif
