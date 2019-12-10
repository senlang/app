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
#include "adc.h"

#include "queue.h"


#define BOX_TEXT_MODE_CLOSE 0
#define BOX_TEXT_MODE_OPEN 1 


#define TRACK_MAX_RUN_TIME 600
#define MSG_RESEND_TIME_500MS 500	//500s
#define MSG_RESEND_TIME_SLEEP_100MS 100	//100ms

#define BOX_FrontDOOR_CONTROL_BOARD	5
#define BOX_BackDOOR_CONTROL_BOARD	7
#define BOX_COOLING_CONTROL_BOARD 2
#define BOX_LIGHT_CONTROL_BOARD 6
#define HEALTH_MONITOR_BOARD 0xf



#define COOLING_PART1 1
#define COOLING_PART2 2
#define COOLING_PART3 3
#define COOLING_PART4 4


#define RS485_ENABLE


#define COOLING_AREA_TEMPERATURE_MAX 280
#define COOLING_AREA_TEMPERATURE_MIN 220

#define SHADE_AREA_TEMPERATURE_MAX 200
#define SHADE_AREA_TEMPERATURE_MIN 160


//����ҩ����صĽṹ��
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


#define MOTOR_STANDBY_VOLTAGE ((float)2.51 / 3.3 * 4096)
#define NORMAL_RUNNING_VOLTAGE ((float)2.53 / 3.3 * 4096)
#define SHORTCIRCUIT_BLOCK_VOLTAGE ((float)2.6 / 3.3 * 4096)
#define BROKENCIRCUIT_VOLTAGE ((float)2.51 / 3.3 * 4096)


typedef enum
{
	//MOTOR_STANDBY = 0,	//����
	//NORMAL_RUNNING,	//������ת
	SHORTCIRCUIT_BLOCK=1,//��·����ת
	BROKENCIRCUIT,	//��·
}MOTOR_STATUS_ENUM;

typedef enum
{
	TRACK_STANDBY = 0,
	TRACK_WORKING,
	TRACK_IDLE,
}TRACK_STATUS_ENUM;


typedef struct
{
	uint16_t board_push_finish;/*1111 1111ÿһ��bit��ʾ1������*/
	uint16_t board_add_finish;/*1111 1111ÿһ��bit��ʾ1������*/
	uint16_t board_push_ackmsg;/*1111 1111ÿһ��bit��ʾ1������*/
	uint16_t board_add_ackmsg;/*1111 1111ÿһ��bit��ʾ1������*/
} box_struct, *pbox_struct;


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

extern OS_EVENT *SemOfMotor;        	//Motor�����ź���
extern OS_EVENT *SemOfUart1RecvData;	//uart1 ���ڽ��������ź���
extern OS_EVENT *SemOfUart2RecvData;	//uart2 ���ڽ��������ź���
extern OS_EVENT *SemOfDataParse;	//���ݽ����߳��ź���
extern OS_EVENT *SemOfKey;				// ���������ź���
extern OS_EVENT *SemOfConveyor;        	//Motor�����ź���
extern OS_EVENT *SemOfTrack;        	//track �����ź���
extern OS_EVENT *SemOfCalcTime;        	//��������ʱ��ͳ���ź���
extern OS_EVENT *SemOfOverCurrent;				//���������ź���
extern OS_EVENT *MsgMutex;
extern OS_EVENT *SemOfFactoryTest;				//��������
extern uint8_t OverCurrentDetected;	//��������״̬1Ϊ��⵽

extern MOTOR_ENUM track_work;

#endif
