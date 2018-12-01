//单片机头文件
#include "stm32f10x.h"

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

//C库
#include <string.h>
#include <stdio.h>

#include "stm32_protocol.h"



//看门狗任务
#define IWDG_TASK_PRIO		6
#define IWDG_STK_SIZE		64
OS_STK IWDG_TASK_STK[IWDG_STK_SIZE];
void IWDG_Task(void *pdata);


//心跳任务
#define HEART_TASK_PRIO		7
#define HEART_STK_SIZE		256
OS_STK HEART_TASK_STK[HEART_STK_SIZE]; //
void HEART_Task(void *pdata);



//UART1 下行数据接收
#define UP_RECEIVE_TASK_PRIO		8 //
#define UP_RECEIVE_STK_SIZE		768
OS_STK UP_RECEIVE_TASK_STK[UP_RECEIVE_STK_SIZE]; //
void UART1_RECEIVE_Task(void *pdata);

//UART2 上行数据接收
#define DOWN_RECEIVE_TASK_PRIO		9 //
#define DOWN_RECEIVE_STK_SIZE		512
OS_STK DOWN_RECEIVE_TASK_STK[DOWN_RECEIVE_STK_SIZE]; //
void UART2_RECEIVE_Task(void *pdata);


//传感器任务
#define SENSOR_TASK_PRIO	10
#define SENSOR_STK_SIZE		128
OS_STK SENSOR_TASK_STK[SENSOR_STK_SIZE]; 
void SENSOR_Task(void *pdata);


//电机控制
#define MOTOR_TASK_PRIO		11
#define MOTOR_STK_SIZE		256
OS_STK MOTOR_TASK_STK[MOTOR_STK_SIZE];
void MOTOR_Task(void *pdata);


//传送带控制任务
#define TRACK_TASK_PRIO		12
#define TRACK_STK_SIZE		512
OS_STK TRACK_TASK_STK[TRACK_STK_SIZE];
void Conveyor_Task(void *pdata);


//按键任务
#define KEY_TASK_PRIO		13
#define KEY_STK_SIZE		256
OS_STK KEY_TASK_STK[KEY_STK_SIZE];
void KEY_Task(void *pdata);



OS_EVENT *SemOfMotor;        	//Motor控制信号量
OS_EVENT *SemOfUart1RecvData;	//uart1 串口接收数据信号量
OS_EVENT *SemOfKey;				// 按键控制信号量
OS_EVENT *SemOfConveyor;        	//Motor控制信号量


extern struct status_report_request_info_struct  heart_info;



/*
************************************************************
*	函数名称：	Hardware_Init
*
*	函数功能：	硬件初始化
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		初始化单片机功能以及外接设备
************************************************************
*/
void Hardware_Init(void)
{
	int i = 0;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);								//中断控制器分组设置

	delay_init();																//systick初始化
	
	Led_Init();																	//LED初始化

	Key_Init();																	//按键初始化

	EXTIX_Init();

	Motor_Init();
	
	Door_Control_Init();
	
	Door_Key_Init();

	Sensor_Init();

	Conveyor_Init();

	Track_Init();
	
	Debug_USART_Config();														//初始化串口   115200bps	

	Usart1_Init(115200);
	
	Usart2_Init(115200);
	
	Iwdg_Init(4, 1250); 														//64分频，每秒625次，重载1250次，2s

	BoardId_Init();

	for(i = 0; i < 10; i++)
	{
		Led_Set(LED_1, LED_ON);
		delay_ms(100);
		Led_Set(LED_1, LED_OFF);
		delay_ms(100);
	}
	
	UsartPrintf(USART_DEBUG, "Hardware init OK\r\n");						//提示初始化完成	
}

/*
************************************************************
*	函数名称：	main
*
*	函数功能：	完成初始化任务，创建应用任务并执行
*
*	入口参数：	无
*
*	返回参数：	0
*
*	说明：		
************************************************************
*/
int main(void)
{	
	#if 0
	int i = 0;
	Led_Init();																	//LED初始化

	while(1)
	{
		for(i=0; i< 72000; i++)
		;
		Led_Set(LED_1, LED_ON);
		
		for(i=0; i< 72000; i++)
		;
		Led_Set(LED_1, LED_OFF);
	}
	#else
	Hardware_Init();								//硬件初始化

	OSInit();										//RTOS初始化
	
	//创建应用任务
	
	OSTaskCreate(IWDG_Task, (void *)0, (OS_STK*)&IWDG_TASK_STK[IWDG_STK_SIZE - 1], IWDG_TASK_PRIO);

	OSTaskCreate(UART1_RECEIVE_Task, (void *)0, (OS_STK*)&UP_RECEIVE_TASK_STK[UP_RECEIVE_STK_SIZE - 1], UP_RECEIVE_TASK_PRIO);

	OSTaskCreate(UART2_RECEIVE_Task, (void *)0, (OS_STK*)&DOWN_RECEIVE_TASK_STK[DOWN_RECEIVE_STK_SIZE - 1], DOWN_RECEIVE_TASK_PRIO);
	
	OSTaskCreate(HEART_Task, (void *)0, (OS_STK*)&HEART_TASK_STK[HEART_STK_SIZE - 1], HEART_TASK_PRIO);
	
	OSTaskCreate(MOTOR_Task, (void *)0, (OS_STK*)&MOTOR_TASK_STK[MOTOR_STK_SIZE- 1], MOTOR_TASK_PRIO);

	OSTaskCreate(Conveyor_Task, (void *)0, (OS_STK*)&TRACK_TASK_STK[TRACK_STK_SIZE- 1], TRACK_TASK_PRIO);

	OSTaskCreate(SENSOR_Task, (void *)0, (OS_STK*)&SENSOR_TASK_STK[SENSOR_STK_SIZE- 1], SENSOR_TASK_PRIO);

	OSTaskCreate(KEY_Task, (void *)0, (OS_STK*)&KEY_TASK_STK[KEY_STK_SIZE- 1], KEY_TASK_PRIO);
	
	UsartPrintf(USART_DEBUG, "OSStart\r\n");		//提示任务开始执行
	
	OSStart();										//开始执行任务
	#endif
	return 0;

}

/*
************************************************************
*	函数名称：	IWDG_Task
*
*	函数功能：	清除看门狗
*
*	入口参数：	void类型的参数指针
*
*	返回参数：	无
*
*	说明：		看门狗任务
************************************************************
*/
void IWDG_Task(void *pdata)
{

	while(1)
	{
	
		Iwdg_Feed(); 		//喂狗
		
		RTOS_TimeDly(50); 	//挂起任务250ms
		//UsartPrintf(USART_DEBUG, "feed dog!!!!\r\n");		//提示任务开始执行
	}

}

void UART1_RECEIVE_Task(void *pdata)
{
    INT8U            err;
	SemOfUart1RecvData = OSSemCreate(0);

	while(1)
	{
		OSSemPend(SemOfUart1RecvData, 0u, &err);
		do{
			if(uart1_receive_data() == 0)
				break;
			RTOS_TimeDly(10);
		}while(1);
	}

}

void UART2_RECEIVE_Task(void *pdata)
{

	while(1)
	{
		uart2_receive_data();
		RTOS_TimeDly(10);
	}
}


void HEART_Task(void *pdata)
{
	int heart_count = 0;
	
	while(1)
	{	
		Led_Set(LED_1, LED_OFF);
		RTOS_TimeDlyHMSM(0, 0, 1, 0);	//挂起任务1s
		Led_Set(LED_1, LED_ON);
		RTOS_TimeDlyHMSM(0, 0, 1, 0);	//挂起任务1s
		heart_count++;

		if(heart_count == 5)
		{
			UsartPrintf(USART_DEBUG, "Heart Report--------\r\n");
			board_send_message(STATUS_REPORT_REQUEST, &heart_info);
			heart_count = 0;
		}
	}
}
#if 1
void MOTOR_Task(void *pdata)
{
    INT8U            err;
	SemOfMotor = OSSemCreate(0);
	
	while(1)
	{
		OSSemPend(SemOfMotor, 0u, &err);
		
		UsartPrintf(USART_DEBUG, "Run Motor----------\r\n");		//提示任务开始执行
		Motor_Start();
	}

	//OSSemDel(SemOfMotor, 0, &err);
}

void Conveyor_Task(void *pdata)
{
	uint8_t conveyor = 0;
	uint8_t run_time = 10;
	
	SemOfConveyor= OSSemCreate(0);
	
	while(1)
	{
		conveyor = Conveyor_check();		
		if(conveyor == 1)
		{
			if(Conveyor_run() != 0 )
			{
				mcu_push_medicine_complete();//所有单板出货完成
				
				Door_Control_Set(MOTOR_RUN_BACKWARD);
				do{
					RTOS_TimeDlyHMSM(0, 0, 0, 100);
				}while(Door_Key_Detect(DOOR_OPEN) == SENSOR_NO_DETECT);
				Door_Control_Set(MOTOR_STOP);
				UsartPrintf(USART_DEBUG, "Open The Door, End!!!!!!!!!!\r\n");
			
				RTOS_TimeDlyHMSM(0, 0, run_time, 0);
				Door_Control_Set(MOTOR_RUN_FORWARD);
				do{
					if(Sensor_Detect() == SENSOR_DETECT)
					{
						Door_Control_Set(MOTOR_STOP);
						UsartPrintf(USART_DEBUG, "Close Door Detect Somebody, Stop!!!!!!!!!!\r\n");
					}
					else
					{
						Door_Control_Set(MOTOR_RUN_FORWARD);
					}
					RTOS_TimeDlyHMSM(0, 0, 0, 100);
				}while(Door_Key_Detect(DOOR_CLOSE) == SENSOR_NO_DETECT);
				Door_Control_Set(MOTOR_STOP);
				UsartPrintf(USART_DEBUG, "Close The DOor, End!!!!!!!!!!\r\n");
			}
			conveyor = 0;
		}
		RTOS_TimeDlyHMSM(0, 0, 2, 0);
	}
}

/*
************************************************************
*	函数名称：	KEY_Task
*
*	函数功能：	扫描按键是否按下，如果有按下，进行对应的处理
*
*	入口参数：	void类型的参数指针
*
*	返回参数：	无
*
*	说明：		按键任务
************************************************************
*/
void KEY_Task(void *pdata)
{
    INT8U            err;
	SemOfKey = OSSemCreate(0);
	
	while(1)
	{
		OSSemPend(SemOfKey, 0u, &err);
		Keyboard();
		track_calibrate();
	}
//	OSSemDel(SemOfKey, 0, &err);
}
extern void iap_load_app(u32 appxaddr);


void SENSOR_Task(void *pdata)
{
	while(1)
	{	
		#if 0
		push_test();
		RTOS_TimeDlyHMSM(0, 0, 0, 100);	//
		replenish_test();
		RTOS_TimeDlyHMSM(0, 0, 0, 100);	//
		test_test();
		RTOS_TimeDlyHMSM(0, 0, 0, 100);	//
		calibrate_test();
		RTOS_TimeDlyHMSM(0, 0, 15, 0);	//
		replenish_complete_test();
		#endif
		
		RTOS_TimeDlyHMSM(0, 0, 15, 0);	//
		//UsartPrintf(USART_DEBUG, "will jump\r\n");
		//iap_load_app(0x08010000);
	}
}
#endif

