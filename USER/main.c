//��Ƭ��ͷ�ļ�
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
#include "timer.h"


//C��
#include <string.h>
#include <stdio.h>

#include "stm32_protocol.h"


#define SW_VERSION		"SV 1.0.0"

//���Ź�����
#define IWDG_TASK_PRIO		6
#define IWDG_STK_SIZE		64
OS_STK IWDG_TASK_STK[IWDG_STK_SIZE];
void IWDG_Task(void *pdata);


//UART1 �������ݽ���
#define UP_RECEIVE_TASK_PRIO		7 //
#define UP_RECEIVE_STK_SIZE		512
OS_STK UP_RECEIVE_TASK_STK[UP_RECEIVE_STK_SIZE]; //
void UART1_RECEIVE_Task(void *pdata);


//UART2 �������ݽ���
#define DOWN_RECEIVE_TASK_PRIO		8 //
#define DOWN_RECEIVE_STK_SIZE		512
OS_STK DOWN_RECEIVE_TASK_STK[DOWN_RECEIVE_STK_SIZE]; //
void UART2_RECEIVE_Task(void *pdata);


//uart1 ������Ϣ����
#define PARSE_TASK_PRIO		9
#define PARSE_STK_SIZE		512
OS_STK PARSE_TASK_STK[PARSE_STK_SIZE];
void Info_Parse_Task(void *pdata);



//����������������������
#define TRACK_TASK_PRIO		10
#define TRACK_STK_SIZE		512
OS_STK TRACK_TASK_STK[TRACK_STK_SIZE];
void Track_Run_Task(void *pdata);



//���ʹ����ſ�������
#define Drug_Push_TASK_PRIO		11
#define Drug_Push_STK_SIZE		512
OS_STK Drug_Push_TASK_STK[Drug_Push_STK_SIZE];
void Drug_Push_Task(void *pdata);


//��������
#define KEY_TASK_PRIO		12
#define KEY_STK_SIZE		256
OS_STK KEY_TASK_STK[KEY_STK_SIZE];
void KEY_Task(void *pdata);


//����������
#define SENSOR_TASK_PRIO	13
#define SENSOR_STK_SIZE		128
OS_STK SENSOR_TASK_STK[SENSOR_STK_SIZE]; 
void SENSOR_Task(void *pdata);


//���Ե������
#define MOTOR_TASK_PRIO		14
#define MOTOR_STK_SIZE		256
OS_STK MOTOR_TASK_STK[MOTOR_STK_SIZE];
void MOTOR_Task(void *pdata);


//����ʱ��ͳ��
#define trigger_calc_runtime_TASK_PRIO		15
#define trigger_calc_runtime_STK_SIZE		384
OS_STK trigger_calc_runtime_TASK_STK[trigger_calc_runtime_STK_SIZE]; //
void trigger_calc_runtime_Task(void *pdata);

//��������
#define HEART_TASK_PRIO		16
#define HEART_STK_SIZE		256
OS_STK HEART_TASK_STK[HEART_STK_SIZE]; //
void HEART_Task(void *pdata);


OS_EVENT *SemOfMotor;        	//Motor�����ź���
OS_EVENT *SemOfUart1RecvData;	//uart1 ���ڽ��������ź���
OS_EVENT *SemOfUart2RecvData;	//uart2 ���ڽ��������ź���
OS_EVENT *SemOfDataParse;	//���ݽ����߳��ź���

OS_EVENT *SemOfKey;				// ���������ź���
OS_EVENT *SemOfConveyor;        	//Motor�����ź���
OS_EVENT *SemOfTrack;        	//track �����ź���
OS_EVENT *SemOfCalcTime;        	//��������ʱ��ͳ���ź���

uint8_t trigger_calc_flag = 0;
uint8_t trigger_calc_runtime = 0;
uint8_t cur_calc_track = 0;
uint8_t calc_track_start_idx = 0;
uint8_t calc_track_count = 0;

uint8_t motor_run_detect_flag = 0;
uint8_t motor_run_detect_track_num = 0;
uint8_t key_stat = 0;


extern struct status_report_request_info_struct  heart_info;
extern uint8_t track_work;


/*
************************************************************
*	�������ƣ�	Hardware_Init
*
*	�������ܣ�	Ӳ����ʼ��
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		��ʼ����Ƭ�������Լ�����豸
************************************************************
*/
void Hardware_Init(void)
{
	int i = 0;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);								//�жϿ�������������

	delay_init();																//systick��ʼ��
	
	Led_Init();																	//LED��ʼ��

	Key_Init();																	//������ʼ��

	EXTIX_Init();

	Motor_Init();
	
	Door_Control_Init();
	
	Door_Key_Init();

	Sensor_Init();

	Conveyor_Init();

	Track_Init();
	
	Debug_USART_Config();														//��ʼ������   115200bps	

	Usart1_Init(115200);
	
	Usart2_Init(115200);
	
	
	//TIM3_Int_Init(9999,7199);//10Khz�ļ���Ƶ�ʣ�������5000Ϊ500ms  
	TIM3_Int_Init(999,7199);//10Khz�ļ���Ƶ�ʣ�������5000Ϊ500ms  


	BoardId_Init();

	for(i = 0; i < 10; i++)
	{
		Led_Set(LED_1, LED_ON);
		delay_ms(50);
		Led_Set(LED_1, LED_OFF);
		delay_ms(50);
	}
	
	Iwdg_Init(4, 1250); 														//64��Ƶ��ÿ��625�Σ�����1250�Σ�2s

	heart_info.board_status = FIRSTBOOT_STATUS;
	board_send_message(STATUS_REPORT_REQUEST, &heart_info);
	heart_info.board_status = STANDBY_STATUS;
	
	//UsartPrintf(USART_DEBUG, "Hardware init OK\r\n");						//��ʾ��ʼ�����	
}

/*
************************************************************
*	�������ƣ�	main
*
*	�������ܣ�	��ɳ�ʼ�����񣬴���Ӧ������ִ��
*
*	��ڲ�����	��
*
*	���ز�����	0
*
*	˵����		
************************************************************
*/
int main(void)
{	
	Hardware_Init();								//Ӳ����ʼ��

	UsartPrintf(USART_DEBUG, "SW_VERSION: %s\r\n", SW_VERSION);		
	UsartPrintf(USART_DEBUG, "Version Build: %s %s\r\n", __DATE__, __TIME__);


	OSInit();										//RTOS��ʼ��
	
	//����Ӧ������
	
	OSTaskCreate(IWDG_Task, (void *)0, (OS_STK*)&IWDG_TASK_STK[IWDG_STK_SIZE - 1], IWDG_TASK_PRIO);

	OSTaskCreate(UART1_RECEIVE_Task, (void *)0, (OS_STK*)&UP_RECEIVE_TASK_STK[UP_RECEIVE_STK_SIZE - 1], UP_RECEIVE_TASK_PRIO);

	OSTaskCreate(UART2_RECEIVE_Task, (void *)0, (OS_STK*)&DOWN_RECEIVE_TASK_STK[DOWN_RECEIVE_STK_SIZE - 1], DOWN_RECEIVE_TASK_PRIO);
	
	OSTaskCreate(HEART_Task, (void *)0, (OS_STK*)&HEART_TASK_STK[HEART_STK_SIZE - 1], HEART_TASK_PRIO);
	
	OSTaskCreate(MOTOR_Task, (void *)0, (OS_STK*)&MOTOR_TASK_STK[MOTOR_STK_SIZE- 1], MOTOR_TASK_PRIO);

	OSTaskCreate(Drug_Push_Task, (void *)0, (OS_STK*)&Drug_Push_TASK_STK[Drug_Push_STK_SIZE- 1], Drug_Push_TASK_PRIO);

	//OSTaskCreate(SENSOR_Task, (void *)0, (OS_STK*)&SENSOR_TASK_STK[SENSOR_STK_SIZE- 1], SENSOR_TASK_PRIO);
	OSTaskCreate(trigger_calc_runtime_Task, (void *)0, (OS_STK*)&trigger_calc_runtime_TASK_STK[trigger_calc_runtime_STK_SIZE- 1], trigger_calc_runtime_TASK_PRIO);

	OSTaskCreate(KEY_Task, (void *)0, (OS_STK*)&KEY_TASK_STK[KEY_STK_SIZE- 1], KEY_TASK_PRIO);

	OSTaskCreate(Info_Parse_Task, (void *)0, (OS_STK*)&PARSE_TASK_STK[PARSE_STK_SIZE- 1], PARSE_TASK_PRIO);

	OSTaskCreate(Track_Run_Task, (void *)0, (OS_STK*)&TRACK_TASK_STK[TRACK_STK_SIZE- 1], TRACK_TASK_PRIO);

	
	UsartPrintf(USART_DEBUG, "OSStart\r\n");		//��ʾ����ʼִ��
	
	OSStart();										//��ʼִ������

	return 0;
}

/*
************************************************************
*	�������ƣ�	IWDG_Task
*
*	�������ܣ�	������Ź�
*
*	��ڲ�����	void���͵Ĳ���ָ��
*
*	���ز�����	��
*
*	˵����		���Ź�����
************************************************************
*/
void IWDG_Task(void *pdata)
{

	while(1)
	{
	
		Iwdg_Feed(); 		//ι��
		
		RTOS_TimeDly(50); 	//��������250ms
		//UsartPrintf(USART_DEBUG, "feed dog!!!!\r\n");		//��ʾ����ʼִ��
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
			RTOS_TimeDly(2);
		}while(1);
	}
}

void Info_Parse_Task(void *pdata)
{
    INT8U            err;
	
	SemOfDataParse = OSSemCreate(0);
	while(1)
	{
		OSSemPend(SemOfDataParse, 0u, &err);
		do{
			if(parse_protocol() == 0)
				break;
		}while(1);
	}

}


void UART2_RECEIVE_Task(void *pdata)
{
    INT8U            err;
	
	SemOfUart2RecvData = OSSemCreate(0);
	while(1)
	{
		OSSemPend(SemOfUart2RecvData, 0u, &err);
		do{
			if(uart2_receive_data() == 0)
				break;
			RTOS_TimeDly(2);
		}while(1);
	}
}


void HEART_Task(void *pdata)
{
	int heart_count = 0;
	
	while(1)
	{	
		Led_Set(LED_1, LED_OFF);
		RTOS_TimeDlyHMSM(0, 0, 1, 0);	//��������1s
		Led_Set(LED_1, LED_ON);
		RTOS_TimeDlyHMSM(0, 0, 1, 0);	//��������1s
		heart_count++;

		if(heart_count == 5)
		{
			UsartPrintf(USART_DEBUG, "Heart Report--------\r\n");
			board_send_message(STATUS_REPORT_REQUEST, &heart_info);
			heart_count = 0;
		}
	}
}

void MOTOR_Task(void *pdata)
{
    INT8U            err;
	SemOfMotor = OSSemCreate(0);
	
	while(1)
	{
		OSSemPend(SemOfMotor, 0u, &err);
		
		UsartPrintf(USART_DEBUG, "Run Motor----------\r\n");		//��ʾ����ʼִ��
		Motor_Start();
	}
	//OSSemDel(SemOfMotor, 0, &err);
}

void Track_Run_Task(void *pdata)
{
    INT8U            err;
	
	SemOfTrack = OSSemCreate(0);
	while(1)
	{
		OSSemPend(SemOfTrack, 0u, &err);
		
		UsartPrintf(USART_DEBUG, "Run Track----------\r\n");		//��ʾ����ʼִ��
		Track_run(track_work);
	}
	//OSSemDel(SemOfMotor, 0, &err);
}

void Drug_Push_Task(void *pdata)
{
	uint8_t conveyor = 0;
	uint8_t delay_time = 10;
	uint8_t run_time = 0;
	INT8U            err;

	SemOfConveyor= OSSemCreate(0);
	
	while(1)
	{		
		OSSemPend(SemOfConveyor, 0u, &err);
		//UsartPrintf(USART_DEBUG, "Will run conveyor!!!!!!!!!!\r\n");
		//conveyor = Conveyor_check();		
		if(1)//(conveyor == 1)
		{
			run_time = 0;
			if(Conveyor_run() != 0 )
			{
				Door_Control_Set(MOTOR_RUN_BACKWARD);
				do{
					RTOS_TimeDlyHMSM(0, 0, 0, 100);
					run_time += 1;

					
					//UsartPrintf(USART_DEBUG, "run_time = %d\r\n", run_time);
					if(run_time >= 300)
					break;
				}while(Door_Key_Detect(DOOR_OPEN) == SENSOR_NO_DETECT);
				
				mcu_push_medicine_open_door_complete();//���е���������,���Ѵ�
				
				UsartPrintf(USART_DEBUG, "Open The Door, End!!!!!!!!!!\r\n");
				Door_Control_Set(MOTOR_STOP);
			
				RTOS_TimeDlyHMSM(0, 0, delay_time, 0);
				Door_Control_Set(MOTOR_RUN_FORWARD);
				run_time = 0;
				do{
					if(Sensor_Detect() == SENSOR_DETECT)
					{
						Door_Control_Set(MOTOR_STOP);
						UsartPrintf(USART_DEBUG, "Close Door Detect Somebody, Stop!!!!!!!!!!\r\n");
					}
					else
					{
						Door_Control_Set(MOTOR_RUN_FORWARD);
						run_time += 1;
					}
					RTOS_TimeDlyHMSM(0, 0, 0, 100);
					
					if(run_time >= 300)
					{
						UsartPrintf(USART_DEBUG, "run_time %d > 150\r\n", run_time);
						break;
					}
				}while(Door_Key_Detect(DOOR_CLOSE) == SENSOR_NO_DETECT);
				Door_Control_Set(MOTOR_STOP);
				mcu_push_medicine_close_door_complete();
				UsartPrintf(USART_DEBUG, "Close The Door, End!!!!!!!!!!\r\n");
			}
			conveyor = 0;
		}
	}
}

/*
************************************************************
*	�������ƣ�	KEY_Task
*
*	�������ܣ�	ɨ�谴���Ƿ��£�����а��£����ж�Ӧ�Ĵ���
*
*	��ڲ�����	void���͵Ĳ���ָ��
*
*	���ز�����	��
*
*	˵����		��������
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
		RTOS_TimeDlyHMSM(0, 0, 15, 0);	//
		//UsartPrintf(USART_DEBUG, "will jump\r\n");
		//iap_load_app(0x08010000);
	}
}

extern uint16_t running_time;

void trigger_calc_runtime_Task(void *pdata)
{
	INT8U			 err;
	uint8_t i = 0;

	SemOfCalcTime = OSSemCreate(0);
	while(1)
	{
		UsartPrintf(USART_DEBUG, "00trigger_calc_runtime_Task run!!!!!!!!!!!!\r\n");
		OSSemPend(SemOfCalcTime, 0u, &err);
		UsartPrintf(USART_DEBUG, "11trigger_calc_runtime_Task run!!!!!!!!!!!!\r\n");
		trigger_calc_flag = 1;
		for(cur_calc_track = calc_track_start_idx; cur_calc_track <= calc_track_count; cur_calc_track ++)
		//for(cur_calc_track = 1; cur_calc_track <= 1; cur_calc_track ++)
		{
			UsartPrintf(USART_DEBUG, "cur_calc_track :%d, calc_track_count :%d\r\n", cur_calc_track, calc_track_count);
			for( i = 0; i < 3; i++)
			{
				trigger_calc_runtime = 0;
				if(i == 0)
				{
					RTOS_TimeDlyHMSM(0, 0, 2, 0);
					
					UsartPrintf(USART_DEBUG, "trigger_calc_runtime_Task, do prepare\r\n");
					trigger_calc_runtime = 1;
					Track_trigger_calc_runtime(1, MOTOR_RUN_FORWARD);
				}
				else if(i == 1)
				{	
					RTOS_TimeDlyHMSM(0, 0, 2, 0);
					
					UsartPrintf(USART_DEBUG, "trigger_calc_runtime_Task, do backward\r\n");
					trigger_calc_runtime = 1;
					Track_trigger_calc_runtime(0, MOTOR_RUN_BACKWARD);
				}
				else if(i == 2)
				{
					RTOS_TimeDlyHMSM(0, 0, 2, 0);
					
					UsartPrintf(USART_DEBUG, "trigger_calc_runtime_Task, do forward\r\n");
					trigger_calc_runtime = 1;
					Track_trigger_calc_runtime(0, MOTOR_RUN_FORWARD);
				}
				do{
					RTOS_TimeDlyHMSM(0, 0, 0, 200);	//

					if(running_time >= 600)
					{
						//UsartPrintf(USART_DEBUG, "Track calc time %d longer than 60s, error!!!!\r\n", running_time);
						break;
					}
				}while(trigger_calc_runtime);

				key_stat = 0;
				if(running_time >= 600)
				{
					UsartPrintf(USART_DEBUG, "Track calc time %d longer than 60s, error!!!!\r\n", running_time);
					running_time = 0;
					break;
				}
				RTOS_TimeDlyHMSM(0, 0, 0, 500); //
			}
		}
		trigger_calc_flag = 0;
	}
	//	OSSemDel(SemOfKey, 0, &err);


}

