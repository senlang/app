//单片机头文件
#include "stm32f10x.h"
#include "sys.h"

//Box头文件

#include "box.h"

//C库
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


//通讯协议
#include "stm32_protocol.h"
#include "stm32_uart2.h"
#include "protocol_func.h"


//mem
#include "malloc.h"

#define SW_VERSION		"SV 2.0.1"

//看门狗任务
#define IWDG_TASK_PRIO		6
#define IWDG_STK_SIZE		64
OS_STK IWDG_TASK_STK[IWDG_STK_SIZE];
void IWDG_Task(void *pdata);


//UART1 串口数据接收
#define UP_RECEIVE_TASK_PRIO		7 //
#define UP_RECEIVE_STK_SIZE		1024
OS_STK UP_RECEIVE_TASK_STK[UP_RECEIVE_STK_SIZE]; //
void UART1_RECEIVE_Task(void *pdata);


//UART2 串口数据接收
#define DOWN_RECEIVE_TASK_PRIO		8 //
#define DOWN_RECEIVE_STK_SIZE		1024
OS_STK DOWN_RECEIVE_TASK_STK[DOWN_RECEIVE_STK_SIZE]; //
void UART2_RECEIVE_Task(void *pdata);


//uart1 接收消息解析
#define PARSE_TASK_PRIO		9
#define PARSE_STK_SIZE		1024
OS_STK PARSE_TASK_STK[PARSE_STK_SIZE];
void UARTMessageParse_Task(void *pdata);



//出货、补货货道运行任务
#define TRACK_TASK_PRIO		10
#define TRACK_STK_SIZE		512
OS_STK TRACK_TASK_STK[TRACK_STK_SIZE];
void Track_Run_Task(void *pdata);



//传送带、门控制任务
#define Drug_Push_TASK_PRIO		11
#define Drug_Push_STK_SIZE		512
OS_STK Drug_Push_TASK_STK[Drug_Push_STK_SIZE];
void DrugPush_Task(void *pdata);

//消息重传
#define MSG_SEND_TASK_PRIO		12
#define MSG_SEND_STK_SIZE		256
OS_STK MSG_SEND_TASK_STK[MSG_SEND_STK_SIZE]; //
void Message_Send_Task(void *pdata);
void Message_Send_Task_HostBoard(void *pdata);


//过流保护线程
#define OVERCURRENT_TASK_PRIO		13
#define OVERCURRENT_STK_SIZE		256
OS_STK OVERCURRENT_TASK_STK[OVERCURRENT_STK_SIZE];
void Track_OverCurrent_Task(void *pdata);


//轮询任务
#define QUERY_TASK_PRIO		14
#define QUERY_STK_SIZE		256
OS_STK QUERY_TASK_STK[QUERY_STK_SIZE];
void QueryMain_Task(void *pdata);

//货道时长统计
#define Trigger_CalcRuntime_Task_PRIO		15
#define trigger_calc_runtime_STK_SIZE		384
OS_STK Trigger_CalcRuntime_Task_STK[trigger_calc_runtime_STK_SIZE]; //
void Trigger_CalcRuntime_Task(void *pdata);


//心跳任务
#define HEART_TASK_PRIO		16
#define HEART_STK_SIZE		256
OS_STK HEART_TASK_STK[HEART_STK_SIZE]; //
void HeartBeat_Task(void *pdata);


//货道监测
#define TrackMonitor_TASK_PRIO	17
#define TrackMonitor_STK_SIZE		256
OS_STK TrackMonitor_TASK_STK[TrackMonitor_STK_SIZE]; 
void TrackMonitor_Task(void *pdata);


//温度传感器，制冷控制
#define Cooling_TASK_PRIO	18
#define Cooling_STK_SIZE		256
OS_STK Cooling_TASK_STK[Cooling_STK_SIZE]; 
void CoolingControl_Task(void *pdata);


//产测任务
#define FACTORY_TEST_TASK_PRIO		19
#define FACTORY_TEST_STK_SIZE		256
OS_STK FACTORY_TEST_TASK_STK[FACTORY_TEST_STK_SIZE];
void Factory_Test_Task(void *pdata);





OS_EVENT *SemOfMotor;        	//Motor控制信号量
OS_EVENT *SemOfUart1RecvData;	//uart1 串口接收数据信号量
OS_EVENT *SemOfUart2RecvData;	//uart2 串口接收数据信号量
OS_EVENT *SemOfDataParse;	//数据解析线程信号量
OS_EVENT *SemOf485DataParse;	//数据解析线程信号量


OS_EVENT *SemOfKey;				// 按键控制信号量
OS_EVENT *SemOfConveyor;        	//Motor控制信号量
OS_EVENT *SemOfTrack;        	//track 控制信号量
OS_EVENT *SemOfCalcTime;        	//触发货道时间统计信号量
OS_EVENT *SemOfOverCurrent;				//过流保护信号量
OS_EVENT *SemOfFactoryTest;				//出厂测试
OS_EVENT *MsgMutex;

OS_EVENT *SemOf485MsgSend;				//rs485消息轮询发送



uint8_t trigger_calc_flag = 0;
uint8_t trigger_calc_runtime = 0;
uint8_t cur_calc_track = 0;
uint8_t calc_track_start_idx = 0;
uint8_t calc_track_count = 0;

uint8_t motor_run_detect_flag = 0;
uint8_t motor_run_detect_track_num = 0;
uint8_t motor_run_direction = MOTOR_STOP;	//货道电机方向
uint8_t OverCurrentDetected = 0;	//货道开关状态1为检测到



uint8_t key_stat = 0;
uint16_t board_push_finish = 0;/*1111 1111每一个bit表示1个单板*/
uint16_t board_add_finish = 0;/*1111 1111每一个bit表示1个单板*/
uint16_t board_push_ackmsg = 0;/*1111 1111每一个bit表示1个单板*/
uint16_t board_add_ackmsg = 0;/*1111 1111每一个bit表示1个单板*/


uint8_t key_init = 0;

uint8_t  g_src_board_id = 0;
uint8_t g_track_state = 0;
uint8_t g_track_id = 0;


	
MOTOR_ENUM track_work = 0;


extern struct status_report_request_info_struct  heart_info;
extern uint32_t time_passes;

uint8_t board_drug_push_status[BOARD_ID_MAX] = {0};

struct node* UartMsgNode = NULL;

uint8_t NeedClearBuffer = 0;
uint32_t TrunkInitTime = 0;
uint32_t TrackPassTime = 0;
uint16_t TrackPushAllTime = 0;


/*内存块32 *100*/
#ifdef USE_OS_MEM
OS_MEM *MemBuf = NULL;
INT8U MemPartition[100][32];
#endif


//START 任务
//设置任务优先级
#define START_TASK_PRIO      			25 //开始任务的优先级设置为最低
//设置任务堆栈大小
#define START_STK_SIZE  				128
//任务堆栈，8字节对齐	
static OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);		


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
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//中断控制器分组设置

	delay_init();	//systick初始化
	
	Led_Init();		//LED初始化
	
	Debug_USART_Config(); //初始化串口   115200bps	
		
	Usart1_Init(115200);	//初始化串口1
		
	//Usart2_Init(115200);	//初始化串口2 ok

	RS485_Init(9600);	//初始化rs485
	
	EXTIX_Init();	//按键中断初始化

	Key_Init();		//按键初始化

	Motor_Init();	//电机初始化
	
	Door_Control_Init();//
	
	Track_Init();	//货道初始化
	
	//TIM3_Int_Init(9999,7199);//10Khz的计数频率，计数到5000为500ms  
	TIM3_Int_Init(999,7199);//10Khz的计数频率，计数到5000为500ms

	BoardId_Init();

	Adc_Init();

	DHT12_Init();
	
	UsartPrintf(USART_DEBUG, " Current Board ID:0x%x\r\n", g_src_board_id); 
	
	if(g_src_board_id == 1)
	{
		Door_Key_Init();	//取药口滑动门初始化

		Sensor_Init();		//人体检测传感器初始化

		Belt_Init();		//传送带初始化

		Lifter_Init();		//取药升降机初始化
	}
	else if(g_src_board_id == BOX_COOLING_CONTROL_BOARD)
	{
		Cooling_Init(); //制冷设备初始化
	}
	else if(g_src_board_id == BOX_FrontDOOR_CONTROL_BOARD || g_src_board_id == BOX_BackDOOR_CONTROL_BOARD)
	{
		Door_Init();	//前后大门控制初始化
	}
	else if(g_src_board_id == BOX_LIGHT_CONTROL_BOARD)
	{
		Light_Init();	//灯箱初始化
	}
	
	for(i = 0; i < 10; i++)
	{
		Led_Set(LED_1, LED_OFF);
		delay_ms(50);
		Led_Set(LED_1, LED_ON);
		delay_ms(50);
	}

	Iwdg_Init(4, 1250); 														//64分频，每秒625次，重载1250次，2s
}


int main(void)
{ 		   
    INT8U err;
	
   	Hardware_Init();		//系统初始化
   	

	UsartPrintf(USART_DEBUG, "SW_VERSION: %s\r\n", SW_VERSION);		
	UsartPrintf(USART_DEBUG, "Version Build: %s %s\r\n", __DATE__, __TIME__);

	
	OSInit();   

	#ifdef USE_OS_MEM
	/*分配内存*/
    MemBuf = OSMemCreate(MemPartition, 100, 32, &err);
    #else
	mem_init(SRAMIN);			//内部内存池初始化
	#endif
	
   	MessageDealQueueCreate();//消息队列初始化
   	
	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务

	OSStart();	  	


}   	  
//开始任务
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	
	pdata = pdata; 	 
	
	OSStatInit();		//初始化统计任务.这里会延时1秒钟左右	
	
	OS_ENTER_CRITICAL();//进入临界区(无法被中断打断)    
	
	OSTaskCreate(IWDG_Task, (void *)0, (OS_STK*)&IWDG_TASK_STK[IWDG_STK_SIZE - 1], IWDG_TASK_PRIO);

	OSTaskCreate(HeartBeat_Task, (void *)0, (OS_STK*)&HEART_TASK_STK[HEART_STK_SIZE - 1], HEART_TASK_PRIO);

	OSTaskCreate(UART1_RECEIVE_Task, (void *)0, (OS_STK*)&UP_RECEIVE_TASK_STK[UP_RECEIVE_STK_SIZE - 1], UP_RECEIVE_TASK_PRIO);

	OSTaskCreate(UART2_RECEIVE_Task, (void *)0, (OS_STK*)&DOWN_RECEIVE_TASK_STK[DOWN_RECEIVE_STK_SIZE - 1], DOWN_RECEIVE_TASK_PRIO);

	OSTaskCreate(Trigger_CalcRuntime_Task, (void *)0, (OS_STK*)&Trigger_CalcRuntime_Task_STK[trigger_calc_runtime_STK_SIZE- 1], Trigger_CalcRuntime_Task_PRIO);

	OSTaskCreate(UARTMessageParse_Task, (void *)0, (OS_STK*)&PARSE_TASK_STK[PARSE_STK_SIZE- 1], PARSE_TASK_PRIO);

	OSTaskCreate(Track_Run_Task, (void *)0, (OS_STK*)&TRACK_TASK_STK[TRACK_STK_SIZE- 1], TRACK_TASK_PRIO);

	OSTaskCreate(Track_OverCurrent_Task, (void *)0, (OS_STK*)&OVERCURRENT_TASK_STK[OVERCURRENT_STK_SIZE- 1], OVERCURRENT_TASK_PRIO);

	OSTaskCreate(TrackMonitor_Task, (void *)0, (OS_STK*)&TrackMonitor_TASK_STK[TrackMonitor_STK_SIZE- 1], TrackMonitor_TASK_PRIO);

	//OSTaskCreate(Factory_Test_Task, (void *)0, (OS_STK*)&FACTORY_TEST_TASK_STK[FACTORY_TEST_STK_SIZE- 1], FACTORY_TEST_TASK_PRIO);

	if(g_src_board_id == 1)
	{
		UsartPrintf(USART_DEBUG, "g_src_board_id: %d\r\n", g_src_board_id);
		OSTaskCreate(DrugPush_Task, (void *)0, (OS_STK*)&Drug_Push_TASK_STK[Drug_Push_STK_SIZE- 1], Drug_Push_TASK_PRIO);
		OSTaskCreate(Message_Send_Task_HostBoard, (void *)0, (OS_STK*)&MSG_SEND_TASK_STK[MSG_SEND_STK_SIZE- 1], MSG_SEND_TASK_PRIO);
		OSTaskCreate(QueryMain_Task, (void *)0, (OS_STK*)&QUERY_TASK_STK[QUERY_STK_SIZE- 1], QUERY_TASK_PRIO);
	}	
	else
	{
		OSTaskCreate(Message_Send_Task, (void *)0, (OS_STK*)&MSG_SEND_TASK_STK[MSG_SEND_STK_SIZE- 1], MSG_SEND_TASK_PRIO);
		OSTaskCreate(CoolingControl_Task, (void *)0, (OS_STK*)&Cooling_TASK_STK[Cooling_STK_SIZE- 1], Cooling_TASK_PRIO);
	}
	
	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	
	OS_EXIT_CRITICAL();	//退出临界区(可以被中断打断)
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
	LED_STATUS_ENUM status = LED_ON;
	UsartPrintf(USART_DEBUG, "%s running!!!!!!!!!!\r\n", __FUNCTION__);

	while(1)
	{
		Iwdg_Feed(); 		//喂狗
		
		Led_Set(LED_1, status);
		status = !status;


		if(TrunkInitTime && (time_passes - TrunkInitTime > 600))
		{
			UsartPrintf(USART_DEBUG, "In 60s not receive finish cmd, clear!!!!\r\n", time_passes, TrunkInitTime, TrackPushAllTime);
			CleanTrackParam();
			track_work = MOTOR_STOP;
			TrunkInitTime = 0;
			TrackPushAllTime = 0;
			board_add_finish = 0;
			board_push_finish = 0;
		}
		else if(TrackPushAllTime && (time_passes - TrackPassTime > TrackPushAllTime + 1200))
		{
			UsartPrintf(USART_DEBUG, "Track run over 120s not(%d, %d, %d) finish, clear!!!!\r\n", time_passes, TrackPassTime, TrackPushAllTime);
			CleanTrackParam();
			track_work = MOTOR_STOP;
			TrunkInitTime = 0;
			TrackPushAllTime = 0;
		}
		
		RTOS_TimeDly(50);	//挂起任务250ms
	}
}

void UART1_RECEIVE_Task(void *pdata)
{
    INT8U            err;
	UsartPrintf(USART_DEBUG, "%s running!!!!!!!!!!\r\n", __FUNCTION__);
	SemOfUart1RecvData = OSSemCreate(0);
	while(1)
	{
		OSSemPend(SemOfUart1RecvData, 0u, &err);
		do{
			if(uart1_receive_data() == 0)
				break;
			RTOS_TimeDly(20);
		}while(1);
	}
}

void UARTMessageParse_Task(void *pdata)
{
    INT8U            err;
	UsartPrintf(USART_DEBUG, "%s running!!!!!!!!!!\r\n", __FUNCTION__);
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
	UsartPrintf(USART_DEBUG, "%s running!!!!!!!!!!\r\n", __FUNCTION__);
	SemOfUart2RecvData = OSSemCreate(0);
	while(1)
	{
		OSSemPend(SemOfUart2RecvData, 0u, &err);
		//UsartPrintf(USART_DEBUG, "UART2_RECEIVE_Task--------\r\n");
		
		do{
			if(uart2_receive_data() == 0)
				break;
			RTOS_TimeDly(20);
		}while(1);
	}
}

void RS485MessageParse_Task(void *pdata)
{
    INT8U            err;
	
	SemOf485DataParse = OSSemCreate(0);
	while(1)
	{
		OSSemPend(SemOf485DataParse, 0u, &err);

		
		//UsartPrintf(USART_DEBUG, "RS485MessageParse_Task--------\r\n");
		do{
			if(uart2_parse_protocol()== 0)
				break;
		}while(1);
	}
}



void HeartBeat_Task(void *pdata)
{
	int heart_count = 0;
	int have_run_time = 0;
	//uint16_t wait_time = 0;
	
	heart_count = g_src_board_id * 2 - 1;

	UsartPrintf(USART_DEBUG, "heart_count = %d\r\n", heart_count);
	
	RTOS_TimeDlyHMSM(0, 0, heart_count, 0);
	
	if(UartMsgNode == NULL)
	UartMsgNode = CreateNode();	
	
	heart_info.board_status = FIRSTBOOT_STATUS;
	board_send_message(STATUS_REPORT_REQUEST, &heart_info);
	heart_info.board_status = STANDBY_STATUS;
	UsartPrintf(USART_DEBUG, "%s running!!!!!!!!!!\r\n", __FUNCTION__);
	while(1)
	{
		TrackRunMonitor();
		RTOS_TimeDlyHMSM(0, 0, 0, 100);
		if(heart_count >= 600)
		{
			board_send_message(STATUS_REPORT_REQUEST, &heart_info);
			heart_count = 0;
			have_run_time++;
			UsartPrintf(USART_DEBUG, "mcu run %dh:%dmin!!!!!!!!!!!!!!!\r\n", have_run_time/60, have_run_time%60);
		}
		heart_count++;
	}
}
void Track_Run_Task(void *pdata)
{
    INT8U            err;
	UsartPrintf(USART_DEBUG, "%s running!!!!!!!!!!\r\n", __FUNCTION__);
	SemOfTrack = OSSemCreate(0);
	while(1)
	{
		OSSemPend(SemOfTrack, 0u, &err);		
		UsartPrintf(USART_DEBUG, "Run Track----------\r\n");		//提示任务开始执行
		
		Track_run(track_work);

		if(0)
		{
			Track_run_only(MOTOR_RUN_BACKWARD);
			CleanTrackParam();
		}
		
		track_work = MOTOR_STOP;
	}
	OSSemDel(SemOfTrack, 0, &err);
}

void DrugPush_Task(void *pdata)
{
	uint8_t delay_time = 10;
	uint16_t run_time = 0;
	INT8U            err;
	uint16_t push_time = 0;
	uint8_t drug_push_status = 0;
	INT8U try_times = 0;
	

	SemOfConveyor= OSSemCreate(0);

	/*上电启动传送带，回收药品*/
	#if 0
	UsartPrintf(USART_DEBUG, "Drug Collect Start!!!!!!!!!!\r\n");
	PushBeltControl(BELT_RUN);
	RTOS_TimeDlyHMSM(0, 0, 15, 0);
	PushBeltControl(BELT_STOP);
	Lifter_Set(LIFTER_UP);
	Collect_Belt_Run();
	Lifter_Set(LIFTER_FALL);
	UsartPrintf(USART_DEBUG, "Drug Collect End!!!!!!!!!!\r\n");
	#endif
	UsartPrintf(USART_DEBUG, "%s running!!!!!!!!!!\r\n", __FUNCTION__);
	
	while(1)
	{		
		OSSemPend(SemOfConveyor, 0u, &err);
		run_time = 0;
		push_time = GetMaxPushTime();
		drug_push_status = 0;

		
		PushBeltControl(BELT_RUN);
		do{
			UsartPrintf(USART_DEBUG, "board_push_finish = 0x%x, board_push_ackmsg = 0x%x, runtime = %d/%d!!!!!!!!!!\r\n", board_push_finish, board_push_ackmsg, run_time, push_time);
			if((board_push_finish == 0))// && (board_push_ackmsg == 0))
			{
				drug_push_status = 1;
				break;
			}

			RTOS_TimeDlyHMSM(0, 0, 1, 0);
			run_time ++;
			if((run_time >= push_time + 20) && (run_time > 120))// 最大运行时间+20S 后未出货完成开始回收
			{
				break;
			}
		}while(1);
		if(drug_push_status == 0)// 150s后未出货完成开始回收
		{
			UsartPrintf(USART_DEBUG, "Push Fail, Clean!!!!!!!!!!\r\n");
			PushBeltControl(BELT_STOP);
			Collect_Belt_Run();
			CleanTrackParam();
			
			board_add_finish = 0;
			board_push_finish = 0;
			continue;
		}

		UsartPrintf(USART_DEBUG, "Will run conveyor!!!!!!!!!!\r\n");
		//conveyor = Push_Belt_Check();		
		if(1)//(conveyor == 1)
		{
			run_time = 0;
			RTOS_TimeDlyHMSM(0, 0, BELT_RUN_TIME, 0);//传送带运行10s 时间
			PushBeltControl(BELT_STOP);
			
			if(1)//(Push_Belt_Run() != 0 )
			{
				Lifter_Set(LIFTER_UP);
				
				Door_Control_Set(MOTOR_RUN_BACKWARD);
				do{
					RTOS_TimeDlyHMSM(0, 0, 0, 100);
					run_time += 1;					
					//UsartPrintf(USART_DEBUG, "run_time = %d\r\n", run_time);
					if(run_time >= 300)
					break;
				}while(Door_Key_Detect(DOOR_OPEN) == SENSOR_NO_DETECT);
				
				mcu_push_medicine_open_door_complete();//所有单板出货完成,门已打开

				UsartPrintf(USART_DEBUG, "Open The Door, End!!!!!!!!!!\r\n");
				Door_Control_Set(MOTOR_STOP);

				RTOS_TimeDlyHMSM(0, 0, delay_time, 0);
				
				Door_Control_Set(MOTOR_RUN_FORWARD);
				run_time = 0;
				try_times = 0;
				do{
					if((Sensor_Detect() == SENSOR_DETECT)&&(try_times <= 10))
					{
						UsartPrintf(USART_DEBUG, "Close Door Detect Somebody, Stop!!!!!!!!!!\r\n");
						Door_Control_Set(MOTOR_STOP);
						RTOS_TimeDlyHMSM(0, 0, 10, 0);
						try_times++;
						Door_Control_Set(MOTOR_RUN_FORWARD);
					}
					else
					{
						run_time += 1;
					}
					RTOS_TimeDlyHMSM(0, 0, 0, 100);
					
					if(run_time >= 300)
					{
						UsartPrintf(USART_DEBUG, "run_time %d > 30s\r\n", run_time);
						break;
					}
				}while(Door_Key_Detect(DOOR_CLOSE) == SENSOR_NO_DETECT);
				
				Door_Control_Set(MOTOR_STOP);
				mcu_push_medicine_close_door_complete();
				UsartPrintf(USART_DEBUG, "Close The Door, End!!!!!!!!!!\r\n");
				
				Collect_Belt_Run();
				Lifter_Set(LIFTER_FALL);
			}
		}
	}
}



/*
产测任务
硬件基本功能测试
0、串口测试:由uart2发送消息给uart1
1、货道前进后退
2、传送带
3、开关门

02 09 50 id 80 00 00 00 00 DF
*/

void Factory_Test_Task(void *pdata)
{	
	int test_time = 0;
	uint8_t track = 0;
	
	uint8_t msg[BOARD_TEST_REQUEST_PACKET_SIZE] = {0x02,0x09,0x50,0xFF,0x80,00,00,00,00,0x74};
	
	//uint8_t msg4track[TRACK_RUNTIME_CALC_REQUEST_PACKET_SIZE] = {0x02, 0x06, 0x80, 0x01, 0x01, 0x08, 0x92};

    INT8U            err;
	
	UsartPrintf(USART_DEBUG, "%s running!!!!!!!!!!\r\n", __FUNCTION__);
	SemOfFactoryTest = OSSemCreate(0);
	
	while(1)
	{
		msg[3] = g_src_board_id;
		msg[BOARD_TEST_REQUEST_PACKET_SIZE - 1] = add_checksum(msg, BOARD_TEST_REQUEST_PACKET_SIZE - 1);

		UsartPrintf(USART_DEBUG, "Send factory test cmd!!!!!\r\n");

		
		RTOS_TimeDlyHMSM(0, 0, 1, 00);
		/*产测流程*/
		//UART2_IO_Send(msg, BOARD_TEST_REQUEST_PACKET_SIZE);	
		RS485_Send_Data(msg, BOARD_TEST_REQUEST_PACKET_SIZE);

		/*货道调试*/
		//msg4track[3] = g_src_board_id;
		//msg4track[TRACK_RUNTIME_CALC_REQUEST_PACKET_SIZE - 1] = add_checksum(msg4track, TRACK_RUNTIME_CALC_REQUEST_PACKET_SIZE - 1);
		//UART2_IO_Send(msg4track, TRACK_RUNTIME_CALC_REQUEST_PACKET_SIZE);	
		
		RTOS_TimeDlyHMSM(0, 0, 0, 50);
		
		#if 0
		for(track = 0; track <= 1000; track++)
		{
			msg[0] = track;
			RS485_Send_Data(msg, BOARD_TEST_REQUEST_PACKET_SIZE);
			RTOS_TimeDlyHMSM(0, 0, 5, 0);
		}
		#endif
		
		OSSemPend(SemOfFactoryTest, 0u, &err);
		//if(KeyScan(GPIOB, ForwardDetectKey) == KEYDOWN)
		{
			UsartPrintf(USART_DEBUG, "Facroty test %d, ", test_time);
			FactoryFuncTest();
			
			UsartPrintf(USART_DEBUG, "Facroty Track test\r\n");

			#if 0
			for(track = 1; track <= TRACK_MAX; track++)
			{
				UsartPrintf(USART_DEBUG, "track - %d\r\n", track);
				
				SetTrackTestTime(track, MOTOR_RUN_FORWARD, 300);
				Track_run_only(MOTOR_RUN_FORWARD);
				
				SetTrackTestTime(track, MOTOR_RUN_BACKWARD, 300);
				Track_run_only(MOTOR_RUN_BACKWARD);
				
				RTOS_TimeDlyHMSM(0, 0, 1, 0);
			}
			#else
			calc_track_start_idx = 0;
			calc_track_count = TRACK_MAX;
			OSSemPost(SemOfCalcTime);
			#endif
		}
		
		test_time ++;
		RTOS_TimeDlyHMSM(0, 0, 10, 0);
	}
}




extern uint16_t running_time;


void Trigger_CalcRuntime_Task(void *pdata)
{
	INT8U			 err;
	uint8_t i = 0;
	UsartPrintf(USART_DEBUG, "%s running!!!!!!!!!!\r\n", __FUNCTION__);
	SemOfCalcTime = OSSemCreate(0);
	while(1)
	{
		OSSemPend(SemOfCalcTime, 0u, &err);
		UsartPrintf(USART_DEBUG, "Trigger_CalcRuntime_Task run!!!!!!!!!!!!\r\n");
		trigger_calc_flag = 0;
		motor_run_detect_flag = 0;
		key_init = 1;
		
		for(cur_calc_track = calc_track_start_idx; cur_calc_track < calc_track_count + calc_track_start_idx; cur_calc_track ++)
		{
			UsartPrintf(USART_DEBUG, "cur_calc_track :%d, calc_track_count :%d\r\n", cur_calc_track, calc_track_count);
			RTOS_TimeDlyHMSM(0, 0, 2, 0);	//延时2s
			
			for( i = 0; i < 4; i++)
			{
				if(i == 0)
				{
					trigger_calc_runtime = 0;	//清计时
					trigger_calc_flag = 0;		//关中断
					UsartPrintf(USART_DEBUG, "Track[%d], do prepare\r\n", cur_calc_track);

					running_time = 0;
					trigger_calc_runtime = 1;
					Track_trigger_calc_runtime(1, MOTOR_RUN_FORWARD);
					RTOS_TimeDlyHMSM(0, 0, 0, KEY_DELAY_500MS * 100);
					trigger_calc_flag = 1;//开中断，进循环
					key_stat = 0;
				}
				else if(i == 1)
				{	
					UsartPrintf(USART_DEBUG, "Track[%d], do backward\r\n", cur_calc_track);
					
					trigger_calc_runtime = 0;	//清计时
					trigger_calc_flag = 0;		//关中断
					RTOS_TimeDlyHMSM(0, 0, 1, 0);	//延时1S
					
					running_time = 0;
					trigger_calc_runtime = 1;
					Track_trigger_calc_runtime(0, MOTOR_RUN_BACKWARD);
					RTOS_TimeDlyHMSM(0, 0, 0, KEY_DELAY_500MS * 100);
					trigger_calc_flag = 1;		//开中断，进循环
					key_stat = 1;
				}
				else if(i == 2)
				{
					UsartPrintf(USART_DEBUG, "Track[%d], do forward\r\n", cur_calc_track);
					trigger_calc_runtime = 0;	//清计时
					trigger_calc_flag = 0;		//关中断
					RTOS_TimeDlyHMSM(0, 0, 1, 0);	//延时1S

					running_time = 0;
					trigger_calc_runtime = 1;
					Track_trigger_calc_runtime(0, MOTOR_RUN_FORWARD);
					RTOS_TimeDlyHMSM(0, 0, 0, KEY_DELAY_500MS * 100);
					trigger_calc_flag = 1;
					key_stat = 2;
				}
				else if(i == 3)
				{
					UsartPrintf(USART_DEBUG, "Track[%d], do little backword00\r\n", cur_calc_track);

					trigger_calc_flag = 0;
					trigger_calc_runtime = 0;
					
					/*货道第二次运行到头部，回退1S*/
					Track_trigger(cur_calc_track, MOTOR_RUN_BACKWARD);
					RTOS_TimeDlyHMSM(0, 0, 0, TRACK_BACK_TIME);
					Track_trigger(cur_calc_track, MOTOR_STOP);
					
					UsartPrintf(USART_DEBUG, "Track[%d], do little backword11\r\n", cur_calc_track);
				}
				
				do{
					if((i == 0)&&(Key_Check(ForwardDetectKey) || Key_Check(CurrentDetectKey)))
					{
						Track_Runtime(1, MOTOR_STOP, MOTOR_RUN_FORWARD);
						trigger_calc_flag = 0;
						UsartPrintf(USART_DEBUG, "Forward Key Block!!!!\r\n");
						break;
					}	
					else if((i == 0)&&Key_Check(BackwardDetectKey))
					{
						trigger_calc_flag = 0;
						running_time = TRACK_MAX_TIME_MS + 100;
						UsartPrintf(USART_DEBUG, "Motor Reverse!!!!\r\n");
						break;
					}	
					else if((i == 1)&&(Key_Check(BackwardDetectKey)||Key_Check(CurrentDetectKey)))
					{
						Track_Runtime(0, MOTOR_STOP, MOTOR_RUN_BACKWARD);
						trigger_calc_flag = 0;
						UsartPrintf(USART_DEBUG, "Backword Key Block!!!!\r\n");
						break;
					}
					else if((i == 2)&&(Key_Check(ForwardDetectKey)||Key_Check(CurrentDetectKey)))
					{
						Track_Runtime(0, MOTOR_STOP, MOTOR_RUN_FORWARD);
						trigger_calc_flag = 0;
						UsartPrintf(USART_DEBUG, "Little Forward Key Block!!!!\r\n");
						break;
					}
					else if(i == 3)
					{
					
					}
					
					if(running_time >= TRACK_MAX_TIME_MS)
					{
						break;
					}
					RTOS_TimeDlyHMSM(0, 0, 0, 100);
				}while(trigger_calc_flag);

				key_stat = 0;
				UsartPrintf(USART_DEBUG, "running_time = %d!!!!\r\n", running_time);
				if(running_time >= TRACK_MAX_TIME_MS)
				{
					running_time = 0;
					trigger_calc_runtime = 0;
					Track_trigger_calc_runtime_error(0, running_time, MOTOR_STOP);
		
					UsartPrintf(USART_DEBUG, "Track calc time %d longer than 80s, error!!!!\r\n", running_time);
					break;
				}
			}
		}
		trigger_calc_flag = 0;
	}
	OSSemDel(SemOfCalcTime, 0, &err);
}

void Track_OverCurrent_Task(void *pdata)
{
    INT8U            err;
	
	UsartPrintf(USART_DEBUG, "%s running!!!!!!!!!!\r\n", __FUNCTION__);
	SemOfOverCurrent = OSSemCreate(0);
	while(1)
	{
		OSSemPend(SemOfOverCurrent, 0u, &err);
		
		motor_run_detect_flag = 0;
		
		UsartPrintf(USART_DEBUG, "Track[%d],dir[%d] do OverCurrent protect!!!\r\n", motor_run_detect_track_num, motor_run_direction);
		
		if(motor_run_direction == MOTOR_RUN_BACKWARD)
		{
			Motor_Set(MOTOR_RUN_FORWARD);
			set_track(motor_run_detect_track_num, MOTOR_RUN_FORWARD);
		}
		else if(motor_run_direction == MOTOR_RUN_FORWARD)
		{
			Motor_Set(MOTOR_RUN_BACKWARD);
			set_track(motor_run_detect_track_num, MOTOR_RUN_BACKWARD);
		}
		RTOS_TimeDlyHMSM(0, 0, 0, TRACK_BACK_TIME);
		
		set_track(motor_run_detect_track_num, MOTOR_STOP);//货道停止
		Motor_Set(MOTOR_STOP);
		
	}
	OSSemDel(SemOfOverCurrent, 0, &err);
}


void Message_Send_Task(void *pdata)
{		
	uint16_t node_num = 0;
	uint32_t cur_time = 0;
	uint8_t err = 0;
	uint8_t i = 0, node_i = 0;
	
	struct node* MsgNode = NULL;
	struct node* NewMsgNode = NULL;


	UsartPrintf(USART_DEBUG, "%s running!!!!!!!!!!\r\n", __FUNCTION__);
	
	SemOf485MsgSend = OSSemCreate(0);
	while(1)
	{
		OSSemPend(SemOf485MsgSend, 0u, &err);
		
		//请求信号量
		OSMutexPend(MsgMutex,0,&err);

		cur_time = time_passes;
		/*消息队列取消息*/
		node_num = GetNodeNum(UartMsgNode);
		//UsartPrintf(USART_DEBUG, "node_num[%d]cur_time[%d]!!!\r\n", node_num, cur_time);

		MsgNode = UartMsgNode;
		for(i = 1, node_i = 1; i <= node_num; i++)
		{
			//NewMsgNode = GetMsgNode(MsgNode);
			NewMsgNode = GetNode(UartMsgNode, node_i);	
			UsartPrintf(USART_DEBUG, "Message_Send_Task Node[%d/%d]node_p[%d]!!!\r\n", i, node_num, node_i);
			
			if(NewMsgNode)
			{
				if(NewMsgNode->data.times == 0xff)
				{
					/*立即发送*/
					RS485_Send_Data(NewMsgNode->data.payload, NewMsgNode->data.size); 
					UsartPrintf(USART_DEBUG, "NewMsgNode data.size[%d]time[%d]times[%d]!!!\r\n", 
					NewMsgNode->data.size, NewMsgNode->data.create_time,NewMsgNode->data.times);
					NewMsgNode->data.times = 5;
				}
				else 
				{
					if((NewMsgNode->data.times == 0) || 
						(cur_time >= NewMsgNode->data.create_time + (NewMsgNode->data.times + 1) * 20))//超时20*100ms,重传
					{
						RS485_Send_Data(NewMsgNode->data.payload, NewMsgNode->data.size); 
						UsartPrintf(USART_DEBUG, "NewMsgNode data.size[%d]time[%d]times[%d]!!!\r\n", 
						NewMsgNode->data.size, NewMsgNode->data.create_time,NewMsgNode->data.times);
						NewMsgNode->data.times++;
					}
				}
				
				if(NewMsgNode->data.times >= 5 && NewMsgNode->data.times != 0xff)//设定最大重传次数
				{
					UsartPrintf(USART_DEBUG, "Retry Send TimeOut, Delete Node[%d]\r\n", i);
					if(i == node_num)
					DeleNode(MsgNode, TAIL);
					else
					DeleNode(MsgNode, i);
					NewMsgNode = NULL;
				}
				else
				{
					MsgNode = NewMsgNode;
					node_i++;
				}
			}
			RTOS_TimeDlyHMSM(0, 0, 0, 20);//两个包间隔20ms发送
		}
		//释放信号量
		OSMutexPost(MsgMutex);
	}

	DeleNode(MsgNode, 0);
}


void Message_Send_Task_HostBoard(void *pdata)
{		
	uint16_t node_num = 0;
	uint32_t cur_time = 0;
	uint8_t err = 0;
	uint8_t i = 0, j = 0, node_i = 0;
	
	struct node* MsgNode = NULL;
	struct node* NewMsgNode = NULL;

	if(UartMsgNode == NULL)
	UartMsgNode = CreateNode();

	UsartPrintf(USART_DEBUG, "%s running!!!!!!!!!!\r\n", __FUNCTION__);
	
	while(1)
	{
		//请求信号量
		OSMutexPend(MsgMutex,0,&err);

		cur_time = time_passes;
		/*消息队列取消息*/
		node_num = GetNodeNum(UartMsgNode);
		//UsartPrintf(USART_DEBUG, "node_num[%d]cur_time[%d]!!!\r\n", node_num, cur_time);

		MsgNode = UartMsgNode;
		for(i = 1, node_i = 1; i <= node_num; i++)
		{
			//NewMsgNode = GetMsgNode(MsgNode);
			NewMsgNode = GetNode(UartMsgNode, node_i);	
			UsartPrintf(USART_DEBUG, "Message_Send_Task Node[%d/%d]node_p[%d]!!!\r\n", i, node_num, node_i);
			
			if(NewMsgNode)
			{
				UsartPrintf(USART_DEBUG, "NewMsgNode data.size[%d]time[%d]times[%d]!!!\r\n", 
					NewMsgNode->data.size, NewMsgNode->data.create_time,NewMsgNode->data.times);

				if((NewMsgNode->data.times == 0) || 
					(cur_time >= NewMsgNode->data.create_time + (NewMsgNode->data.times + 1) * 20))//超时20*100ms,重传
				{
					for(j = 0; j < NewMsgNode->data.size; j++)
					{
						UsartPrintf(USART_DEBUG, "0x%02x,", NewMsgNode->data.payload[j]);
					}
					UsartPrintf(USART_DEBUG, "<--Retry Send Message\r\n");
					
					if(NewMsgNode->data.uart_idx == UART1_IDX)
					{
						UART1_IO_Send(NewMsgNode->data.payload, NewMsgNode->data.size); 
					}
					else
					{
						RS485_Send_Data(NewMsgNode->data.payload, NewMsgNode->data.size); 
					}
					NewMsgNode->data.times++;
				}
				
				if(NewMsgNode->data.times >= 5)//设定最大重传次数
				{
					UsartPrintf(USART_DEBUG, "Retry Send TimeOut, Delete Node[%d]\r\n", i);
					if(i == node_num)
					DeleNode(MsgNode, TAIL);
					else
					DeleNode(MsgNode, i);
					NewMsgNode = NULL;
				}
				else
				{
					MsgNode = NewMsgNode;
					node_i++;
				}
			}
			//RTOS_TimeDlyHMSM(0, 0, 0, MSG_RESEND_TIME_SLEEP_100MS);//两个包间隔100ms发送
		}
		//释放信号量
		OSMutexPost(MsgMutex);
		
		RTOS_TimeDlyHMSM(0, 0, 2, 0);//等待500ms，重传
	}

	DeleNode(MsgNode, 0);
}







/*
void QueryMain_Task(void *pdata)
{
	uint8_t loop_id = 0;
	uint8_t msg[QUERY_REQUEST_PACKET_SIZE] = {0x02,0x04,0xF1,0x01,0x00};
	int i = 0;
	
	while(1)
	{	
		for(loop_id = 0; loop_id < BOARD_ID_MAX; loop_id++)
		{
			msg[3] = loop_id;
			msg[QUERY_REQUEST_PACKET_SIZE - 1] = add_checksum(msg, QUERY_REQUEST_PACKET_SIZE - 1);
			RS485_Send_Data(msg, QUERY_REQUEST_PACKET_SIZE);
			RTOS_TimeDlyHMSM(0, 0, 0, 150);	//挂起任务150ms
		}
	}
}
*/
	
void QueryMain_Task(void *pdata)
{
	uint8_t id = 0;
	
	UsartPrintf(USART_DEBUG, "%s running!!!!!!!!!!\r\n", __FUNCTION__);

	while(1)
	{	
		for(id = 2; id <= BOARD_ID_MAX; id++)
		{
			send_query_message(id);
			RTOS_TimeDlyHMSM(0, 0, 0, 250); //挂起任务250ms
		}
	}
}


void TrackMonitor_Task(void *pdata)
{
	float adcx;
	float voltage;
	static float g_standby_voltage = 0;
	static float g_standby_adcx = 0;
	
	uint8_t status = 0;
	uint8_t track_id = 0;
	uint8_t is_report = 0;
	int i = 0;
	
	static LED_STATUS_ENUM led_st = LED_ON;
	
	UsartPrintf(USART_DEBUG, "SENSOR_Task run!!!!!!!!!!!!\r\n");

	while(1)
	{	
		is_report = 0;
		voltage =0;

		if(g_standby_voltage == 0)
		{
			voltage = 0;
			for(i = 0; i < 3; i++)
			{
				adcx = (float)Get_Adc_Average();
				voltage = adcx*(3.3/4096);
				
				g_standby_adcx += adcx;
				g_standby_voltage += voltage;
				
				RTOS_TimeDlyHMSM(0, 0, 0, 500);
				UsartPrintf(USART_DEBUG, "First Boot voltage:%.3f[%.3f]\r\n", voltage, adcx);
			}
			
			g_standby_adcx = g_standby_adcx/3;
			g_standby_voltage = g_standby_voltage/3;
			
			UsartPrintf(USART_DEBUG, "First Boot g_standby_voltage:%.3f[%.3f]\r\n", g_standby_voltage, g_standby_adcx);
			
			RTOS_TimeDlyHMSM(0, 0, 1, 0);
		}
		else
		{			
			//UsartPrintf(USART_DEBUG, "%s:%d. %d\r\n", __FUNCTION__, g_track_state, motor_run_detect_flag);
			if ((g_track_state == TRACK_STANDBY) && (motor_run_detect_flag == 0)&&(g_track_id == 0)) 
			{
				RTOS_TimeDlyHMSM(0, 0, 0, 500);
				for(i = 0; i < 2; i++)
				{
					is_report = 0;
					adcx = Get_Adc_Average();			
					voltage += adcx*(3.3/4096);
					
					RTOS_TimeDlyHMSM(0, 0, 0, 500);
				}
				if((g_track_state != TRACK_STANDBY) || (motor_run_detect_flag != 0) || (g_track_id != 0))
				{
					UsartPrintf(USART_DEBUG, "Track Status Change!!!!!!\r\n");
					goto NEXT_SETP;
				}
				
				voltage = voltage/2;
				UsartPrintf(USART_DEBUG, "TRACK_STANDBY voltage:%.3f[%.3f]\r\n", voltage, adcx);
				if(voltage >= g_standby_voltage + 0.3)//g_standby_voltage + 0.3，单板短路故障
				{
					is_report = 1;
					track_id = 0xff;
					status = SHORTCIRCUIT_BLOCK;
					UsartPrintf(USART_DEBUG, "TRACK_STANDBY voltage[%.3f] > [%.3f]\r\n", voltage, g_standby_voltage + 0.3);
				}
				Led_Set(LED_2, LED_OFF);
			}
			else if((g_track_state == TRACK_WORKING) && (motor_run_detect_flag == 1) && (g_track_id != 0))
			{
				for(i = 0; i < 2; i++)
				{
					is_report = 0;
					adcx = Get_Adc_Average();			
					voltage += adcx*(3.3/4096);
					
					RTOS_TimeDlyHMSM(0, 0, 0, 500);
				}
				if((g_track_state != TRACK_WORKING) || (motor_run_detect_flag != 1) || (g_track_id == 0))
				{
					UsartPrintf(USART_DEBUG, "Track Status Change!!!!!!\r\n");
					goto NEXT_SETP;
				}
				
				voltage = voltage/2;
				UsartPrintf(USART_DEBUG, "TRACK_WORKING adcx:%.3f[%.3f]\r\n", voltage, adcx);
				
				if(voltage < g_standby_voltage + 0.03)//< g_standby_voltage + 0.03，断路
				{
					status = BROKENCIRCUIT;
					is_report = 1;
					track_id = g_track_id;
					UsartPrintf(USART_DEBUG, "TRACK_WORKING BROKENCIRCUIT voltage[%.3f] < [%.3f]\r\n", voltage, g_standby_voltage + 0.03);
				}	
				else if(voltage > g_standby_voltage + 0.3)// > g_standby_voltage + 0.5，堵转
				{
					status = SHORTCIRCUIT_BLOCK;
					is_report = 1;
					track_id = g_track_id;
					UsartPrintf(USART_DEBUG, "TRACK_WORKING SHORTCIRCUIT_BLOCK voltage[%.3f] > [%.3f]\r\n", voltage, g_standby_voltage + 0.3);
					Track_trigger(track_id, MOTOR_STOP);
				}
				
				Led_Set(LED_2, led_st);
				led_st = !led_st;		
			}

			NEXT_SETP:
			if(is_report)
			send_track_status_report(track_id, status);
		}
		
	}
}


void CoolingControl_Task(void *pdata)
{
	int temperature = 0;  	    
	int humidity = 0;
	
	UsartPrintf(USART_DEBUG, "%s running!!!!!!!!!!\r\n", __FUNCTION__);
	
	while(1)
	{	
		RTOS_TimeDlyHMSM(0, 0, 10, 0);
		if(DHT12_READ(&temperature, &humidity) == 0)
		{
			send_temperature_report(temperature, humidity);
			UsartPrintf(USART_DEBUG, "temperature:%0.1f, humidity:%0.1f\r\n", (float)temperature/10, (float)humidity/10);

			if(temperature > SHADE_AREA_TEMPERATURE_MAX)
			{
				UsartPrintf(USART_DEBUG, "temperature:%d > %d", temperature , SHADE_AREA_TEMPERATURE_MAX);
				Coolingcompressor_Set(COOLING_ON);
			}
			else if(temperature < SHADE_AREA_TEMPERATURE_MIN)
			{
				UsartPrintf(USART_DEBUG, "temperature:%d < %d", temperature , SHADE_AREA_TEMPERATURE_MIN);
				Coolingcompressor_Set(COOLING_OFF);
			}
		}
	}
}



