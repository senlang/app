/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	track.c
	*
	*	作者： 		
	*
	*	日期： 		2018-10-10
	*
	*	版本： 		V1.0
	*
	*	说明： 		货道使能控制
	*
	*	修改记录：	
	************************************************************
	************************************************************
	************************************************************
**/

//单片机头文件
#include "stm32f10x.h"

//头文件
#include "track.h"
#include "delay.h"
#include "usart.h"
#include "key.h"
#include "stm32_protocol.h"

extern uint8_t  g_src_board_id;  
extern OS_EVENT *SemOfConveyor;        	//Motor控制信号量
extern struct track_work_struct track_struct[10][10];
extern struct status_report_request_info_struct  heart_info;
extern uint8_t cur_calc_track;
extern uint16_t running_time;

extern uint8_t motor_run_detect_flag;
extern uint8_t motor_run_detect_track_num;
extern uint16_t board_push_finish;
extern uint16_t board_add_finish;
extern uint8_t OverCurrentDetected;	//货道开关状态1为检测到

extern uint8_t g_track_state;
extern uint8_t g_track_id;

//static  uint16_t forward_running_time;  
//static  uint16_t backward_running_time; 

static struct track_cale_report_info_struct track_time;



track_elem X_value[10] = {

	{GPIOB, GPIO_Pin_12},
	{GPIOB, GPIO_Pin_13},

	{GPIOB, GPIO_Pin_14},
	{GPIOB, GPIO_Pin_15},	

	{GPIOD, GPIO_Pin_8},
	{GPIOD, GPIO_Pin_9},

	{GPIOD, GPIO_Pin_10},
	{GPIOD, GPIO_Pin_11},

	{GPIOD, GPIO_Pin_12},
	{GPIOD, GPIO_Pin_13}
};


track_elem Y_value[10] = {

	{GPIOD, GPIO_Pin_14},
	{GPIOD, GPIO_Pin_15},

	{GPIOC, GPIO_Pin_6},
	{GPIOC, GPIO_Pin_7},	

	{GPIOC, GPIO_Pin_8},
	{GPIOC, GPIO_Pin_9},

	{GPIOC, GPIO_Pin_10},
	{GPIOC, GPIO_Pin_11},

	{GPIOC, GPIO_Pin_12},
	{GPIOD, GPIO_Pin_2}
};



void Track_Init(void)
{
	
	GPIO_InitTypeDef gpioInitStrcut;
	int i = 0;

	//使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
	
	//IO配置
	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	//IO初始化
	GPIO_Init(GPIOB, &gpioInitStrcut);




	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &gpioInitStrcut);
	

	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &gpioInitStrcut);
	

	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_2;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &gpioInitStrcut);


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);				//禁止JTAG功能


	for(i = 0; i < 10; i++)
	{
		//for(j = 0; j < 10; j++)
		{
			GPIO_WriteBit(X_value[i].GPIOx, X_value[i].GPIO_Pin, Bit_RESET);
			GPIO_WriteBit(Y_value[i].GPIOx, Y_value[i].GPIO_Pin, Bit_RESET);
		}
	}
}

uint8_t get_track_addr(uint16_t track_num, track_addr *addr)
{
	addr->x = (track_num - 1)/10;
	addr->y = (track_num - 1)%10;
	
	//UsartPrintf(USART_DEBUG, "track_num[%d]addr->x[%d]addr->y[%d]\r\n", track_num, addr->x, addr->y);
	return 1;
}


uint8_t set_track(uint16_t track_num, uint8_t status)
{
	track_elem x;
	track_elem y;
	track_addr addr;

	if(track_num == 255)
	return 0;
	
	get_track_addr(track_num, &addr);
	
	x = X_value[addr.x];
	y = Y_value[addr.y];

	//UsartPrintf(USART_DEBUG, "x.GPIO_Pin[0x%04x]y.GPIO_Pin[0x%04x]\r\n", x.GPIO_Pin, y.GPIO_Pin);
	
	UsartPrintf(USART_DEBUG, "set_track[%d]:%d\r\n", track_num, status);
	
	if(0 == status)
	{
		GPIO_WriteBit(x.GPIOx, x.GPIO_Pin, Bit_RESET);
		GPIO_WriteBit(y.GPIOx, y.GPIO_Pin, Bit_RESET);
		g_track_id = 0;
	}
	else if((1 == status) || (2== status))
	{
		GPIO_WriteBit(x.GPIOx, x.GPIO_Pin, Bit_SET);
		GPIO_WriteBit(y.GPIOx, y.GPIO_Pin, Bit_SET);
		g_track_id = track_num;
	}

	return 1;
}


uint8_t set_track_x(uint8_t row, uint8_t status)
{
	track_elem x;	
	x = X_value[row];

	//UsartPrintf(USART_DEBUG, "x.GPIO_Pin[0x%04x]y.GPIO_Pin[0x%04x]\r\n", x.GPIO_Pin, y.GPIO_Pin);
	if(0 == status)
	{
		GPIO_WriteBit(x.GPIOx, x.GPIO_Pin, Bit_RESET);
	}
	else if((1 == status) || (2== status))
	{
		GPIO_WriteBit(x.GPIOx, x.GPIO_Pin, Bit_SET);
	}

	return 1;
}



uint8_t set_track_y(uint16_t col, uint8_t status)
{
	track_elem y;

	y = Y_value[col];
	if(0 == MOTOR_STOP)
	{
		GPIO_WriteBit(y.GPIOx, y.GPIO_Pin, Bit_RESET);
	}
	else if((1 == MOTOR_RUN_FORWARD) || (2== MOTOR_RUN_BACKWARD))
	{
		GPIO_WriteBit(y.GPIOx, y.GPIO_Pin, Bit_SET);
	}

	return 1;
}

#if 0
int Track_run(MOTOR_ENUM run_mode)
{
	int x = 0, y = 0;
	uint16_t time_elapsed = 0;
	uint16_t all_finish = 0;

	UsartPrintf(USART_DEBUG, "Enter Track_run!!!\r\n");
	Motor_Set(run_mode);
	for(x = 0; x < 10; x++)
	{
		//UsartPrintf(USART_DEBUG, "Enter Track_run x = %d!!!\r\n", x);
		set_track_x(x, run_mode);
		for(y = 0; y < 10; y++)
		{
			if(track_struct[x][y].push_time > 0)			
			{
				UsartPrintf(USART_DEBUG, "track_struct[%d] push_time: %d\r\n", x*10 + y, track_struct[x][y].push_time);
				set_track_y(y, run_mode);
			}
		}

		all_finish = 0;
		do{
			for(y = 0; y < 10; y++)
			{
				//UsartPrintf(USART_DEBUG, "time_elapsed = %d, track_struct[%d][%d].push_time = %d\r\n", time_elapsed, x, y, track_struct[x][y].push_time);
				if(track_struct[x][y].push_time)
				{
					if(time_elapsed >= track_struct[x][y].push_time)
					{
						UsartPrintf(USART_DEBUG, "track %d stop!!!!\r\n", x*10 + y);
						set_track_y(y, MOTOR_STOP);
						all_finish &= ~(1<<y);
					}
					else
					{
						all_finish |= 1<<y;
					}
				}
				//UsartPrintf(USART_DEBUG, "all_finish = 0x%04x\r\n", all_finish);
			}
			
			if(all_finish == 0)
			break;
			
			time_elapsed += 2;
			RTOS_TimeDlyHMSM(0, 0, 0, 200);
		}while(1);

		time_elapsed = 0;
		set_track_x(x, MOTOR_STOP);
	}
	
	Motor_Set(MOTOR_STOP);	
	memset(track_struct, 0x00, sizeof(struct track_work_struct) * 10 * 10);

	if(MOTOR_RUN_FORWARD == run_mode)
	OSSemPost(SemOfConveyor);

	UsartPrintf(USART_DEBUG, "Exit Track_run!!!\r\n");
	return 0;
}

#else
void CleanTrackParam(void)
{
	int i = 0;
	
	for(i = 1; i <= TRACK_MAX; i++)
	{
		set_track(i, MOTOR_STOP);//货道停止
	}
	Motor_Set(MOTOR_STOP);	//电机停止
	
	memset(track_struct, 0x00, sizeof(struct track_work_struct) * 10 * 10);
}


int Track_run(MOTOR_ENUM run_mode)
{
	int x = 0, y = 0;
	uint16_t delay_s = 0;
	uint16_t delay_ms = 0;
	//struct push_medicine_complete_request_info_struct  push_complete_info;

	UsartPrintf(USART_DEBUG, "Enter Track_run, mode[%d]!!!\r\n", run_mode);
	for(x = 0; x < 10; x++)
	{
		for(y = 0; y < 10; y++)
		{
			if(track_struct[x][y].push_time > KEY_DELAY_MS)
			{
				delay_s = (track_struct[x][y].push_time - KEY_DELAY_MS)/10;
				delay_ms = ((track_struct[x][y].push_time - KEY_DELAY_MS) % 10) * 100;
				
				motor_run_detect_track_num = x*10 + y + 1;	
				UsartPrintf(USART_DEBUG, "start:track[%d]mode[%d]time[%d]=>%ds.%dms\r\n", motor_run_detect_track_num, track_struct[x][y].motor_run, track_struct[x][y].push_time, delay_s, delay_ms);
				
				Motor_Set(track_struct[x][y].motor_run);//电机方向使能
				set_track(motor_run_detect_track_num, track_struct[x][y].motor_run);//货道使能
				
				RTOS_TimeDlyHMSM(0, 0, 0, KEY_DELAY_MS * 100);
				
				motor_run_detect_flag = 1;
				OverCurrentDetected = 0;
				
				RTOS_TimeDlyHMSM(0, 0, delay_s, delay_ms);
				
				motor_run_detect_flag = 0;
				
				UsartPrintf(USART_DEBUG, "stop:track[%d]mode[%d]time[%d]=>%ds.%dms\r\n", motor_run_detect_track_num, track_struct[x][y].motor_run, track_struct[x][y].push_time, delay_s, delay_ms);

				/*发生开关检测到，延时1.5S，预留时间给回退*/
				if(OverCurrentDetected)
				RTOS_TimeDlyHMSM(0, 0, 1, 500);
				
				set_track(motor_run_detect_track_num, MOTOR_STOP);//货道停止
				Motor_Set(MOTOR_STOP);	//电机停止
				OverCurrentDetected = 0;

				//if(g_src_board_id != 1)
				{
					/*暂时注释，方便消息应答处理*/
					if(MOTOR_RUN_FORWARD == run_mode)
					{
						mcu_push_medicine_track_only(g_src_board_id, motor_run_detect_track_num);
					}
					else if(MOTOR_RUN_BACKWARD == run_mode)
					{
						mcu_add_medicine_track_only(g_src_board_id, motor_run_detect_track_num);
					}
				}

				#if 1
				RTOS_TimeDlyHMSM(0, 0, 0, 500);
				if((Key_Check(ForwardDetectKey) == KEYDOWN)||
				((MOTOR_RUN_FORWARD == run_mode)&&(Key_Check(CurrentDetectKey) == KEYDOWN)))//行程开关检测到到达货道头
				{
					UsartPrintf(USART_DEBUG, "Forward detect keep down, track[%d]\r\n", motor_run_detect_track_num);

					Motor_Set(MOTOR_RUN_BACKWARD);//电机方向使能
					set_track(motor_run_detect_track_num, MOTOR_RUN_BACKWARD);//货道使能
					
					RTOS_TimeDlyHMSM(0, 0, 1, 0);
					
					set_track(motor_run_detect_track_num, MOTOR_STOP);//货道停止
					Motor_Set(MOTOR_STOP);	//电机停止
				}
				else if((Key_Check(BackwardDetectKey) == KEYDOWN)||
					((MOTOR_RUN_BACKWARD == run_mode)&&(Key_Check(CurrentDetectKey) == KEYDOWN)))//行程开关检测到到达货尾
				{
					UsartPrintf(USART_DEBUG, "Backword detect keep down, track[%d]\r\n", motor_run_detect_track_num);
					Motor_Set(MOTOR_RUN_FORWARD);//电机方向使能
					set_track(motor_run_detect_track_num, MOTOR_RUN_BACKWARD);//货道使能
					
					RTOS_TimeDlyHMSM(0, 0, 1, 0);
					
					set_track(motor_run_detect_track_num, MOTOR_STOP);//货道停止
					Motor_Set(MOTOR_STOP);	//电机停止
				}
				#endif
			}
		}
		
	}


	for(x = 1; x <= TRACK_MAX; x++)
	{
		set_track(x, MOTOR_STOP);//货道停止
	}
	Motor_Set(MOTOR_STOP);	//电机停止
	memset(track_struct, 0x00, sizeof(struct track_work_struct) * 10 * 10);
	
	if(MOTOR_RUN_FORWARD == run_mode)
	{
		//if(g_src_board_id != 1)
		{
			mcu_push_medicine_track_only(g_src_board_id, 0xFF);//向1号板发送当前单板出货完成
			RTOS_TimeDlyHMSM(0, 0, 1, 0);
			//mcu_push_medicine_track_only(g_src_board_id, 0xFF);//向1号板发送当前单板出货完成
		}

		if(g_src_board_id == 1)
		board_push_finish &= ~(1<<0);//清标志
	}
	else if(MOTOR_RUN_BACKWARD == run_mode)
	{
		//if(g_src_board_id != 1)
		mcu_add_medicine_track_only(g_src_board_id, 0xFF);//向1号板发送当前单板补货完成
		RTOS_TimeDlyHMSM(0, 0, 1, 0);
		mcu_add_medicine_track_only(g_src_board_id, 0xFF);//向1号板发送当前单板补货完成
		
		if(g_src_board_id == 1)
		board_add_finish &= ~(1<<0);//清标志
	}
	run_mode = MOTOR_STOP;
	UsartPrintf(USART_DEBUG, "Exit Track_run!!!\r\n");

	heart_info.board_id = g_src_board_id;
	heart_info.board_status = STANDBY_STATUS;
	heart_info.medicine_track_number = 0; 
	
	return 0;
}


int Track_run_only(MOTOR_ENUM run_mode)
{
	int x = 0, y = 0;
	uint16_t delay_s = 0;
	uint16_t delay_ms = 0;
	//struct push_medicine_complete_request_info_struct  push_complete_info;

	UsartPrintf(USART_DEBUG, "Enter Track_run_only, mode[%d]!!!\r\n", run_mode);
	for(x = 0; x < 10; x++)
	{
		for(y = 0; y < 10; y++)
		{
			if(track_struct[x][y].push_time > KEY_DELAY_MS)
			{
				delay_s = (track_struct[x][y].push_time - KEY_DELAY_MS)/10;
				delay_ms = ((track_struct[x][y].push_time - KEY_DELAY_MS) % 10) * 100;
				
				motor_run_detect_track_num = x*10 + y + 1;	
				UsartPrintf(USART_DEBUG, "start:track[%d]mode[%d]time[%d]=>%ds.%dms\r\n", motor_run_detect_track_num, track_struct[x][y].motor_run, track_struct[x][y].push_time, delay_s, delay_ms);
				
				Motor_Set(track_struct[x][y].motor_run);//电机方向使能
				set_track(motor_run_detect_track_num, track_struct[x][y].motor_run);//货道使能
				
				RTOS_TimeDlyHMSM(0, 0, 0, KEY_DELAY_MS * 100);
				
				motor_run_detect_flag = 1;
				OverCurrentDetected = 0;
				
				RTOS_TimeDlyHMSM(0, 0, delay_s, delay_ms);
				
				motor_run_detect_flag = 0;
				
				UsartPrintf(USART_DEBUG, "stop:track[%d]mode[%d]time[%d]=>%ds.%dms\r\n", motor_run_detect_track_num, track_struct[x][y].motor_run, track_struct[x][y].push_time, delay_s, delay_ms);

				/*发生开关检测到，延时1.5S，预留时间给回退*/
				if(OverCurrentDetected)
				RTOS_TimeDlyHMSM(0, 0, 1, 500);
				
				set_track(motor_run_detect_track_num, MOTOR_STOP);//货道停止
				Motor_Set(MOTOR_STOP);	//电机停止
				OverCurrentDetected = 0;

				#if 1
				if(KeyScan(GPIOB, ForwardDetectKey) == KEYDOWN)//行程开关检测到到达货道头
				{
					UsartPrintf(USART_DEBUG, "Forward detect keep down\r\n");

					Motor_Set(MOTOR_RUN_BACKWARD);//电机方向使能
					set_track(motor_run_detect_track_num, MOTOR_RUN_BACKWARD);//货道使能
					
					RTOS_TimeDlyHMSM(0, 0, 1, 0);
					
					set_track(motor_run_detect_track_num, MOTOR_STOP);//货道停止
					Motor_Set(MOTOR_STOP);	//电机停止
				}
				else if(KeyScan(GPIOB, BackwardDetectKey) == KEYDOWN)//行程开关检测到到达货尾
				{
					UsartPrintf(USART_DEBUG, "Backword detect keep down\r\n");
					Motor_Set(MOTOR_RUN_FORWARD);//电机方向使能
					set_track(motor_run_detect_track_num, MOTOR_RUN_FORWARD);//货道使能
					
					RTOS_TimeDlyHMSM(0, 0, 1, 0);
					
					set_track(motor_run_detect_track_num, MOTOR_STOP);//货道停止
					Motor_Set(MOTOR_STOP);	//电机停止
				}
				#endif
				
			}

			
		}
		
	}
	
	Motor_Set(MOTOR_STOP);	
	memset(track_struct, 0x00, sizeof(struct track_work_struct) * 10 * 10);
	

	return 0;
}

#endif

static uint8_t old_status = MOTOR_STOP;

int Track_trigger_calc_runtime(uint8_t is_init, MOTOR_ENUM run_mode)
{
	Motor_Set(run_mode);
	set_track(cur_calc_track, run_mode);
	if(is_init)
	{
		memset(&track_time, 0x00, sizeof(struct track_cale_report_info_struct));
		track_time.track_start_num = cur_calc_track;
		track_time.board_id = g_src_board_id;
		running_time = 0;
		UsartPrintf(USART_DEBUG, "Calc Time[START]%d,%d,%d!!!\r\n", track_time.track_start_num, track_time.track_backward_time, track_time.track_backward_time);
	}
	if((!is_init) && (run_mode == MOTOR_STOP))
	{
		if(old_status == MOTOR_RUN_FORWARD)
		{
			//forward_running_time = running_time;
			track_time.track_forward_time = running_time;
			UsartPrintf(USART_DEBUG, "Forward Running Time :%d, %d\r\n", track_time.track_forward_time, running_time);
			running_time = 0;
			send_track_runtime_report(&track_time);
		}
		else if(old_status == MOTOR_RUN_BACKWARD)
		{
			//backward_running_time = running_time;
			track_time.track_backward_time = running_time;
			UsartPrintf(USART_DEBUG, "Backward Running Time:%d, %\r\n", track_time.track_backward_time, running_time);
			running_time = 0;
		}
		UsartPrintf(USART_DEBUG, "Calc Time[END]%d,%d,%d!!!\r\n", track_time.track_start_num, track_time.track_forward_time, track_time.track_backward_time);
		//if(track_time.track_forward_time && track_time.track_backward_time)
		//send_track_runtime_report(&track_time);
	}
	old_status = run_mode;
	return 0;
}

int Track_Runtime(uint8_t is_init, MOTOR_ENUM run_mode, MOTOR_ENUM cur_run_mode)
{
	Motor_Set(run_mode);
	set_track(cur_calc_track, run_mode);
	if(is_init)
	{
		memset(&track_time, 0x00, sizeof(struct track_cale_report_info_struct));
		track_time.track_start_num = cur_calc_track;
		track_time.board_id = g_src_board_id;
		running_time = 0;
		UsartPrintf(USART_DEBUG, "Calc Time[START]%d,%d,%d!!!\r\n", track_time.track_start_num, track_time.track_backward_time, track_time.track_backward_time);
	}
	if((!is_init) && (run_mode == MOTOR_STOP))
	{
		if(cur_run_mode == MOTOR_RUN_BACKWARD)
		{
			track_time.track_backward_time = running_time;
			UsartPrintf(USART_DEBUG, "Backward Running Time:%d, %\r\n", track_time.track_backward_time, running_time);
			running_time = 0;
		}
		else if(cur_run_mode == MOTOR_RUN_FORWARD)
		{
			track_time.track_forward_time = running_time;
			UsartPrintf(USART_DEBUG, "Forward Running Time :%d, %d\r\n", track_time.track_forward_time, running_time);
			running_time = 0;
			send_track_runtime_report(&track_time);
		}
		UsartPrintf(USART_DEBUG, "Calc Time[END]%d,%d,%d!!!\r\n", track_time.track_start_num, track_time.track_forward_time, track_time.track_backward_time);
	}
	return 0;
}

int Track_trigger_calc_runtime_error(int is_block, int step, MOTOR_ENUM run_mode)
{
	Motor_Set(run_mode);
	set_track(cur_calc_track, run_mode);
	if(!is_block)
	{
		if(step >= TRACK_MAX_TIME_MS)
		{
			track_time.track_backward_time = 999;
			track_time.track_forward_time = 999;
			
			Motor_Set(MOTOR_RUN_BACKWARD);//电机方向使能
			set_track(cur_calc_track, MOTOR_RUN_BACKWARD);//货道使能
			
			RTOS_TimeDlyHMSM(0, 0, 5, 0);
			
			set_track(cur_calc_track, MOTOR_STOP);//货道停止
			Motor_Set(MOTOR_STOP);	//电机停止
		}
		else
		{
			track_time.track_backward_time = 888;
			track_time.track_forward_time = 888;
		}
		send_track_runtime_report(&track_time);
	}
	
	return 0;
}


int Track_trigger(uint8_t track_num, MOTOR_ENUM run_mode)
{
	Motor_Set(run_mode);
	set_track(track_num, run_mode);
	return 0;
}

