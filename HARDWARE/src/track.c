/**
	************************************************************
	************************************************************
	************************************************************
	*	�ļ����� 	track.c
	*
	*	���ߣ� 		
	*
	*	���ڣ� 		2018-10-10
	*
	*	�汾�� 		V1.0
	*
	*	˵���� 		����ʹ�ܿ���
	*
	*	�޸ļ�¼��	
	************************************************************
	************************************************************
	************************************************************
**/

//��Ƭ��ͷ�ļ�
#include "stm32f10x.h"

//ͷ�ļ�
#include "track.h"
#include "delay.h"
#include "usart.h"
#include "stm32_protocol.h"

extern uint8_t  g_src_board_id;  
extern OS_EVENT *SemOfConveyor;        	//Motor�����ź���
extern struct track_work_struct track_struct[10][10];
extern struct status_report_request_info_struct  heart_info;
extern uint8_t cur_calc_track;
extern uint16_t running_time;

extern uint8_t motor_run_detect_flag;
extern uint8_t motor_run_detect_track_num;
extern uint16_t board_push_finish;
extern uint16_t board_add_finish;


static  uint16_t forward_running_time;  
static  uint16_t backward_running_time; 

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

	//ʹ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
	
	//IO����
	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	//IO��ʼ��
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
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);				//��ֹJTAG����


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
	if(0 == status)
	{
		GPIO_WriteBit(x.GPIOx, x.GPIO_Pin, Bit_RESET);
		GPIO_WriteBit(y.GPIOx, y.GPIO_Pin, Bit_RESET);
	}
	else if((1 == status) || (2== status))
	{
		GPIO_WriteBit(x.GPIOx, x.GPIO_Pin, Bit_SET);
		GPIO_WriteBit(y.GPIOx, y.GPIO_Pin, Bit_SET);
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
	if(0 == status)
	{
		GPIO_WriteBit(y.GPIOx, y.GPIO_Pin, Bit_RESET);
	}
	else if((1 == status) || (2== status))
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
int Track_run(MOTOR_ENUM run_mode)
{
	int x = 0, y = 0;
	uint16_t delay_s = 0;
	uint16_t delay_ms = 0;
	struct push_medicine_complete_request_info_struct  push_complete_info;

	UsartPrintf(USART_DEBUG, "Enter Track_run, mode[%d]!!!\r\n");
	Motor_Set(run_mode);
	for(x = 0; x < 10; x++)
	{
		//UsartPrintf(USART_DEBUG, "Enter Track_run x = %d!!!\r\n", x);
		set_track_x(x, run_mode);
		do{
			for(y = 0; y < 10; y++)
			{
				if(track_struct[x][y].push_time > KEY_DELAY_MS)
				{
					delay_s = (track_struct[x][y].push_time - KEY_DELAY_MS)/10;
					delay_ms = ((track_struct[x][y].push_time - KEY_DELAY_MS) %10) * 100;
					UsartPrintf(USART_DEBUG, "start:track[%d]mode[%d]time[%d]=>%ds.%dms\r\n", x*10 + y + 1, track_struct[x][y].motor_run, track_struct[x][y].push_time, delay_s, delay_ms);
					set_track_y(y, run_mode);
					motor_run_detect_track_num = x*10 + y + 1;					
					RTOS_TimeDlyHMSM(0, 0, 0, KEY_DELAY_MS * 100);
					
					motor_run_detect_flag = 1;
					RTOS_TimeDlyHMSM(0, 0, delay_s, delay_ms);
					
					motor_run_detect_flag = 0;
					set_track_y(y, MOTOR_STOP);
					UsartPrintf(USART_DEBUG, "stop:track[%d]mode[%d]time[%d]=>%ds.%dms\r\n", x*10 + y + 1, track_struct[x][y].motor_run, track_struct[x][y].push_time, delay_s, delay_ms);

					if(MOTOR_RUN_FORWARD == run_mode)
					{
						mcu_push_medicine_track_only(g_src_board_id, motor_run_detect_track_num);
					}
					else if(MOTOR_RUN_BACKWARD == run_mode)
					{
						mcu_add_medicine_track_only(g_src_board_id, motor_run_detect_track_num);
					}

				}
			}
		}while(0);
		
		set_track_x(x, MOTOR_STOP);
	}
	
	Motor_Set(MOTOR_STOP);	
	memset(track_struct, 0x00, sizeof(struct track_work_struct) * 10 * 10);
	heart_info.board_id = g_src_board_id;
	heart_info.board_status = STANDBY_STATUS;
	heart_info.medicine_track_number = 0; 


	if(MOTOR_RUN_FORWARD == run_mode)
	{
		mcu_push_medicine_track_only(g_src_board_id, 0xFF);
		board_push_finish &= ~(1<<(g_src_board_id - 1));
		if(1 == g_src_board_id)
		OSSemPost(SemOfConveyor);
	}
	else if(MOTOR_RUN_BACKWARD == run_mode)
	{
		mcu_add_medicine_track_only(g_src_board_id, 0xFF);
		board_add_finish &= ~(1<<(g_src_board_id - 1));
	}

	UsartPrintf(USART_DEBUG, "Exit Track_run!!!\r\n");
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
		UsartPrintf(USART_DEBUG, "Start Calc:%d,%d,%d!!!\r\n", track_time.track_start_num, track_time.track_backward_time, track_time.track_backward_time);
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
		UsartPrintf(USART_DEBUG, "End Calc:%d,%d,%d!!!\r\n", track_time.track_start_num, track_time.track_forward_time, track_time.track_backward_time);
		//if(track_time.track_forward_time && track_time.track_backward_time)
		//send_track_runtime_report(&track_time);
	}
	old_status = run_mode;
}

int Track_trigger_calc_runtime_error(int is_block, int step, MOTOR_ENUM run_mode)
{
	Motor_Set(run_mode);
	set_track(cur_calc_track, run_mode);
	if(!is_block || (2 == step))
	{
		if(!is_block)
		{
			track_time.track_backward_time = 888;
			track_time.track_forward_time = 888;
		}
		
		send_track_runtime_report(&track_time);
	}
}


