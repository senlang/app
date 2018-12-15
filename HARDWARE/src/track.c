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
#include "stm32_protocol.h"

extern OS_EVENT *SemOfConveyor;        	//Motor控制信号量
extern struct track_work_struct track_struct[10][10];


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


int Track_run(MOTOR_ENUM run_mode)
{
	int x = 0, y = 0;
	uint16_t time_elapsed = 0;
	uint16_t all_finish = 0;
	
	Motor_Set(run_mode);
	for(x = 0; x < 10; x++)
	{
		set_track_x(x, run_mode);
		for(y = 0; y < 10; y++)
		{
			UsartPrintf(USART_DEBUG, "track_struct[%d][%d].push_time = %d\r\n", x, y, track_struct[x][y].push_time);
			if(track_struct[x][y].push_time > 0)			
			set_track_y(y, run_mode);
		}

		all_finish = 0;
		do{
			for(y = 0; y < 10; y++)
			{
			
				//UsartPrintf(USART_DEBUG, "time_elapsed = %d, track_struct[%d][%d].push_time = %d\r\n", time_elapsed, x, y, track_struct[x][y].push_time);
				if(time_elapsed >= track_struct[x][y].push_time)
				{
					set_track_y(y, MOTOR_STOP);
					all_finish &= ~(1<<y);
				}
				else
				{
					all_finish |= 1<<y;
				}
				//UsartPrintf(USART_DEBUG, "all_finish = 0x%04x\r\n", all_finish);
			}
			time_elapsed += 5;
			RTOS_TimeDlyHMSM(0, 0, 0, 500);

			if(all_finish == 0)
			break;
		}while(1);

		time_elapsed = 0;
		set_track_x(x, MOTOR_STOP);
	}
	
	Motor_Set(MOTOR_STOP);	
	memset(track_struct, 0x00, sizeof(struct track_work_struct) * 10 * 10);
	
	OSSemPost(SemOfConveyor);
	return 0;
}


