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

#include "usart.h"
#include "stm32_protocol.h"





track_elem X_value[10] = {

	{GPIOB, GPIO_Pin_12},
	{GPIOB, GPIO_Pin_13},

	{GPIOB, GPIO_Pin_14},
	{GPIOB, GPIO_Pin_15},	

	{GPIOC, GPIO_Pin_6},
	{GPIOC, GPIO_Pin_7},

	{GPIOC, GPIO_Pin_8},
	{GPIOC, GPIO_Pin_9},

	{GPIOA, GPIO_Pin_8},
	{GPIOA, GPIO_Pin_11}
};


track_elem Y_value[10] = {

	{GPIOA, GPIO_Pin_12},
	{GPIOA, GPIO_Pin_15},

	{GPIOC, GPIO_Pin_10},
	{GPIOC, GPIO_Pin_11},	

	{GPIOC, GPIO_Pin_12},
	{GPIOD, GPIO_Pin_2},

	{GPIOB, GPIO_Pin_3},
	{GPIOB, GPIO_Pin_4},

	{GPIOB, GPIO_Pin_5},
	{GPIOB, GPIO_Pin_6}
};



void Track_Init(void)
{
	
	GPIO_InitTypeDef gpioInitStrcut;
	int i = 0, j = 0;

	//使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
	
	//IO配置
	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_15;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	//IO初始化
	GPIO_Init(GPIOA, &gpioInitStrcut);




	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioInitStrcut);
	

	gpioInitStrcut.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &gpioInitStrcut);
	


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



