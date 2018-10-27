/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	key.c
	*
	*	作者： 		
	*
	*	日期： 		2016-11-23
	*
	*	版本： 		V1.0
	*
	*	说明： 		按键IO初始化，按键功能判断
	*
	*	修改记录：	
	************************************************************
	************************************************************
	************************************************************
**/

//按键头文件
#include "key.h"

//硬件驱动
#include "delay.h"


KEY_STATUS key_status;


/*
************************************************************
*	函数名称：	Key_Init
*
*	函数功能：	按键IO初始化
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		SW2-PD2		SW3-PC11	SW4-PC12	SW5-PC13	
*				按下为低电平		释放为高电平
************************************************************
*/
void Key_Init(void)
{

	GPIO_InitTypeDef gpioInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	gpioInitStructure.GPIO_Mode = GPIO_Mode_IPU;
	gpioInitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_11 | GPIO_Pin_12;
	gpioInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &gpioInitStructure);

	memset(&key_status, 0, sizeof(KEY_STATUS));
}

/*
************************************************************
*	函数名称：	KeyScan
*
*	函数功能：	按键电平扫描
*
*	入口参数：	GPIOX：需要扫描的GPIO组	NUM：该GPIO组内的编号
*
*	返回参数：	IO电平状态
*
*	说明：		
************************************************************
*/
_Bool KeyScan(GPIO_TypeDef* GPIOX, unsigned int NUM)
{
	
	if(GPIOX == GPIOB)
	{
		if(!GPIO_ReadInputDataBit(GPIOB, NUM))	//按下  为低
		{
			return KEYDOWN;
		}
		else									//弹起  为高
		{
			return KEYUP;
		}
	}
	return KEYUP;								//默认返回按键释放
	
}

/*
************************************************************
*	函数名称：	Keyboard
*
*	函数功能：	按键功能检测
*
*	入口参数：	GPIOX：需要扫描的GPIO组	NUM：该GPIO组内的编号
*
*	返回参数：	IO电平状态
*
*	说明：		分单击、双击、长安
************************************************************
*/
unsigned char Keyboard(void)
{
	unsigned char ret_val = 0;
	if(KeyScan(GPIOB, KEY0) == KEYDOWN)							//有第二次按下，说明为双击
	{
			RTOS_TimeDly(5);									//让此任务进入阻塞态，这不影响代码正常的运行
			if(KeyScan(GPIOB, KEY0) == KEYDOWN)					//等待释放，无此句，双击后会跟一个单击动作
			key_status.Key0Stat = KEYDOWN;			
	}
	else
	{
		key_status.Key0Stat = KEYUP;
	}	

	if((key_status.Key0Stat == KEYDOWN) && (key_status.Key0OldStat == KEYUP))
	{
		key_status.Key0StatChange = KEY_UP2DOWN;
	}
	else if((key_status.Key0Stat == KEYDOWN) && (key_status.Key0OldStat == KEYDOWN))
	{
		key_status.Key0StatChange = KEY_DOWNKEEP;
	}
	else if((key_status.Key0Stat == KEYUP) && (key_status.Key0OldStat == KEYDOWN))
	{
		key_status.Key0StatChange = KEY_DOWN2UP;
	}
	else if((key_status.Key0Stat == KEYUP) && (key_status.Key0OldStat ==  KEYUP))
	{
		key_status.Key0StatChange = KEY_UPKEEP;
	}

	
	
	if(KeyScan(GPIOB, KEY1) == KEYDOWN)							//有第二次按下，说明为双击
	{
			RTOS_TimeDly(5);									//让此任务进入阻塞态，这不影响代码正常的运行
			if(KeyScan(GPIOB, KEY1) == KEYDOWN)					//等待释放，无此句，双击后会跟一个单击动作
			key_status.Key1Stat = KEYDOWN;			
	}
	else
	{
		key_status.Key1Stat = KEYUP;
	}

	
	if(key_status.Key0Stat ^ key_status.Key1Stat)
	{
		ret_val = 1;
	}

	if((key_status.Key1Stat == KEYDOWN) && (key_status.Key1OldStat == KEYUP))
	{
		key_status.Key1StatChange = KEY_UP2DOWN;
	}
	else if((key_status.Key1Stat == KEYDOWN) && (key_status.Key1OldStat == KEYDOWN))
	{
		key_status.Key1StatChange = KEY_DOWNKEEP;
	}
	else if((key_status.Key1Stat == KEYUP) && (key_status.Key1OldStat == KEYDOWN))
	{
		key_status.Key1StatChange = KEY_DOWN2UP;
	}
	else if((key_status.Key1Stat == KEYUP) && (key_status.Key1OldStat == KEYUP))
	{
		key_status.Key1StatChange = KEY_UPKEEP;
	}

	return ret_val;	
}
