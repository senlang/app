#ifndef _KEY_H_
#define _KEY_H_


#include "stm32f10x.h"




//#define KEY0			GPIO_Pin_8	//前进到位
//#define KEY1			GPIO_Pin_9	//后退到位
//#define KEY2			GPIO_Pin_4	//过流检查

#define ForwardDetectKey			GPIO_Pin_8	//前进到位
#define BackwardDetectKey			GPIO_Pin_9	//后退到位
#define CurrentDetectKey			GPIO_Pin_4	//过流检查
#define DrugPushFinishKey			GPIO_Pin_3	//前进到位

#define RedDetectIntLine			3	//红外检测中断line
#define OverCurrtenIntLine			4	//过流保护中断line



/*******************************************
			按键按下与弹起
*******************************************/
#define KEYDOWN			1
#define KEYUP			0

typedef enum{
	KEY_UPKEEP = 0,
	KEY_UP2DOWN = 1,
    KEY_DOWN2UP = 2,
	KEY_DOWNKEEP = 3,
}KEY_STAT_CHANGE;  



/*******************************************
			按键计时
*******************************************/
#define KEYDOWN_LONG_TIME		20 //计算长按时长。目前keyboard函数每50ms调用一次。


extern void Key_Init(void);

extern void EXTIX_Init(void);

_Bool KeyScan(GPIO_TypeDef* GPIOX, unsigned int NUM);

_Bool Key_Check(unsigned int NUM);

extern unsigned char Keyboard(void);



typedef struct
{
	_Bool ForwardDetectKeyStat;
	_Bool ForwardDetectKeyOldStat;
	_Bool BackwardDetectKeyStat;
	_Bool BackwardDetectKeyOldStat;
	unsigned char ForwardDetectKeyStatChange;
	unsigned char BackwardDetectKeyStatChange;
} KEY_STATUS;

void exti_interrupt_set(uint8_t status);


#endif
