#ifndef _KEY_H_
#define _KEY_H_


#include "stm32f10x.h"




#define KEY0			GPIO_Pin_1
#define KEY1			GPIO_Pin_0
//#define KEY2			GPIO_Pin_1
//#define KEY3			GPIO_Pin_0



/*******************************************
			���������뵯��
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
			������ʱ
*******************************************/
#define KEYDOWN_LONG_TIME		20 //���㳤��ʱ����Ŀǰkeyboard����ÿ50ms����һ�Ρ�


extern void Key_Init(void);

extern void EXTIX_Init(void);

_Bool KeyScan(GPIO_TypeDef* GPIOX, unsigned int NUM);

extern unsigned char Keyboard(void);



typedef struct
{
	_Bool Key0Stat;
	_Bool Key0OldStat;
	_Bool Key1Stat;
	_Bool Key1OldStat;
	unsigned char Key0StatChange;
	unsigned char Key1StatChange;
} KEY_STATUS;


#endif
