#ifndef _KEY_H_
#define _KEY_H_


#include "stm32f10x.h"




//#define KEY0			GPIO_Pin_8	//ǰ����λ
//#define KEY1			GPIO_Pin_9	//���˵�λ
//#define KEY2			GPIO_Pin_4	//�������

#define ForwardDetectKey			GPIO_Pin_8	//ǰ����λ
#define BackwardDetectKey			GPIO_Pin_9	//���˵�λ
#define CurrentDetectKey			GPIO_Pin_4	//�������
#define DrugPushFinishKey			GPIO_Pin_3	//ǰ����λ

#define RedDetectIntLine			3	//�������ж�line
#define OverCurrtenIntLine			4	//���������ж�line



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
