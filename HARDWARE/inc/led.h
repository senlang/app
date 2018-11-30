#ifndef _LED_H_
#define _LED_H_







typedef struct
{
	_Bool Led1Sta;
	_Bool Led2Sta;
} LED_STATUS;

extern LED_STATUS ledStatus;

typedef enum
{

	LED_OFF = 0,
	LED_ON

} LED_STATUS_ENUM;


typedef enum
{

	LED_1 = 0,
	LED_2

} LED_SELECT_ENUM;




void Led_Init(void);

void Led_Set(LED_SELECT_ENUM gpio, LED_STATUS_ENUM status);


#endif
