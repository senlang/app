#ifndef _MOTOR_H_
#define _MOTOR_H_


#include "stm32f10x.h"
#include "motor.h"




typedef struct
{
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
} track_elem;


typedef struct
{
	uint8_t x;
	uint8_t y;
} track_addr;

uint8_t set_track(uint16_t track_num, uint8_t status);
void Track_Init(void);


#endif

