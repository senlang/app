#ifndef _TRACK_H_
#define _TRACK_H_


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

#define KEY_DELAY_MS 5	//100ms

uint8_t set_track(uint16_t track_num, uint8_t status);
void Track_Init(void);
int Track_run(MOTOR_ENUM run_mode);
int Track_trigger_calc_runtime(uint8_t is_init, MOTOR_ENUM run_mode);
int Track_trigger_calc_runtime_error(int is_block, int step, MOTOR_ENUM run_mode);
uint8_t set_track_y(uint16_t col, uint8_t status);


#endif

