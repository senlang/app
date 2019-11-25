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

#define KEY_DELAY_500MS 5	//5 * 100ms,delay时间不能太短，否则电机启动电流过大会误判
#define MOTOR_RESERVE_TIME 2
#define TRACK_MAX_TIME_MS 3000	//800	//货道最大运行时间
#define TRACK_BACK_TIME 500

uint8_t set_track(uint16_t track_num, uint8_t status);
void Track_Init(void);
int Track_run(MOTOR_ENUM run_mode);
int Track_trigger_calc_runtime(uint8_t is_init, MOTOR_ENUM run_mode);
int Track_trigger_calc_runtime_error(int is_block, int step, MOTOR_ENUM run_mode);
int Track_Runtime(uint8_t is_init, MOTOR_ENUM run_mode, MOTOR_ENUM cur_run_mode);

int Track_trigger(uint8_t track_num, MOTOR_ENUM run_mode);//货道控制

uint8_t set_track_y(uint16_t col, uint8_t status);

void CleanTrackParam(void);

int Track_run_only(MOTOR_ENUM run_mode);

void CleanTrackParam(void);

void TrackRunMonitor(void);

#endif

