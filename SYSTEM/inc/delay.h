#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"

//使用SysTick的普通计数模式对延迟进行管理

////////////////////////////////////////////////////////////////////////////////// 	 


#define RTOS_TimeDly(ticks) 						OSTimeDly(ticks)
#define RTOS_TimeDlyHMSM(hour, min, sec, ms)		OSTimeDlyHMSM(hour, min, sec, ms)

#define RTOS_EnterInt()								OSIntEnter()
#define RTOS_ExitInt()								OSIntExit()

#define RTOS_ENTER_CRITICAL()						{int cpu_sr;OS_ENTER_CRITICAL();}
#define RTOS_EXIT_CRITICAL()						{int cpu_sr;OS_EXIT_CRITICAL();}



void delay_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#endif





























