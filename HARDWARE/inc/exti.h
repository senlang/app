#ifndef __EXTI_H
#define __EXIT_H	 
#include "sys.h"

//外部中断 驱动代码	   
							  
//////////////////////////////////////////////////////////////////////////////////   	 


//#define EXTI_IO1  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)//读取按键0
//#define EXTI_IO2  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)//读取按键1
//#define EXTI_IO3  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)//读取按键2 
#define EXTI_IO4  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//读取按键3(WK_UP) 


void EXTIX_Init(void);//外部中断初始化		 		



#endif



