#ifndef __EXTI_H
#define __EXIT_H	 
#include "sys.h"

//STM32F103���İ�����
//�⺯���汾����
/************** Ƕ��ʽ������  **************/
/********** mcudev.taobao.com ��Ʒ  ********/

//////////////////////////////////////////////////////////////////////////////////	 

//STM32������
//�ⲿ�ж� ��������	   
							  
//////////////////////////////////////////////////////////////////////////////////   	 


//#define EXTI_IO1  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)//��ȡ����0
//#define EXTI_IO2  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)//��ȡ����1
//#define EXTI_IO3  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)//��ȡ����2 
#define EXTI_IO4  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//��ȡ����3(WK_UP) 


void EXTIX_Init(void);//�ⲿ�жϳ�ʼ��		 		



#endif



