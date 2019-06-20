/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	key.c
	*
	*	作者： 		
	*
	*	日期： 		2016-11-23
	*
	*	版本： 		V1.0
	*
	*	说明： 		按键IO初始化，按键功能判断
	*
	*	修改记录：	
	************************************************************
	************************************************************
	************************************************************
**/

//硬件驱动
#include "box.h"


KEY_STATUS key_status;
extern unsigned char calibrate_enable;




/*
************************************************************
*	函数名称：	Key_Init
*
*	函数功能：	按键IO初始化
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		SW1-PB1		SW2-PB0	
*				按下为低电平		释放为高电平
************************************************************
*/
void Key_Init(void)
{
	#if 0
	GPIO_InitTypeDef gpioInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	gpioInitStructure.GPIO_Mode = GPIO_Mode_IPU;
	gpioInitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	gpioInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioInitStructure);
	#else
 	GPIO_InitTypeDef GPIO_InitStructure;
	//初始化KEY0-->GPIOB.1,KEY1-->GPIOB.0  上拉输入
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);//使能PORTB时钟

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8|GPIO_Pin_9;//PB1~0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB1,0

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4;//PE4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIOB1,0
	
	#endif
	memset(&key_status, 0, sizeof(KEY_STATUS));
}

/*
************************************************************
*	函数名称：	KeyScan
*
*	函数功能：	按键电平扫描
*
*	入口参数：	GPIOX：需要扫描的GPIO组	NUM：该GPIO组内的编号
*
*	返回参数：	IO电平状态
*
*	说明：		
************************************************************
*/
_Bool KeyScan(GPIO_TypeDef* GPIOX, unsigned int NUM)
{
	
	if(!GPIO_ReadInputDataBit(GPIOX, NUM))	//按下  为低
	{
		return KEYDOWN;
	}
	else									//弹起  为高
	{
		return KEYUP;
	}
}

/*
************************************************************
*	函数名称：	Keyboard
*
*	函数功能：	按键功能检测
*
*	入口参数：	GPIOX：需要扫描的GPIO组	NUM：该GPIO组内的编号
*
*	返回参数：	IO电平状态
*
*	说明：		分单击、双击、长安
************************************************************
*/
#if 0
unsigned char Keyboard(void)
{
	unsigned char ret_val = 0;
	
	if(calibrate_enable == 0)
	return;
	
	if(KeyScan(GPIOB, KEY0) == KEYDOWN)						//有第二次按下，说明为双击
	{
		RTOS_TimeDly(5);									//让此任务进入阻塞态，这不影响代码正常的运行
		if(KeyScan(GPIOB, KEY0) == KEYDOWN)					//等待释放，无此句，双击后会跟一个单击动作
		key_status.Key0Stat = KEYDOWN;			
	}
	else
	{
		key_status.Key0Stat = KEYUP;
	}
	//UsartPrintf(USART_DEBUG, "key_status.Key0Stat[%d]\r\n",key_status.Key0Stat);

	if((key_status.Key0Stat == KEYDOWN) && (key_status.Key0OldStat == KEYUP))
	{
		key_status.Key0StatChange = KEY_UP2DOWN;
	}
	else if((key_status.Key0Stat == KEYDOWN) && (key_status.Key0OldStat == KEYDOWN))
	{
		key_status.Key0StatChange = KEY_DOWNKEEP;
	}
	else if((key_status.Key0Stat == KEYUP) && (key_status.Key0OldStat == KEYDOWN))
	{
		key_status.Key0StatChange = KEY_DOWN2UP;
	}
	else if((key_status.Key0Stat == KEYUP) && (key_status.Key0OldStat ==  KEYUP))
	{
		key_status.Key0StatChange = KEY_UPKEEP;
	}
	key_status.Key0OldStat = key_status.Key0Stat;

	
	
	if(KeyScan(GPIOB, KEY1) == KEYDOWN)							//有第二次按下，说明为已按下
	{
			RTOS_TimeDly(5);									//让此任务进入阻塞态，这不影响代码正常的运行
			if(KeyScan(GPIOB, KEY1) == KEYDOWN)					//等待释放，无此句，双击后会跟一个单击动作
			key_status.Key1Stat = KEYDOWN;			
	}
	else
	{
		key_status.Key1Stat = KEYUP;
	}

	
	//UsartPrintf(USART_DEBUG, "key_status.Key1Stat[%d]\r\n",key_status.Key1Stat);


	if((key_status.Key1Stat == KEYDOWN) && (key_status.Key1OldStat == KEYUP))
	{
		key_status.Key1StatChange = KEY_UP2DOWN;
	}
	else if((key_status.Key1Stat == KEYDOWN) && (key_status.Key1OldStat == KEYDOWN))
	{
		key_status.Key1StatChange = KEY_DOWNKEEP;
	}
	else if((key_status.Key1Stat == KEYUP) && (key_status.Key1OldStat == KEYDOWN))
	{
		key_status.Key1StatChange = KEY_DOWN2UP;
	}
	else if((key_status.Key1Stat == KEYUP) && (key_status.Key1OldStat == KEYUP))
	{
		key_status.Key1StatChange = KEY_UPKEEP;
	}
	key_status.Key1OldStat = key_status.Key1Stat;


	if(key_status.Key0Stat ^ key_status.Key1Stat)
	{
		ret_val = 1;
	}

	
	UsartPrintf(USART_DEBUG, "key_status.Key0StatChange[%d]key_status.Key1StatChange[%d]\r\n", key_status.Key0StatChange, key_status.Key1StatChange);

	return ret_val;	
}

#else


/*
************************************************************
*	函数名称：	Keyboard
*
*	函数功能：	按键功能检测
*
*	入口参数：	GPIOX：需要扫描的GPIO组	NUM：该GPIO组内的编号
*
*	返回参数：	IO电平状态
*
*	说明：		分单击、双击、长安
************************************************************
*/
unsigned char Keyboard(void)
{
	unsigned char ret_val = 0;
	
	//if(calibrate_enable == 0)
	//return;
	
	if(KeyScan(GPIOB, KEY0) == KEYDOWN)						//有第二次按下，说明为双击
	{
		RTOS_TimeDly(5);									//让此任务进入阻塞态，这不影响代码正常的运行
		if(KeyScan(GPIOB, KEY0) == KEYDOWN)					//等待释放，无此句，双击后会跟一个单击动作
		key_status.Key0Stat = KEYDOWN;

	}
	else
	{
		key_status.Key0Stat = KEYUP;
	}
	//UsartPrintf(USART_DEBUG, "key_status.Key0Stat[%d]\r\n",key_status.Key0Stat);
		
	if(KeyScan(GPIOB, KEY1) == KEYDOWN)							//有第二次按下，说明为已按下
	{
			RTOS_TimeDly(5);									//让此任务进入阻塞态，这不影响代码正常的运行
			if(KeyScan(GPIOB, KEY1) == KEYDOWN)					//等待释放，无此句，双击后会跟一个单击动作
			key_status.Key1Stat = KEYDOWN;			
	}
	else
	{
		key_status.Key1Stat = KEYUP;
	}


	if(key_status.Key0Stat ^ key_status.Key1Stat)
	{
		ret_val = 1;
	}

	
	UsartPrintf(USART_DEBUG, "key_status.Key0StatChange[%d]key_status.Key1StatChange[%d]\r\n", key_status.Key0StatChange, key_status.Key1StatChange);

	return ret_val;	
}

#endif


void EXTIX_Init(void)
{
 
 	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;

    Key_Init();	 //	按键端口初始化

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	//使能复用功能时钟

  	//GPIOB.8 中断线以及中断初始化配置   下降沿触发
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line8;	//KEY1
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

 	//GPIOB.9	  中断线以及中断初始化配置 下降沿触发 //KEY1
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line9;
  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器


	//GPIOE.4 中断线以及中断初始化配置	 下降沿触发
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource4);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line4;
	EXTI_Init(&EXTI_InitStructure);	  //根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器


    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能按键KEY2所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);




  	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			//使能按键KEY1所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级1 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
 
}


//外部中断4服务程序 
void EXTI4_IRQHandler(void)
{
	OS_SEM_DATA sema_info;

#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntEnter();	  
#endif  	
	if(EXTI_GetITStatus(EXTI_Line4)==SET)//是8线的中断
	{
		delay_ms(10);
		if(KeyScan(GPIOE, GPIO_Pin_4) == KEYDOWN) 					//消抖后检查是否有过流
		{		
			UsartPrintf(USART_DEBUG, "OverCurrent Dir = %d\r\n", motor_run_direction);
			if(motor_run_detect_flag == 1){
				
				UsartPrintf(USART_DEBUG, "Track[%d] Arrive End Position!!!!!!\r\n", motor_run_detect_track_num);
				
				set_track_y((motor_run_detect_track_num - 1)%10, MOTOR_STOP);

				OSSemQuery (SemOfOverCurrent, &sema_info);
				UsartPrintf(USART_DEBUG, "sema_info.OSCnt = %d!!!!!!\r\n", sema_info.OSCnt);

				if(sema_info.OSCnt == 0)
				OSSemPost(SemOfOverCurrent);
			}

			if(key_init == 0 && trigger_calc_flag == 0)
			{
				key_init = 1;			
				key_stat = 0;
				
				UsartPrintf(USART_DEBUG, "OverCurrent DO NOTHING!!!!!!\r\n");
			}
			else if(trigger_calc_flag == 1)
			{
				UsartPrintf(USART_DEBUG, "OverCurrent DOWN\r\n");
				if(key_stat == 0) //初始位置
				{
					UsartPrintf(USART_DEBUG, "Arrive First Position!!!!!!\r\n");
					Track_trigger_calc_runtime(1, MOTOR_STOP);
					trigger_calc_runtime = 0;
					trigger_calc_flag = 0;
				}
				else if(key_stat == 1) //正向
				{
					UsartPrintf(USART_DEBUG, "Arrive Last Position!!!!!!\r\n");
					Track_trigger_calc_runtime(0, MOTOR_STOP);
					trigger_calc_runtime = 0; 	
					trigger_calc_flag = 0;
				}
				else if(key_stat == 2)//反向		
				{
					UsartPrintf(USART_DEBUG, "Arrive 2First Position!!!!!!\r\n");
					Track_trigger_calc_runtime(0, MOTOR_STOP);
					trigger_calc_runtime = 0; 	
					trigger_calc_flag = 0;
					
					key_stat = 0xff;
					
				}
			}
			else if(motor_run_detect_flag == 1){
				UsartPrintf(USART_DEBUG, "OverCurrent DOWN:Track %d Arrive End Position!!!!!!\r\n", motor_run_detect_track_num);
				set_track_y((motor_run_detect_track_num - 1)%10, MOTOR_STOP);
			}
			
		}
		else if(KeyScan(GPIOE, KEY2) == KEYUP)
		{
			//UsartPrintf(USART_DEBUG, "UP-------------\r\n");
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line4);  //清除LINE2上的中断标志位	
#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	 OSIntExit();	 
#endif  

}
#if 0
 //外部中断5~9服务程序
void EXTI9_5_IRQHandler(void)
{		 		
#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntEnter();    
#endif  		


	if(EXTI_GetITStatus(EXTI_Line8)==SET)//是8线的中断
	{
		
		UsartPrintf(USART_DEBUG, "KEY0 CHECK:");
		if(KeyScan(GPIOB, KEY0) == KEYDOWN) 					//
		{		
			UsartPrintf(USART_DEBUG, "DOWN-------------\r\n");
		}
		else if(KeyScan(GPIOB, KEY0) == KEYUP)
		{
			UsartPrintf(USART_DEBUG, "UP-------------\r\n");
		}
		
		OSSemPost(SemOfKey);
	}
			 
	EXTI_ClearITPendingBit(EXTI_Line8);  //清除LINE2上的中断标志位	


	if(EXTI_GetITStatus(EXTI_Line9)==SET)//是8线的中断
	{
		UsartPrintf(USART_DEBUG, "KEY1 CHECK:");
		if(KeyScan(GPIOB, KEY1) == KEYDOWN) 					//
		{		
			UsartPrintf(USART_DEBUG, "DOWN-------------\r\n");
			
		}
		else if(KeyScan(GPIOB, KEY1) == KEYUP)
		{
			UsartPrintf(USART_DEBUG, "UP-------------\r\n");
		}
		OSSemPost(SemOfKey);
	}
	EXTI_ClearITPendingBit(EXTI_Line9);  //清除LINE2上的中断标志位
	
#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntExit();    
#endif    			  
} 

#else
void EXTI9_5_IRQHandler(void)
{		 	
	OS_SEM_DATA sema_info;

#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntEnter();    
#endif  	

	//前进到位检查
	if(EXTI_GetITStatus(EXTI_Line8)==SET)//是8线的中断
	{
		delay_ms(10);
		UsartPrintf(USART_DEBUG, "Forward:KEY0 CHECK DOWN: ", motor_run_direction);
		if((KeyScan(GPIOB, KEY0) == KEYDOWN)&& (motor_run_direction == MOTOR_RUN_FORWARD)) 					//
		{		
			UsartPrintf(USART_DEBUG, "Dir = %d\r\n", motor_run_direction);
			if(motor_run_detect_flag == 1)
			{
				UsartPrintf(USART_DEBUG, "KEY0:Track %d Arrive First Position!!!!!!\r\n", motor_run_detect_track_num);
				set_track_y((motor_run_detect_track_num - 1)%10, MOTOR_STOP);

				OSSemQuery (SemOfOverCurrent, &sema_info);
				UsartPrintf(USART_DEBUG, "sema_info.OSCnt = %d!!!!!!\r\n", sema_info.OSCnt);

				if(sema_info.OSCnt == 0)
				OSSemPost(SemOfOverCurrent);
				
			}
			else if(trigger_calc_flag == 1)
			{
				if(key_stat == 0) //初始位置
				{
					UsartPrintf(USART_DEBUG, "KEY0:Arrive First Position!!!!!!\r\n");
					Track_trigger_calc_runtime(1, MOTOR_STOP);
					trigger_calc_runtime = 0;				
					trigger_calc_flag = 0;
				}
				else if(key_stat == 2)//反向		
				{
					UsartPrintf(USART_DEBUG, "KEY0:Arrive 2First Position!!!!!!\r\n");

					Track_trigger_calc_runtime(0, MOTOR_STOP);
					trigger_calc_runtime = 0; 	
					trigger_calc_flag = 0;
					
					key_stat = 0xff;
				}
			}
		}
		else if(KeyScan(GPIOB, KEY0) == KEYUP)
		{
			UsartPrintf(USART_DEBUG, "UP \r\n");
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line8);  //清除LINE2上的中断标志位


	//后退到位检查
	if(EXTI_GetITStatus(EXTI_Line9) == SET)//是8线的中断
	{
		UsartPrintf(USART_DEBUG, "Backword:KEY1 CHECK DOWN:", motor_run_direction);
		delay_ms(10);
		if((KeyScan(GPIOB, KEY1) == KEYDOWN)&&(motor_run_direction == MOTOR_RUN_BACKWARD))					//
		{		
			UsartPrintf(USART_DEBUG, "Dir = %d\r\n", motor_run_direction);
			if(motor_run_detect_flag == 1){
				
				set_track_y((motor_run_detect_track_num - 1)%10, MOTOR_STOP);
				UsartPrintf(USART_DEBUG, "KEY1:Track %d Arrive End Position!!!!!!\r\n", motor_run_detect_track_num);

				OSSemQuery (SemOfOverCurrent, &sema_info);
				UsartPrintf(USART_DEBUG, "sema_info.OSCnt = %d!!!!!!\r\n", sema_info.OSCnt);

				if(sema_info.OSCnt == 0)
				OSSemPost(SemOfOverCurrent);
			}
			else if(trigger_calc_flag == 1)
			{
				if(key_stat == 1) //正向
				{
					UsartPrintf(USART_DEBUG, "KEY1:Arrive Last Position!!!!!!\r\n");
					Track_trigger_calc_runtime(0, MOTOR_STOP);
					trigger_calc_runtime = 0;	
					trigger_calc_flag = 0;		
				}
			}
		}
		else if(KeyScan(GPIOB, KEY1) == KEYUP)
		{
			UsartPrintf(USART_DEBUG, "UP \r\n");
		}		
	}
EXTI_ClearITPendingBit(EXTI_Line9);  //清除LINE2上的中断标志位	








	
#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntExit();    
#endif    			  
}




#endif

_Bool Key_Check(unsigned int NUM)
{
	_Bool ret_val = KEYUP;

	if(NUM == KEY0 || NUM == KEY1)
	{
		if(KeyScan(GPIOB, NUM) == KEYDOWN) 					
		{
			delay_ms(10);								
			if(KeyScan(GPIOB, NUM) == KEYDOWN) 			
			{
				UsartPrintf(USART_DEBUG, "%s KEYDOWN!!!\r\n", (NUM == KEY0) ? "KEY0":"KEY1");
				ret_val= KEYDOWN;
			}
		}
		else
		{
			ret_val = KEYUP;
		}
	}
	else if(NUM == KEY2)
	{
		if(KeyScan(GPIOE, KEY2) == KEYDOWN) 					
		{
			delay_ms(10);								
			if(KeyScan(GPIOE, KEY2) == KEYDOWN) 			
			{
				UsartPrintf(USART_DEBUG, "KEY2 KEYDOWN!!!\r\n");
				ret_val= KEYDOWN;
			}
		}
		else
		{
			ret_val = KEYUP;
		}
	}
	
	return ret_val;	
}

