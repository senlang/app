/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	key.c
	*
	*	作者： 		
	*
	*	日期： 		
	*
	*	版本： 		
	*
	*	说明： 行程开关和过流保护检测
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
extern uint8_t cur_calc_track;




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
	//初始化ForwardDetectKey-->GPIOB.1,BackwardDetectKey-->GPIOB.0  上拉输入
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
unsigned char Keyboard(void)
{
	unsigned char ret_val = 0;
	
	//if(calibrate_enable == 0)
	//return;
	
	if(KeyScan(GPIOB, ForwardDetectKey) == KEYDOWN)						//有第二次按下，说明为双击
	{
		RTOS_TimeDly(5);									//让此任务进入阻塞态，这不影响代码正常的运行
		if(KeyScan(GPIOB, ForwardDetectKey) == KEYDOWN)					//等待释放，无此句，双击后会跟一个单击动作
		key_status.ForwardDetectKeyStat = KEYDOWN;

	}
	else
	{
		key_status.ForwardDetectKeyStat = KEYUP;
	}
	//UsartPrintf(USART_DEBUG, "key_status.ForwardDetectKeyStat[%d]\r\n",key_status.ForwardDetectKeyStat);
		
	if(KeyScan(GPIOB, BackwardDetectKey) == KEYDOWN)							//有第二次按下，说明为已按下
	{
			RTOS_TimeDly(5);									//让此任务进入阻塞态，这不影响代码正常的运行
			if(KeyScan(GPIOB, BackwardDetectKey) == KEYDOWN)					//等待释放，无此句，双击后会跟一个单击动作
			key_status.BackwardDetectKeyStat = KEYDOWN;			
	}
	else
	{
		key_status.BackwardDetectKeyStat = KEYUP;
	}


	if(key_status.ForwardDetectKeyStat ^ key_status.BackwardDetectKeyStat)
	{
		ret_val = 1;
	}

	
	UsartPrintf(USART_DEBUG, "key_status.ForwardDetectKeyStatChange[%d]key_status.BackwardDetectKeyStatChange[%d]\r\n", key_status.ForwardDetectKeyStatChange, key_status.BackwardDetectKeyStatChange);

	return ret_val;	
}

void EXTIX_Init(void)
{
 
 	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;

    Key_Init();	 //	按键端口初始化

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	//使能复用功能时钟

  	//GPIOB.8 中断线以及中断初始化配置   下降沿触发
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line8;	//BackwardDetectKey
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

 	//GPIOB.9	  中断线以及中断初始化配置 下降沿触发 //BackwardDetectKey
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line9;
  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器


	//GPIOE.4 中断线以及中断初始化配置	 下降沿触发
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource4);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line4;
	EXTI_Init(&EXTI_InitStructure);	  //根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器


    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能按键CurrentDetectKey所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);




  	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			//使能按键BackwardDetectKey所在的外部中断通道
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
	if(0)//(EXTI_GetITStatus(EXTI_Line4)==SET)//是8线的中断
	{
		delay_ms(10);
		if(KeyScan(GPIOE, GPIO_Pin_4) == KEYDOWN) 					//消抖后检查是否有过流
		{		
			UsartPrintf(USART_DEBUG, "OverCurrent Dir = %d\r\n", motor_run_direction);

			/*出货、补货*/
			if(motor_run_detect_flag == 1)
			{
				UsartPrintf(USART_DEBUG, "Track[%d] over current!!!!!!\r\n", motor_run_detect_track_num);
				
				set_track(motor_run_detect_track_num, MOTOR_STOP);
				Motor_Set(MOTOR_STOP);

				OSSemQuery (SemOfOverCurrent, &sema_info);
				UsartPrintf(USART_DEBUG, "sema_info.OSCnt = %d!!!!!!\r\n", sema_info.OSCnt);

				if(sema_info.OSCnt == 0)
				OSSemPost(SemOfOverCurrent);

				OverCurrentDetected = 1;
			}
			
			/*货道时间计算*/
			else if(trigger_calc_flag == 1)
			{
				UsartPrintf(USART_DEBUG, "OverCurrent DOWN\r\n");

				if(key_stat == 0) //初始位置
				{
					UsartPrintf(USART_DEBUG, "Arrive First Position!!!!!!\r\n");
					Track_Runtime(1, MOTOR_STOP, MOTOR_RUN_FORWARD);
					trigger_calc_runtime = 0;
					trigger_calc_flag = 0;
				}
				else if(key_stat == 1) //正向
				{
					UsartPrintf(USART_DEBUG, "Arrive Last Position!!!!!!\r\n");
					Track_Runtime(0, MOTOR_STOP, MOTOR_RUN_BACKWARD);
					trigger_calc_runtime = 0; 	
					trigger_calc_flag = 0;
				}
				else if(key_stat == 2)//反向		
				{
					UsartPrintf(USART_DEBUG, "Arrive 2First Position!!!!!!\r\n");
					Track_Runtime(0, MOTOR_STOP, MOTOR_RUN_FORWARD);
					trigger_calc_runtime = 0; 	
					trigger_calc_flag = 0;
					key_stat = 0xff;
				}
			}
		}
		else if(KeyScan(GPIOE, CurrentDetectKey) == KEYUP)
		{
			//UsartPrintf(USART_DEBUG, "UP-------------\r\n");
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line4);  //清除LINE2上的中断标志位	
#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	 OSIntExit();	 
#endif  

}

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
		UsartPrintf(USART_DEBUG, "Forward:ForwardDetectKey CHECK: ", motor_run_direction);
		if((KeyScan(GPIOB, ForwardDetectKey) == KEYDOWN))//&& (motor_run_direction == MOTOR_RUN_FORWARD)) 					//
		{		
			UsartPrintf(USART_DEBUG, "DOWN. Dir = %d\r\n", motor_run_direction);
			if(motor_run_detect_flag == 1)
			{
				UsartPrintf(USART_DEBUG, "ForwardDetectKey:Track %d Arrive First Position!!!!!!\r\n", motor_run_detect_track_num);
				set_track(motor_run_detect_track_num, MOTOR_STOP);
				Motor_Set(MOTOR_STOP);
				
				OSSemQuery (SemOfOverCurrent, &sema_info);
				UsartPrintf(USART_DEBUG, "sema_info.OSCnt = %d!!!!!!\r\n", sema_info.OSCnt);

				if(sema_info.OSCnt == 0)
				OSSemPost(SemOfOverCurrent);
				
				OverCurrentDetected = 1;
			}
			else if(trigger_calc_flag == 1)
			{
				#if 1
				set_track(cur_calc_track, MOTOR_STOP);
				Motor_Set(MOTOR_STOP);
				#else
				if(key_stat == 0) //初始位置
				{
					UsartPrintf(USART_DEBUG, "ForwardDetectKey:Arrive First Position!!!!!!\r\n");
					Track_trigger_calc_runtime(1, MOTOR_STOP);
					trigger_calc_runtime = 0;				
					trigger_calc_flag = 0;
				}
				else if(key_stat == 2)//反向		
				{
					UsartPrintf(USART_DEBUG, "ForwardDetectKey:Arrive 2First Position!!!!!!\r\n");

					Track_trigger_calc_runtime(0, MOTOR_STOP);
					trigger_calc_runtime = 0; 	
					trigger_calc_flag = 0;
					
					key_stat = 0xff;
				}
				#endif
			}
		}
		else if(KeyScan(GPIOB, ForwardDetectKey) == KEYUP)
		{
			UsartPrintf(USART_DEBUG, "UP \r\n");
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line8);  //清除LINE2上的中断标志位


	//后退到位检查
	if(EXTI_GetITStatus(EXTI_Line9) == SET)//是8线的中断
	{
		UsartPrintf(USART_DEBUG, "Backword:BackwardDetectKey CHECK:", motor_run_direction);
		delay_ms(10);
		if((KeyScan(GPIOB, BackwardDetectKey) == KEYDOWN))//&&(motor_run_direction == MOTOR_RUN_BACKWARD))					//
		{		
			UsartPrintf(USART_DEBUG, "DOWN. Dir = %d\r\n", motor_run_direction);
			if(motor_run_detect_flag == 1){
				
				set_track(motor_run_detect_track_num, MOTOR_STOP);
				UsartPrintf(USART_DEBUG, "BackwardDetectKey:Track %d Arrive End Position!!!!!!\r\n", motor_run_detect_track_num);

				OSSemQuery (SemOfOverCurrent, &sema_info);
				UsartPrintf(USART_DEBUG, "sema_info.OSCnt = %d!!!!!!\r\n", sema_info.OSCnt);

				if(sema_info.OSCnt == 0)
				OSSemPost(SemOfOverCurrent);

				OverCurrentDetected = 1;
			}
			else if(trigger_calc_flag == 1)
			{
				#if 1
				set_track(cur_calc_track, MOTOR_STOP);
				Motor_Set(MOTOR_STOP);
				#else
				if(key_stat == 1) //正向
				{
					UsartPrintf(USART_DEBUG, "BackwardDetectKey:Arrive Last Position!!!!!!\r\n");
					Track_trigger_calc_runtime(0, MOTOR_STOP);
					trigger_calc_runtime = 0;	
					trigger_calc_flag = 0;		
				}
				#endif
			}
		}
		else if(KeyScan(GPIOB, BackwardDetectKey) == KEYUP)
		{
			UsartPrintf(USART_DEBUG, "UP \r\n");
		}		
	}
	EXTI_ClearITPendingBit(EXTI_Line9);  //清除LINE2上的中断标志位	








	
#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntExit();    
#endif    			  
}



_Bool Key_Check(unsigned int NUM)
{
	_Bool ret_val = KEYUP;

	if(NUM == ForwardDetectKey || NUM == BackwardDetectKey)
	{
		if(KeyScan(GPIOB, NUM) == KEYDOWN) 					
		{
			delay_ms(10);								
			if(KeyScan(GPIOB, NUM) == KEYDOWN) 			
			{
				UsartPrintf(USART_DEBUG, "%s KEYDOWN!!!\r\n", (NUM == ForwardDetectKey) ? "ForwardDetectKey":"BackwardDetectKey");
				ret_val= KEYDOWN;
			}
		}
		else
		{
			ret_val = KEYUP;
		}
	}
	else if(NUM == CurrentDetectKey)
	{
		if(KeyScan(GPIOE, CurrentDetectKey) == KEYDOWN) 					
		{
			delay_ms(10);								
			if(KeyScan(GPIOE, CurrentDetectKey) == KEYDOWN) 			
			{
				UsartPrintf(USART_DEBUG, "CurrentDetectKey KEYDOWN!!!\r\n");
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

