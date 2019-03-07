/**
	************************************************************
	************************************************************
	************************************************************
	*	�ļ����� 	key.c
	*
	*	���ߣ� 		
	*
	*	���ڣ� 		2016-11-23
	*
	*	�汾�� 		V1.0
	*
	*	˵���� 		����IO��ʼ�������������ж�
	*
	*	�޸ļ�¼��	
	************************************************************
	************************************************************
	************************************************************
**/

//����ͷ�ļ�
#include "key.h"

//Ӳ������
#include "delay.h"
#include "usart.h"
#include "motor.h"

KEY_STATUS key_status;
extern unsigned char calibrate_enable;
extern OS_EVENT *SemOfKey;          //Motor�����ź���
extern OS_EVENT *SemOfCalcTime;

extern uint8_t trigger_calc_runtime;
static uint8_t key_stat = 0;
static uint8_t key_init = 0;
extern uint8_t trigger_calc_flag; //0;1;2


/*
************************************************************
*	�������ƣ�	Key_Init
*
*	�������ܣ�	����IO��ʼ��
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		SW1-PB1		SW2-PB0	
*				����Ϊ�͵�ƽ		�ͷ�Ϊ�ߵ�ƽ
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
	//��ʼ��KEY0-->GPIOB.1,KEY1-->GPIOB.0  ��������
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);//ʹ��PORTBʱ��

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8|GPIO_Pin_9;//PB1~0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB1,0

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4;//PE4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
 	GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIOB1,0
	
	#endif
	memset(&key_status, 0, sizeof(KEY_STATUS));
}

/*
************************************************************
*	�������ƣ�	KeyScan
*
*	�������ܣ�	������ƽɨ��
*
*	��ڲ�����	GPIOX����Ҫɨ���GPIO��	NUM����GPIO���ڵı��
*
*	���ز�����	IO��ƽ״̬
*
*	˵����		
************************************************************
*/
_Bool KeyScan(GPIO_TypeDef* GPIOX, unsigned int NUM)
{
	
	if(!GPIO_ReadInputDataBit(GPIOX, NUM))	//����  Ϊ��
	{
		return KEYDOWN;
	}
	else									//����  Ϊ��
	{
		return KEYUP;
	}
}

/*
************************************************************
*	�������ƣ�	Keyboard
*
*	�������ܣ�	�������ܼ��
*
*	��ڲ�����	GPIOX����Ҫɨ���GPIO��	NUM����GPIO���ڵı��
*
*	���ز�����	IO��ƽ״̬
*
*	˵����		�ֵ�����˫��������
************************************************************
*/
#if 0
unsigned char Keyboard(void)
{
	unsigned char ret_val = 0;
	
	if(calibrate_enable == 0)
	return;
	
	if(KeyScan(GPIOB, KEY0) == KEYDOWN)						//�еڶ��ΰ��£�˵��Ϊ˫��
	{
		RTOS_TimeDly(5);									//�ô������������̬���ⲻӰ���������������
		if(KeyScan(GPIOB, KEY0) == KEYDOWN)					//�ȴ��ͷţ��޴˾䣬˫������һ����������
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

	
	
	if(KeyScan(GPIOB, KEY1) == KEYDOWN)							//�еڶ��ΰ��£�˵��Ϊ�Ѱ���
	{
			RTOS_TimeDly(5);									//�ô������������̬���ⲻӰ���������������
			if(KeyScan(GPIOB, KEY1) == KEYDOWN)					//�ȴ��ͷţ��޴˾䣬˫������һ����������
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
*	�������ƣ�	Keyboard
*
*	�������ܣ�	�������ܼ��
*
*	��ڲ�����	GPIOX����Ҫɨ���GPIO��	NUM����GPIO���ڵı��
*
*	���ز�����	IO��ƽ״̬
*
*	˵����		�ֵ�����˫��������
************************************************************
*/
unsigned char Keyboard(void)
{
	unsigned char ret_val = 0;
	
	//if(calibrate_enable == 0)
	//return;
	
	if(KeyScan(GPIOB, KEY0) == KEYDOWN)						//�еڶ��ΰ��£�˵��Ϊ˫��
	{
		RTOS_TimeDly(5);									//�ô������������̬���ⲻӰ���������������
		if(KeyScan(GPIOB, KEY0) == KEYDOWN)					//�ȴ��ͷţ��޴˾䣬˫������һ����������
		key_status.Key0Stat = KEYDOWN;

	}
	else
	{
		key_status.Key0Stat = KEYUP;
	}
	//UsartPrintf(USART_DEBUG, "key_status.Key0Stat[%d]\r\n",key_status.Key0Stat);
		
	if(KeyScan(GPIOB, KEY1) == KEYDOWN)							//�еڶ��ΰ��£�˵��Ϊ�Ѱ���
	{
			RTOS_TimeDly(5);									//�ô������������̬���ⲻӰ���������������
			if(KeyScan(GPIOB, KEY1) == KEYDOWN)					//�ȴ��ͷţ��޴˾䣬˫������һ����������
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

    Key_Init();	 //	�����˿ڳ�ʼ��

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	//ʹ�ܸ��ù���ʱ��

  	//GPIOB.8 �ж����Լ��жϳ�ʼ������   �½��ش���
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line8;	//KEY1
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

 	//GPIOB.9	  �ж����Լ��жϳ�ʼ������ �½��ش��� //KEY1
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line9;
  	EXTI_Init(&EXTI_InitStructure);	  	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���


	//GPIOE.4 �ж����Լ��жϳ�ʼ������	 �½��ش���
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource4);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line4;
	EXTI_Init(&EXTI_InitStructure);	  //����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���


    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//ʹ�ܰ���KEY2���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2�� 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//�����ȼ�2
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);




  	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			//ʹ�ܰ���KEY1���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//��ռ���ȼ�2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//�����ȼ�1 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);  	  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
 
}


//�ⲿ�ж�4������� 
void EXTI4_IRQHandler(void)
{
#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntEnter();	  
#endif  	
	if(EXTI_GetITStatus(EXTI_Line4)==SET)//��8�ߵ��ж�
	{
		//UsartPrintf(USART_DEBUG, "KEY2 CHECK:");
		if(KeyScan(GPIOE, GPIO_Pin_4) == KEYDOWN) 					//�еڶ��ΰ��£�˵��Ϊ˫��
		{		
			//UsartPrintf(USART_DEBUG, "DOWN-------------\r\n");
			//Motor_Set(MOTOR_STOP);
			//set_track(calibrate_track_selected, MOTOR_STOP);

			if(key_init == 0 && trigger_calc_flag == 0)
			{
				key_init = 1;			
				key_stat = 0;
				
				UsartPrintf(USART_DEBUG, "Post SemOfCalcTime!!!!!!\r\n");
				OSSemPost(SemOfCalcTime);
			}
			else if(trigger_calc_flag == 1)
			{
				if(key_stat%3 == 0) //��ʼλ��
				{
					Track_trigger_calc_runtime(1, MOTOR_STOP);
					trigger_calc_runtime = 0;
					key_stat++;
					
					UsartPrintf(USART_DEBUG, "Arrive First Position!!!!!!\r\n");
				}
				else if(key_stat%3 == 1) //����
				{
					Track_trigger_calc_runtime(0, MOTOR_STOP);
					trigger_calc_runtime = 0; 			
					key_stat++;
					
					UsartPrintf(USART_DEBUG, "Arrive Last Position!!!!!!\r\n");
				}
				else if(key_stat%3 == 2)//����		
				{
					Track_trigger_calc_runtime(0, MOTOR_STOP);
					trigger_calc_runtime = 0; 			
					key_stat = 0;
					
					UsartPrintf(USART_DEBUG, "Arrive 2First Position!!!!!!\r\n");
				}
			}
			
		}
		else if(KeyScan(GPIOE, KEY2) == KEYUP)
		{
			//UsartPrintf(USART_DEBUG, "UP-------------\r\n");
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line4);  //���LINE2�ϵ��жϱ�־λ	
#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	 OSIntExit();	 
#endif  

}

 //�ⲿ�ж�5~9�������
void EXTI9_5_IRQHandler(void)
{		 		
#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntEnter();    
#endif  		


	if(EXTI_GetITStatus(EXTI_Line8)==SET)//��8�ߵ��ж�
	{
		
		UsartPrintf(USART_DEBUG, "KEY0 CHECK:");
		if(KeyScan(GPIOB, KEY0) == KEYDOWN) 					//�еڶ��ΰ��£�˵��Ϊ˫��
		{		
			UsartPrintf(USART_DEBUG, "DOWN-------------\r\n");
		}
		else if(KeyScan(GPIOB, KEY0) == KEYUP)
		{
			UsartPrintf(USART_DEBUG, "UP-------------\r\n");
		}
		
		OSSemPost(SemOfKey);
	}
			 
	EXTI_ClearITPendingBit(EXTI_Line8);  //���LINE2�ϵ��жϱ�־λ	


	if(EXTI_GetITStatus(EXTI_Line9)==SET)//��8�ߵ��ж�
	{
		UsartPrintf(USART_DEBUG, "KEY1 CHECK:");
		if(KeyScan(GPIOB, KEY1) == KEYDOWN) 					//�еڶ��ΰ��£�˵��Ϊ˫��
		{		
			UsartPrintf(USART_DEBUG, "DOWN-------------\r\n");
			
		}
		else if(KeyScan(GPIOB, KEY1) == KEYUP)
		{
			UsartPrintf(USART_DEBUG, "UP-------------\r\n");
		}
		OSSemPost(SemOfKey);
	}
	EXTI_ClearITPendingBit(EXTI_Line9);  //���LINE2�ϵ��жϱ�־λ
	
#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntExit();    
#endif    			  
} 






