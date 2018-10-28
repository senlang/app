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


KEY_STATUS key_status;
extern unsigned char calibrate_track_selected;
extern unsigned char calibrate_enable;


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
*	˵����		SW2-PD2		SW3-PC11	SW4-PC12	SW5-PC13	
*				����Ϊ�͵�ƽ		�ͷ�Ϊ�ߵ�ƽ
************************************************************
*/
void Key_Init(void)
{

	GPIO_InitTypeDef gpioInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	gpioInitStructure.GPIO_Mode = GPIO_Mode_IPU;
	gpioInitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	gpioInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioInitStructure);

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
	
	if(GPIOX == GPIOB)
	{
		if(!GPIO_ReadInputDataBit(GPIOB, NUM))	//����  Ϊ��
		{
			return KEYDOWN;
		}
		else									//����  Ϊ��
		{
			return KEYUP;
		}
	}
	return KEYUP;								//Ĭ�Ϸ��ذ����ͷ�
	
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
