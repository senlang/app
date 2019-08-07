
//��Ƭ��ͷ�ļ�
#include "stm32f10x.h"

//Ӳ������
#include "lifter.h"
#include "delay.h"
#include "usart.h"





/*��������ʼ��*/
void Lifter_Key_Init(void)
{
	GPIO_InitTypeDef gpioInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/*PB6/PB7*/
	gpioInitStructure.GPIO_Mode = GPIO_Mode_IPU;
	gpioInitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	gpioInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpioInitStructure);
}

_Bool Lifter_Key_Status(unsigned char door_detect)
{
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	
	if(door_detect == LIFTER_UP)
	{
		GPIOx = GPIOB;
		GPIO_Pin = GPIO_Pin_6;
	}
	else
	{
		GPIOx = GPIOB;
		GPIO_Pin = GPIO_Pin_7;
	}

	if(!GPIO_ReadInputDataBit(GPIOx, GPIO_Pin))
	{
		return LIFTER_KEY_DETECT;
	}
	else			
	{
		return LIFTER_KEY_NOT_DETECT;
	}
}


unsigned char Lifter_Key_Detect(unsigned char door_detect)
{
	unsigned char ret_val = 0;
	
	if(Lifter_Key_Status(door_detect) == LIFTER_KEY_DETECT)
	{
		RTOS_TimeDly(20);					
		if(Lifter_Key_Status(door_detect) == LIFTER_KEY_DETECT)
		{
			UsartPrintf(USART_DEBUG, "Lifter_Key_Detect!!!\r\n");
			ret_val = LIFTER_KEY_DETECT;
		}			
	}
	else
	{
		ret_val = LIFTER_KEY_NOT_DETECT;
	}
	return ret_val;
}

void Lifter_Init(void)
{

	GPIO_InitTypeDef gpioInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//��GPIOA��ʱ��
	
	gpioInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;				//����Ϊ���
	gpioInitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;						//����ʼ����Pin��
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;				//�ɳ��ص����Ƶ��
	
	GPIO_Init(GPIOA, &gpioInitStruct);							//��ʼ��GPIO
	
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
	GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);

	Lifter_Key_Init();
}


void Lifter_Set(uint8_t status)
{
	uint16_t time = 0;
	
	UsartPrintf(USART_DEBUG, "Lifter run, status[%d]!!!\r\n", status);
	if(status == LIFTER_UP)
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);	
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);	

		while(Lifter_Key_Detect(LIFTER_UP) == LIFTER_KEY_NOT_DETECT){
			RTOS_TimeDlyHMSM(0, 0, 0, 100);
			time++;
			
			if(time >= 200)
			break;
		};

		GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);	
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);	
	}

	else if(status == LIFTER_FALL)
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);	
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);	

		while(Lifter_Key_Detect(LIFTER_FALL) == LIFTER_KEY_NOT_DETECT){
			RTOS_TimeDlyHMSM(0, 0, 0, 100);
			time++;
			
			if(time >= 200)
			break;
		};
		
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);	
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);	
	}
	
	UsartPrintf(USART_DEBUG, "Lifter stop, status[%d]!!!\r\n", status);
}


