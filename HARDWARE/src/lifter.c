
//单片机头文件
#include "stm32f10x.h"

//硬件驱动
#include "lifter.h"
#include "delay.h"
#include "usart.h"



#define LIFTER_UP_KEY GPIO_Pin_6	//GPIO_Pin_4
#define LIFTER_DOWN_KEY GPIO_Pin_7	//GPIO_Pin_5


/*升降机初始化*/
void Lifter_Key_Init(void)
{
	GPIO_InitTypeDef gpioInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/*PB6/PB7*/
	gpioInitStructure.GPIO_Mode = GPIO_Mode_IPU;
	gpioInitStructure.GPIO_Pin = LIFTER_UP_KEY | LIFTER_DOWN_KEY;
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
		GPIO_Pin = LIFTER_UP_KEY;
	}
	else
	{
		GPIOx = GPIOB;
		GPIO_Pin = LIFTER_DOWN_KEY;
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
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//打开GPIOA的时钟
	
	gpioInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;				//设置为输出
	gpioInitStruct.GPIO_Pin = LIFTER_UP_KEY|LIFTER_DOWN_KEY;						//将初始化的Pin脚
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;				//可承载的最大频率
	
	GPIO_Init(GPIOA, &gpioInitStruct);							//初始化GPIO
	
	GPIO_WriteBit(GPIOA, LIFTER_UP_KEY, Bit_RESET);
	GPIO_WriteBit(GPIOA, LIFTER_DOWN_KEY, Bit_RESET);

	Lifter_Key_Init();
}


void Lifter_Set(uint8_t status)
{
	uint16_t time = 0;
	
	UsartPrintf(USART_DEBUG, "Lifter run, status[%d]!!!\r\n", status);
	if(status == LIFTER_UP)
	{
		GPIO_WriteBit(GPIOA, LIFTER_DOWN_KEY, Bit_RESET);	
		GPIO_WriteBit(GPIOA, LIFTER_UP_KEY, Bit_SET);	

		while(Lifter_Key_Detect(LIFTER_UP) == LIFTER_KEY_NOT_DETECT){
			RTOS_TimeDlyHMSM(0, 0, 0, 100);
			time++;
			
			if(time >= 180 * 2)
			break;
		};

		GPIO_WriteBit(GPIOA, LIFTER_DOWN_KEY, Bit_RESET);	
		GPIO_WriteBit(GPIOA, LIFTER_UP_KEY, Bit_RESET);	
	}

	else if(status == LIFTER_FALL)
	{
		GPIO_WriteBit(GPIOA, LIFTER_UP_KEY, Bit_RESET);	
		GPIO_WriteBit(GPIOA, GPIO_Pin_7, Bit_SET);	

		while(Lifter_Key_Detect(LIFTER_FALL) == LIFTER_KEY_NOT_DETECT){
			RTOS_TimeDlyHMSM(0, 0, 0, 100);
			time++;
			
			if(time >= 180 * 2)
			break;
		};
		
		GPIO_WriteBit(GPIOA, GPIO_Pin_7, Bit_RESET);	
		GPIO_WriteBit(GPIOA, LIFTER_UP_KEY, Bit_RESET);	
	}
	
	UsartPrintf(USART_DEBUG, "Lifter stop, status[%d]!!!\r\n", status);
}


