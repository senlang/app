#include "stm32f10x.h"
#include "key.h"
#include "sys.h" 
#include "delay.h"


//STM32F103核心板例程
//库函数版本例程
/************** 嵌入式开发网  **************/
/********** mcudev.taobao.com 出品  ********/


//////////////////////////////////////////////////////////////////////////////////	 

//STM32开发板
//按键驱动代码	   
						  
//////////////////////////////////////////////////////////////////////////////////  
								    
//按键初始化函数
void KEY_Init(void) //IO初始化
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
	//初始化KEY1-->PA0上拉输入
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能PA时钟

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;//PA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA


}
//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//0，没有任何按键按下


static u8 key_up=1;//按键按松开标志

u8 KEY_Scan(u8 mode)
{	 
	
	if(mode)key_up=1;  //支持连按		  
	if(key_up&&(KEY1==0))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(KEY1==0) return 1;

	}else if(KEY1==1)key_up=1; 	    
 	return 0;// 无按键按下
}
