#ifndef _DHT12_H_
#define _DHT12_H_
#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"
#include "stdio.h"

//IO方向设置
#define SDA_IN()  {GPIOC->CRL&=0XFFFFF0FF;GPIOC->CRL|=0x00000800;}//上下拉输入
#define SDA_OUT() {GPIOC->CRL&=0XFFFFF0FF;GPIOC->CRL|=0x00000300;}//通用推挽输出

//IO操作函数
#define SEND_SDA   PCout(2) //SDA输出	 
#define READ_SDA   PCin(2)  //SDA输入 

void DHT12_Init(void);//初始化SDA(PB14),CLK(PB13)
u8 DHT12_Rdata(void);
u8 DHT12_READ(int *Temprature, int *Humi);

#endif	//_DHT12_H_
