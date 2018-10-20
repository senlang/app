#ifndef __USART_H
#define	__USART_H


#include "stm32f10x.h"
#include <stdio.h>

/** 
  * 串口宏定义，不同的串口挂载的总线和IO不一样，移植时需要修改这几个宏
  */

#define USART_DEBUG		USART3		//调试打印所使用的串口组


// 串口1-USART1
#define  UP_USARTx                   USART1
#define  UP_USART_CLK                RCC_APB2Periph_USART1
#define  UP_USART_APBxClkCmd         RCC_APB2PeriphClockCmd
#define  UP_USART_BAUDRATE           115200

//// USART GPIO 引脚宏定义
#define  UP_USART_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  UP_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  UP_USART_TX_GPIO_PORT         GPIOA   
#define  UP_USART_TX_GPIO_PIN          GPIO_Pin_9
#define  UP_USART_RX_GPIO_PORT       GPIOA
#define  UP_USART_RX_GPIO_PIN        GPIO_Pin_10

#define  UP_USART_IRQ                USART1_IRQn
//#define  UP_USART_IRQHandler         USART1_IRQHandler


// 串口2-USART2
#define  DOWN_USARTx                   USART2
#define  DOWN_USART_CLK                RCC_APB1Periph_USART2
#define  DOWN_USART_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  DOWN_USART_BAUDRATE           115200

//// USART GPIO 引脚宏定义
#define  DOWN_USART_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  DOWN_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
//    
#define  DOWN_USART_TX_GPIO_PORT         GPIOA   
#define  DOWN_USART_TX_GPIO_PIN          GPIO_Pin_2
#define  DOWN_USART_RX_GPIO_PORT       GPIOA
#define  DOWN_USART_RX_GPIO_PIN        GPIO_Pin_3

#define  DOWN_USART_IRQ                USART2_IRQn
//#define  DOWN_USART_IRQHandler         USART2_IRQHandler

// 串口3-USART3
#define  DEBUG_USARTx                   USART3
#define  DEBUG_USART_CLK                RCC_APB1Periph_USART3
#define  DEBUG_USART_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  DEBUG_USART_BAUDRATE           115200

// USART GPIO 引脚宏定义
#define  DEBUG_USART_GPIO_CLK           (RCC_APB2Periph_GPIOB)
#define  DEBUG_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  DEBUG_USART_TX_GPIO_PORT         GPIOB   
#define  DEBUG_USART_TX_GPIO_PIN          GPIO_Pin_10
#define  DEBUG_USART_RX_GPIO_PORT       GPIOB
#define  DEBUG_USART_RX_GPIO_PIN        GPIO_Pin_11

#define  DEBUG_USART_IRQ                USART3_IRQn
//#define  DEBUG_USART_IRQHandler         USART3_IRQHandler

// 串口4-UART4
//#define  DEBUG_USARTx                   UART4
//#define  DEBUG_USART_CLK                RCC_APB1Periph_UART4
//#define  DEBUG_USART_APBxClkCmd         RCC_APB1PeriphClockCmd
//#define  DEBUG_USART_BAUDRATE           115200

//// USART GPIO 引脚宏定义
//#define  DEBUG_USART_GPIO_CLK           (RCC_APB2Periph_GPIOC)
//#define  DEBUG_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
//    
//#define  DEBUG_USART_TX_GPIO_PORT         GPIOC   
//#define  DEBUG_USART_TX_GPIO_PIN          GPIO_Pin_10
//#define  DEBUG_USART_RX_GPIO_PORT       GPIOC
//#define  DEBUG_USART_RX_GPIO_PIN        GPIO_Pin_11

//#define  DEBUG_USART_IRQ                UART4_IRQn
//#define  DEBUG_USART_IRQHandler         UART4_IRQHandler


// 串口5-UART5
//#define  DEBUG_USARTx                   UART5
//#define  DEBUG_USART_CLK                RCC_APB1Periph_UART5
//#define  DEBUG_USART_APBxClkCmd         RCC_APB1PeriphClockCmd
//#define  DEBUG_USART_BAUDRATE           115200

//// USART GPIO 引脚宏定义
//#define  DEBUG_USART_GPIO_CLK           (RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD)
//#define  DEBUG_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
//    
//#define  DEBUG_USART_TX_GPIO_PORT         GPIOC   
//#define  DEBUG_USART_TX_GPIO_PIN          GPIO_Pin_12
//#define  DEBUG_USART_RX_GPIO_PORT       GPIOD
//#define  DEBUG_USART_RX_GPIO_PIN        GPIO_Pin_2

//#define  DEBUG_USART_IRQ                UART5_IRQn
//#define  DEBUG_USART_IRQHandler         UART5_IRQHandler



/*****************  发送一个字节 **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);

/****************** 发送8位的数组 ************************/
void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num);

/*****************  发送字符串 **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str);


/*****************  发送一个16位数 **********************/
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);










void Up_USART_Config(void);
void Up_Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void Up_Usart_SendString( USART_TypeDef * pUSARTx, char *str);
void Up_Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);


void Down_USART_Config(void);
void Down_Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void Down_Usart_SendString( USART_TypeDef * pUSARTx, char *str);
void Down_Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);

void Debug_USART_Config(void);
void Debug_Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void Debug_Usart_SendString( USART_TypeDef * pUSARTx, char *str);
void Debug_Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);



void Usart1_Init(unsigned int baud);
void Usart2_Init(unsigned int baud);



void UsartPrintf(USART_TypeDef *USARTx, char *fmt,...);


void UART1_IO_Send(unsigned char *str, unsigned short len);
int UART1_IO_Receive(void);



void UART2_IO_Send(unsigned char *str, unsigned short len);
int UART2_IO_Receive(void);

void UART1_IO_ClearRecive(void);
void UART2_IO_ClearRecive(void);

int uart1_receive_data(void);
int uart2_receive_data(void);


#endif /* __USART_H */
