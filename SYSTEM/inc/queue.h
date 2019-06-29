#ifndef __QUEUE_H_
#define __QUEUE_H_

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus


#include <stdint.h>
#include <string.h>
#include <malloc.h>

typedef struct queue
{
    uint32_t read;
    uint32_t write;
    uint32_t size;
    uint8_t* payload;
}Queue, *QueuePtr;


uint32_t Queue_GetUsed(const QueuePtr queue);
uint32_t Queue_GetFree(const QueuePtr queue);
uint32_t Queue_Write(QueuePtr queue, const void* buf, uint32_t length);
uint32_t Queue_Read(QueuePtr queue, void* buf, uint32_t length);
void Queue_Clear(QueuePtr queue);
void Queue_ErrBack(QueuePtr queue, uint32_t len);
void Queue_Init(QueuePtr queue, uint32_t size, void* payload);
void Queue_Destory(QueuePtr queue);
int Queue_isFull(const QueuePtr queue);
int Queue_isEmpty(const QueuePtr queue);
QueuePtr Queue_Create(uint32_t size);

#if 1

#ifndef TURE
#define TURE 1
#endif


#ifndef FAULSE
#define FAULSE 0
#endif


#ifndef TAIL
#define TAIL 0xffff
#endif


#define SEND_FAIL	0	//消息发送失败
#define SEND_SUCCESS  1	//消息发送成功

#define UART1_IDX	0	//由UART1 发送消息
#define UART2_IDX  1	//由UART2 发送消息

#define MAX_PAYLOAD_LEN 16 //byte

//定义一个结构体节点
typedef struct msgqueue
{
    uint8_t size;
    uint8_t status;
    uint8_t uart_idx;
    uint8_t *payload;
}MsgQueue, *MsgQueuePtr;

struct node
{
	MsgQueue data; 		//有效数据
	struct node *pNext; //指向下一个节点的指针
};



//创建链表节点
struct node* CreateNode(void);

//获取链表节点地址
struct node* GetNode(struct node *pHeader, uint16_t num);

//插入节点
uint8_t InsertNode(struct node *pHeader, uint16_t num, struct node *pNewNode);

//删除链表节点
uint8_t DeleNode(struct node *pHeader, uint16_t num);

//获取链表长度
uint16_t GetNodeNum(struct node *pHeader);

//创建消息节点
struct node* CreateMsgNode(void);

//获取消息节点
struct node* GetMsgNode(struct node *pHeader);



void MessageDealQueueCreate(void);

void MessageAckCheck(unsigned char *pdata, uint16_t size);

void MessageInsertQueue(unsigned char *pdata, uint16_t size, uint8_t uart_idx);




#endif








#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __QUEUE_H_

