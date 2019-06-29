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


#define SEND_FAIL	0	//��Ϣ����ʧ��
#define SEND_SUCCESS  1	//��Ϣ���ͳɹ�

#define UART1_IDX	0	//��UART1 ������Ϣ
#define UART2_IDX  1	//��UART2 ������Ϣ

#define MAX_PAYLOAD_LEN 16 //byte

//����һ���ṹ��ڵ�
typedef struct msgqueue
{
    uint8_t size;
    uint8_t status;
    uint8_t uart_idx;
    uint8_t *payload;
}MsgQueue, *MsgQueuePtr;

struct node
{
	MsgQueue data; 		//��Ч����
	struct node *pNext; //ָ����һ���ڵ��ָ��
};



//��������ڵ�
struct node* CreateNode(void);

//��ȡ����ڵ��ַ
struct node* GetNode(struct node *pHeader, uint16_t num);

//����ڵ�
uint8_t InsertNode(struct node *pHeader, uint16_t num, struct node *pNewNode);

//ɾ������ڵ�
uint8_t DeleNode(struct node *pHeader, uint16_t num);

//��ȡ������
uint16_t GetNodeNum(struct node *pHeader);

//������Ϣ�ڵ�
struct node* CreateMsgNode(void);

//��ȡ��Ϣ�ڵ�
struct node* GetMsgNode(struct node *pHeader);



void MessageDealQueueCreate(void);

void MessageAckCheck(unsigned char *pdata, uint16_t size);

void MessageInsertQueue(unsigned char *pdata, uint16_t size, uint8_t uart_idx);




#endif








#ifdef __cplusplus
}
#endif // __cplusplus

#endif // __QUEUE_H_

