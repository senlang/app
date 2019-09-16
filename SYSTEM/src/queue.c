#include "queue.h"
#include "malloc.h"
#include "stm32_protocol.h"
#include "usart.h"
#include "sys.h"

extern uint8_t  g_src_board_id;
extern uint32_t time_passes;

#ifdef USE_OS_MEM
extern OS_MEM *MemBuf;
#endif

#if 0
/**
 * ��ö���ʹ����
 * @param  queue
 * @return       ������ʹ����
 */
uint32_t Queue_GetUsed(const QueuePtr queue)
{
    int len = queue->write - queue->read;

    if(len >= 0)
        return (uint32_t)len;
    else
        return (queue->size + len);
}

/**
 * ��ö��п���ռ�
 * @param  queue
 * @return       ���п�����
 */
uint32_t Queue_GetFree(const QueuePtr queue)
{
    return queue->size - Queue_GetUsed(queue) - 1;
}


/**
 * �������Ƿ���
 * @param  queue
 * @return       1:��    0:����
 */
int Queue_isFull(const QueuePtr queue)
{
    if(queue->read == (queue->write + 1) % queue->size)
        return 1;
    else
        return 0;
}

/**
 * �������Ƿ�Ϊ��
 * @param  queue
 * @return       1:��    0:�ǿ�
 */
int Queue_isEmpty(const QueuePtr queue)
{
    if(queue->read == queue->write)
        return 1;
    else
        return 0;
}

/**
 * �������
 * @param  queue
 * @param  buf    Ҫ��ӵ�����
 * @param  length Ҫ��ӵ����ݳ���
 * @return        ������ӵ��ֽ���
 */
uint32_t Queue_Write(QueuePtr queue, const void* buf, uint32_t length)
{
    uint8_t *dataptr = (uint8_t *)buf;
    uint32_t offset = 0;
    uint32_t nwrite = 0;
    uint32_t nfree = 0;

    /* ��������ݳ���Ϊ0, ֱ�ӷ��� */
    if(!length)		return 0;

    /* ����û�пռ�, ֱ�ӷ��� */
    if(!(nfree = Queue_GetFree(queue)))		return 0;

    /* ����ʵ���ܹ���ӵ����ݳ��� */
    nwrite = nfree>=length ? length : nfree;

    /* �ж϶����Ƿ��β */
    offset = queue->size - queue->write;
    if (offset >= nwrite)
    {
        memcpy(queue->payload + queue->write, dataptr, nwrite);
        queue->write += nwrite;
    }
    else
    {
        memcpy(queue->payload + queue->write, dataptr, offset);
        memcpy(queue->payload, dataptr + offset, nwrite - offset);
        queue->write = nwrite - offset;
    }
    return nwrite;
}

/**
 * ���ݳ���
 * @param  queue
 * @param  buf    ��ų��ӵ�����
 * @param  length ���ӵ����ݳ���
 * @return        ���س����ֽ���
 */
uint32_t Queue_Read(QueuePtr queue, void* buf, uint32_t length)
{
    uint8_t *dataptr = (uint8_t *)buf;
    uint32_t offset = 0;
    uint32_t nused = 0;
    uint32_t nread = 0;

    /* �������ݳ���Ϊ0, ֱ�ӷ��� */
    if(!length)		return 0;

    /* ����ʵ���ܹ����ӵ����ݳ��� */
    if(!(nused = Queue_GetUsed(queue)))		return 0;

    /* ����ʵ���ܹ����������ݳ��� */
    nread = nused>=length ? length : nused;

    /* �ж�Ҫ���������Ƿ��β */
    offset = queue->size - queue->read;
    if( offset >= nread)
    {
        memcpy(dataptr, queue->payload + queue->read, nread);
        queue->read += nread;
    }
    else
    {
        memcpy(dataptr, queue->payload + queue->read, offset);
        memcpy(dataptr + offset, queue->payload, nread - offset);
        queue->read = nread - offset;
    }

    return nread;
}

/**
 * ��ʼ��һ������
 * @param queue
 * @param size ���д�С
 * @param payload ���л����ַ
 */
void Queue_Init(QueuePtr queue, uint32_t size, void* payload)
{
    queue->read = 0;
    queue->write = 0;
    queue->payload = (uint8_t *)payload;
    queue->size = size;
}

/**
 * �������
 * @param queue
 */
void Queue_Clear(QueuePtr queue)
{
    queue->read = queue->write;
}

/**
 * ��̬����һ������
 * @param  size ���д�С
 * @return      �ɹ����ض��ж���ָ��, ʧ�ܷ���NULL
 */
QueuePtr Queue_Create(uint32_t size)
{
    QueuePtr queue = NULL;

    if(!(queue = (QueuePtr)mymalloc(SRAMIN, sizeof(Queue))))
        return NULL;

    queue->size = size;
    queue->read = 0;
    queue->write = 0;
    queue->payload = NULL;

    if(!(queue->payload = (uint8_t *)mymalloc(SRAMIN, size)))
    {
        myfree(SRAMIN,queue);
        return NULL;
    }

    return queue;
}

/**
 * ���ڶ�̬�����Ķ��н���������
 * @param queue
 */
void Queue_Destory(QueuePtr queue)
{
    myfree(SRAMIN,queue->payload);
    myfree(SRAMIN,queue);
    queue = NULL;
}

/**
 * ��ָ���˺�len���ֽ�
 * @param queue
 * @param len Ҫ�˺���ֽ���
 */
void Queue_ErrBack(const QueuePtr queue, uint32_t len)
{
    int tmp = queue->read - len;
    queue->read = tmp < 0 ? ((uint32_t)(tmp + queue->size)):((uint32_t)tmp);
}

#endif

extern OS_EVENT *MsgMutex;
extern struct node* UartMsgNode;

//��������ڵ�
struct node* CreateNode(void)
{
	#ifdef USE_OS_MEM
	INT8U  err;	
	struct node* p = (struct node*)OSMemGet(MemBuf,  &err);
	#else
	struct node* p = (struct node*)mymalloc(SRAMIN, sizeof(struct node));
	#endif
	
	if(NULL == p)
	{
		return NULL;
	}
	memset(p,0,sizeof(struct node));
	p->pNext = NULL;

	UsartPrintf(USART_DEBUG, "CreateNode Node[%p]payload[%p]!!!\r\n", p, p->data.payload);
	return p;
}

//��������ڵ�
struct node* CreateMsgNode(void)
{
	#ifdef USE_OS_MEM
	INT8U  err; 
	struct node* p = (struct node*)OSMemGet(MemBuf,  &err);
	#else
	struct node* p = (struct node*)mymalloc(SRAMIN, sizeof(struct node));
	#endif
	
	if(NULL == p)
	{
		return NULL;
	}
	memset(p,0,sizeof(struct node));

	#ifdef USE_OS_MEM
	p->data.payload = (uint8_t *)OSMemGet(MemBuf,  &err);
	#else
	p->data.payload = (uint8_t *)mymalloc(SRAMIN, 16);
	#endif
	
	if(p->data.payload == NULL)
	{
		UsartPrintf(USART_DEBUG, "CreateMsgNode Payload malloc fail!!!\r\n");
		return NULL;
	}
	memset(p->data.payload, 0, 32);
	p->data.create_time = time_passes;

	UsartPrintf(USART_DEBUG, "CreateMsgNode Node[%p]payload[%p]time[%d]!!!\r\n", p, p->data.payload, p->data.create_time);
	
	p->pNext = NULL;

	return p;
}


//��ȡ����ڵ��ַ
struct node* GetNode(struct node *pHeader, uint16_t num)
{
	uint8_t i;
	struct node *p=pHeader;

	if(NULL == p->pNext)
	{
		return FAULSE;
	}

	for(i=0; i<num; i++)
	{
		if(p->pNext != NULL)//�ڵ��λ�ò��ô���������
		{
			p=p->pNext;
		}
		else
		{
			return FAULSE;
		}
	}

	
	UsartPrintf(USART_DEBUG, "GetNode Node[%p]payload[%p]!!!\r\n", p, p->data.payload);
	
	return p;
}







//��ȡ����ڵ��ַ
struct node* GetMsgNode(struct node *pHeader)
{
	struct node *p = pHeader;

	if(NULL == p->pNext)
	{
		return FAULSE;
	}
	
	if(p->pNext != NULL)//�ڵ��λ�ò��ô���������
	{
		p=p->pNext;
	}
	else
	{
		return FAULSE;
	}

	UsartPrintf(USART_DEBUG, "GetMsgNode Node[%p]payload[%p]!!!\r\n", p, p->data.payload);
	return p;
}

//����ڵ�
//�β� pHeader:����ͷ ?num = 0:ͷ TAIL:β�� !0&&!TAIL : �����м�
//����ֵ TURE:�ɹ� FAULSE:ʧ��
uint8_t InsertNode(struct node *pHeader, uint16_t num, struct node *pNewNode)
{
	uint8_t i;
	struct node *p=pHeader;
	uint8_t err = 0;
	//struct node *new =(struct node*)mymalloc(SRAMIN, sizeof(struct node));
	
	//�����ź���
	OSMutexPend(MsgMutex,0,&err);
	
	if(NULL == pNewNode)
	{
		return FAULSE;
	}
	if(0 == num)//��ͷ�����ӽڵ�
	{
		if(NULL == pHeader)
		{
			pHeader = pNewNode;
		}
		else
		{
			pHeader->pNext = pNewNode;
		}
	}
	else if (TAIL == num)//ĩβ���ӽڵ�
	{
		while(p->pNext != NULL)
		{
			p=p->pNext;
		}
		p->pNext = pNewNode;
	}
	else if(num > 0 && num != TAIL)//�������м����ӽڵ�
	{
		for(i=0;i<num-1;i++)
		{
			if(NULL != p->pNext)//���ӽڵ��λ�ò��ô���������
			{
				p=p->pNext;
			}
			else
			{
				return FAULSE;
			}
		}
		pNewNode->pNext = p->pNext;
		p->pNext = pNewNode;
	}

	//�ͷ��ź���
	OSMutexPost(MsgMutex);

	return TURE;
}


//ɾ������ڵ�
//�β� pHeader:����ͷ ?num = 0:ͷ TAIL:β�� !0&&!TAIL : �����м䣬ע��num����Ϊ0
//����ֵ TURE:�ɹ� FAULSE:ʧ��
uint8_t DeleNode(struct node *pHeader, uint16_t num)
{
	struct node *p=pHeader,*p1,*p2;

	if(NULL == p)//������������
	{
		return FAULSE;
	}

	if(0 == num)//��ͷ��ɾ���ڵ�
	{
		p1 = pHeader;
		pHeader = pHeader->pNext;
		
		#ifdef USE_OS_MEM
		OSMemPut(MemBuf, p1->data.payload);
		OSMemPut(MemBuf, p1);
		#else
		myfree(SRAMIN, p1->data.payload);
		myfree(SRAMIN, p1);
		#endif
		UsartPrintf(USART_DEBUG, "DeleNode Node head[%p]payload[%p]!!!\r\n", p1, p1->data.payload);
	}
	
	if (TAIL == num)//ĩβɾ���ڵ�
	{
		while(p->pNext != NULL)
		{
			p1=p;//��ȡ�µ�β��
			p=p->pNext;//ָ����һ���ڵ�
		}
		p1->pNext = NULL;
		
		#ifdef USE_OS_MEM
		OSMemPut(MemBuf, p->data.payload);
		OSMemPut(MemBuf, p);
		#else
		myfree(SRAMIN, p->data.payload);
		myfree(SRAMIN, p);
		#endif
		
		UsartPrintf(USART_DEBUG, "DeleNode Node tail[%p]payload[%p]!!!\r\n", p, p->data.payload);
	}
	else if(num > 0 && num != TAIL)//�������м�ɾ���ڵ�
	{
		p1 = p->pNext;
		p2 = p1->pNext;
		p->pNext = p2;
		
		#ifdef USE_OS_MEM
		OSMemPut(MemBuf, p1->data.payload);
		OSMemPut(MemBuf, p1);
		#else
		myfree(SRAMIN, p1->data.payload);
		myfree(SRAMIN, p1);
		#endif
		
		UsartPrintf(USART_DEBUG, "DeleNode Node mid[%p]payload[%p]!!!\r\n", p1, p1->data.payload);
	}
	return TURE;
}


//��ȡ������
uint16_t GetNodeNum(struct node *pHeader)
{
	uint16_t ret,i;
	struct node *p = pHeader;

	if(NULL == p)//������
	{
		return FAULSE;
	}

	for(i = 0; p->pNext != NULL; i++)
	{
		p = p->pNext;
	}
	ret = i;

	//UsartPrintf(USART_DEBUG, "GetNodeNum[%d]!!!\r\n", ret);
	
	return ret;
}







void MessageDealQueueCreate(void)
{
	uint8_t err = 0;
	
	MsgMutex = OSMutexCreate(0, &err);
	UsartPrintf(USART_DEBUG, "MsgMutex Create [%d]!!!\r\n", err);

	UartMsgNode = CreateNode();
	UsartPrintf(USART_DEBUG, "UartMsgNode[0x%p]!!!\r\n", UartMsgNode);
	
}


void MessageAckCheck(unsigned char *pdata, uint16_t size)
{		
	uint16_t node_num = 0;
	uint8_t err = 0;
	uint8_t i = 0, j = 0, node_i = 0;

	ack_struct  ack_msg;
	uart_msg_struct cmd_msg;
	struct node* MsgNode = NULL;
	struct node* NewMsgNode = NULL;
	
	memset((unsigned char*)&ack_msg, 0x00, sizeof(ack_struct));
	memset((unsigned char*)&cmd_msg, 0x00, sizeof(uart_msg_struct));
	memcpy((unsigned char*)&ack_msg, (unsigned char*)pdata, size);

	/*ֻ��Գ�����ɡ��������󣬲�������ش�*/
	if((ack_msg.pl_cmd_type != CMD_PUSH_MEDICINE_COMPLETE) && 
		(ack_msg.pl_cmd_type !=CMD_MCU_ADD_MEDICINE_COMPLETE) && 
		(ack_msg.pl_cmd_type !=CMD_PUSH_MEDICINE_REQUEST) &&
		(ack_msg.pl_cmd_type !=CMD_REPLENISH_MEDICINE_REQUEST) &&
		(ack_msg.pl_cmd_type !=CMD_TRACK_RUNTIME_REPORT))
		
	return;

	//�����ź���
	OSMutexPend(MsgMutex,0,&err);
	
	/*��Ϣ����ȡ��Ϣ*/
	node_num = GetNodeNum(UartMsgNode);
	UsartPrintf(USART_DEBUG, "MessageAckCheck node_num[%d]!!!\r\n", node_num);
	if(node_num == 0)
	{
		UsartPrintf(USART_DEBUG, "MessageAckQueue is empty!!!\r\n", node_num);
		goto FINISH_MSGCHK;
	}
	
	MsgNode = UartMsgNode;
	for(i = 1, node_i = 1; i <= node_num; i++)
	{
		//NewMsgNode = GetMsgNode(MsgNode);
		NewMsgNode = GetNode(UartMsgNode, node_i);
		
		UsartPrintf(USART_DEBUG, "Node[%d/%d]node_p[%d]!!!\r\n", i, node_num, node_i);
		
		if(NewMsgNode)
		{
			UsartPrintf(USART_DEBUG, "NewMsgNode [%d]!!!\r\n", NewMsgNode->data.size);
			for(j = 0; j < NewMsgNode->data.size; j++)
			{
				UsartPrintf(USART_DEBUG, "0x%02x, ", NewMsgNode->data.payload[j]);
			}
			UsartPrintf(USART_DEBUG, "\r\n");
			memcpy(&cmd_msg, NewMsgNode->data.payload, NewMsgNode->data.size);
			
			UsartPrintf(USART_DEBUG, "board_id:0x%02x, 0x%02x. cmd_type: 0x%02x, 0x%02x\r\n", 
				ack_msg.pl_board_id, cmd_msg.board_id, ack_msg.pl_cmd_type, cmd_msg.cmd_type);
			
			if((ack_msg.pl_board_id == cmd_msg.board_id) && (ack_msg.pl_cmd_type == cmd_msg.cmd_type))
			{
				UsartPrintf(USART_DEBUG, "Receive ack message, DeleteNode[%d/%d]!!!\r\n", i, node_num);
				if(i == node_num)
				DeleNode(MsgNode, TAIL);
				else
				DeleNode(MsgNode, i);
				
				UsartPrintf(USART_DEBUG, "Remain Node:%d!!!\r\n", GetNodeNum(UartMsgNode));
				break;
			}
			else if((g_src_board_id == 1) && 
				(ack_msg.pl_board_id == 1) &&
				((cmd_msg.board_id == 0xff)||(cmd_msg.board_id == 0xfe)) && 
				(ack_msg.pl_cmd_type == cmd_msg.cmd_type))
			{
				UsartPrintf(USART_DEBUG, "Receive Andriod ack message, Will Delete From queue!!!\r\n");
				if(i == node_num)
				DeleNode(MsgNode, TAIL);
				else
				DeleNode(MsgNode, i);
				UsartPrintf(USART_DEBUG, "Remain Node:%d!!!\r\n", GetNodeNum(UartMsgNode));
				break;
			}
			else
			{
				node_i++;
				MsgNode = NewMsgNode;
			}
		}
	}

	FINISH_MSGCHK:
	//�ͷ��ź���
	OSMutexPost(MsgMutex);
}

void MessageInsertQueue(unsigned char *pdata, uint16_t size, uint8_t uart_idx)
{
	struct node* Uart1MsgNode = NULL;
	
	Uart1MsgNode = CreateMsgNode();
	UsartPrintf(USART_DEBUG, " MessageInsertQueue malloc Uart1MsgNode = 0x%p\r\n", Uart1MsgNode);
	
	if(Uart1MsgNode == NULL)
	return;

	Uart1MsgNode->data.size = size;
	Uart1MsgNode->data.uart_idx = uart_idx;
	memcpy(Uart1MsgNode->data.payload, pdata, size);
	
	InsertNode(UartMsgNode, TAIL, Uart1MsgNode);
}


void NotRetryMessageInsertQueue(unsigned char *pdata, uint16_t size, uint8_t uart_idx)
{
	struct node* Uart1MsgNode = NULL;
	
	Uart1MsgNode = CreateMsgNode();
	UsartPrintf(USART_DEBUG, " MessageInsertQueue malloc Uart1MsgNode = 0x%p\r\n", Uart1MsgNode);
	
	if(Uart1MsgNode == NULL)
	return;

	Uart1MsgNode->data.size = size;
	Uart1MsgNode->data.uart_idx = uart_idx;
	Uart1MsgNode->data.times = 0xff;
	
	memcpy(Uart1MsgNode->data.payload, pdata, size);
	
	InsertNode(UartMsgNode, TAIL, Uart1MsgNode);
}




