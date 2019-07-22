#include "queue.h"
#include "malloc.h"
#include "stm32_protocol.h"
#include "usart.h"

extern uint8_t  g_src_board_id;

/**
 * 获得队列使用量
 * @param  queue
 * @return       返回已使用量
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
 * 获得队列空余空间
 * @param  queue
 * @return       队列空余数
 */
uint32_t Queue_GetFree(const QueuePtr queue)
{
    return queue->size - Queue_GetUsed(queue) - 1;
}


/**
 * 检测队列是否满
 * @param  queue
 * @return       1:满    0:不满
 */
int Queue_isFull(const QueuePtr queue)
{
    if(queue->read == (queue->write + 1) % queue->size)
        return 1;
    else
        return 0;
}

/**
 * 检测队列是否为空
 * @param  queue
 * @return       1:空    0:非空
 */
int Queue_isEmpty(const QueuePtr queue)
{
    if(queue->read == queue->write)
        return 1;
    else
        return 0;
}

/**
 * 数据入队
 * @param  queue
 * @param  buf    要入队的数据
 * @param  length 要入队的数据长度
 * @return        返回入队的字节数
 */
uint32_t Queue_Write(QueuePtr queue, const void* buf, uint32_t length)
{
    uint8_t *dataptr = (uint8_t *)buf;
    uint32_t offset = 0;
    uint32_t nwrite = 0;
    uint32_t nfree = 0;

    /* 传入的数据长度为0, 直接返回 */
    if(!length)		return 0;

    /* 队列没有空间, 直接返回 */
    if(!(nfree = Queue_GetFree(queue)))		return 0;

    /* 计算实际能够入队的数据长度 */
    nwrite = nfree>=length ? length : nfree;

    /* 判断队列是否跨尾 */
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
 * 数据出队
 * @param  queue
 * @param  buf    存放出队的数据
 * @param  length 出队的数据长度
 * @return        返回出队字节数
 */
uint32_t Queue_Read(QueuePtr queue, void* buf, uint32_t length)
{
    uint8_t *dataptr = (uint8_t *)buf;
    uint32_t offset = 0;
    uint32_t nused = 0;
    uint32_t nread = 0;

    /* 出队数据长度为0, 直接返回 */
    if(!length)		return 0;

    /* 计算实际能够出队的数据长度 */
    if(!(nused = Queue_GetUsed(queue)))		return 0;

    /* 计算实际能够读到的数据长度 */
    nread = nused>=length ? length : nused;

    /* 判断要读的数据是否跨尾 */
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
 * 初始化一个队列
 * @param queue
 * @param size 队列大小
 * @param payload 队列缓存地址
 */
void Queue_Init(QueuePtr queue, uint32_t size, void* payload)
{
    queue->read = 0;
    queue->write = 0;
    queue->payload = (uint8_t *)payload;
    queue->size = size;
}

/**
 * 清理队列
 * @param queue
 */
void Queue_Clear(QueuePtr queue)
{
    queue->read = queue->write;
}

/**
 * 动态创建一个队列
 * @param  size 队列大小
 * @return      成功返回队列对象指针, 失败返回NULL
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
 * 对于动态创建的队列进行清理工作
 * @param queue
 */
void Queue_Destory(QueuePtr queue)
{
    myfree(SRAMIN,queue->payload);
    myfree(SRAMIN,queue);
    queue = NULL;
}

/**
 * 读指针退后len个字节
 * @param queue
 * @param len 要退后的字节数
 */
void Queue_ErrBack(const QueuePtr queue, uint32_t len)
{
    int tmp = queue->read - len;
    queue->read = tmp < 0 ? ((uint32_t)(tmp + queue->size)):((uint32_t)tmp);
}



#if 1
extern OS_EVENT *MsgMutex;
extern struct node* UartMsgNode;

//创建链表节点
struct node* CreateNode(void)
{
	struct node* p = (struct node*)mymalloc(SRAMIN, sizeof(struct node));
	if(NULL == p)
	{
		return NULL;
	}

	memset(p,0,sizeof(struct node));
	p->pNext = NULL;

	return p;
}

//创建链表节点
struct node* CreateMsgNode(void)
{
	struct node* p = (struct node*)mymalloc(SRAMIN, sizeof(struct node));
	if(NULL == p)
	{
		return NULL;
	}
	memset(p,0,sizeof(struct node));
	p->data.payload = (uint8_t *)mymalloc(SRAMIN, 16);
	p->pNext = NULL;

	return p;
}


//获取链表节点地址
struct node* GetNode(struct node *pHeader, uint16_t num)
{
	uint8_t i;
	struct node *p=pHeader;

	if(NULL == p->pNext)
	{
		return FAULSE;
	}

	for(i=0;i<num;i++)
	{
		if(p->pNext != NULL)//节点的位置不得大于链表长度
		{
			p=p->pNext;
		}
		else
		{
			return FAULSE;
		}
	}
	return p;
}







//获取链表节点地址
struct node* GetMsgNode(struct node *pHeader)
{
	struct node *p = pHeader;

	if(NULL == p->pNext)
	{
		return FAULSE;
	}
	
	if(p->pNext != NULL)//节点的位置不得大于链表长度
	{
		p=p->pNext;
	}
	else
	{
		return FAULSE;
	}
	return p;
}

//插入节点
//形参 pHeader:链表头 ?num = 0:头 TAIL:尾部 !0&&!TAIL : 链表中间
//返回值 TURE:成功 FAULSE:失败
uint8_t InsertNode(struct node *pHeader, uint16_t num, struct node *pNewNode)
{
	uint8_t i;
	struct node *p=pHeader;
	uint8_t err = 0;
	//struct node *new =(struct node*)mymalloc(SRAMIN, sizeof(struct node));
	
	//请求信号量
	OSMutexPend(MsgMutex,0,&err);
	
	if(NULL == pNewNode)
	{
		return FAULSE;
	}
	if(0 == num)//在头部增加节点
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
	else if (TAIL == num)//末尾增加节点
	{
		while(p->pNext != NULL)
		{
			p=p->pNext;
		}
		p->pNext = pNewNode;
	}
	else if(num > 0 && num != TAIL)//在链表中间增加节点
	{
		for(i=0;i<num-1;i++)
		{
			if(NULL != p->pNext)//增加节点的位置不得大于链表长度
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

	//释放信号量
	OSMutexPost(MsgMutex);

	return TURE;
}


//删除链表节点
//形参 pHeader:链表头 ?num = 0:头 TAIL:尾部 !0&&!TAIL : 链表中间，注意num不能为0
//返回值 TURE:成功 FAULSE:失败
uint8_t DeleNode(struct node *pHeader, uint16_t num)
{
	uint8_t i;
	struct node *p=pHeader,*p1,*p2;

	if(NULL == p)//空链表，无意义
	{
		return FAULSE;
	}

	// if(0 == num)//在头部删除节点
	// {
	// 	p1 = pHeader;
	//	pHeader = pHeader->pNext;
	// 	myfree(SRAMIN, p1);
	// }
	
	if (TAIL == num)//末尾删除节点
	{
		while(p->pNext != NULL)
		{
			p1=p;//获取新的尾巴
			p=p->pNext;//指向下一个节点
		}
		p1->pNext = NULL;
		
		myfree(SRAMIN, p1->data.payload);
		myfree(SRAMIN, p);
	}
	else if(num > 0 && num != TAIL)//在链表中间删除节点
	{
		p1 = p->pNext;
		p2 = p1->pNext;
		p->pNext = p2;
		myfree(SRAMIN, p1->data.payload);
		myfree(SRAMIN, p1);
	}
	return TURE;
}


//获取链表长度
uint16_t GetNodeNum(struct node *pHeader)
{
	uint16_t ret,i;
	struct node *p = pHeader;

	if(NULL == p)//空链表
	{
		return FAULSE;
	}

	for(i = 0; p->pNext != NULL; i++)
	{
		p = p->pNext;
	}
	ret = i;
	return ret;
}

#endif






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
	uint8_t i = 0, j = 0;

	ack_struct  ack_msg;
	uart_msg_struct cmd_msg;
	struct node* MsgNode = NULL;
	struct node* NewMsgNode = NULL;
	
	memset((unsigned char*)&ack_msg, 0x00, sizeof(ack_struct));
	memset((unsigned char*)&cmd_msg, 0x00, sizeof(uart_msg_struct));
	memcpy((unsigned char*)&ack_msg, (unsigned char*)pdata, size);

	//请求信号量
	OSMutexPend(MsgMutex,0,&err);
	
	/*消息队列取消息*/
	node_num = GetNodeNum(UartMsgNode);
	UsartPrintf(USART_DEBUG, "MessageAckCheck node_num[%d]!!!\r\n", node_num);
	if(node_num == 0)
	{
		UsartPrintf(USART_DEBUG, "MessageAckQueue is empty!!!\r\n", node_num);
		goto FINISH_MSGCHK;
	}
	
	MsgNode = UartMsgNode;
	for(i = 1; i <= node_num; i++)
	{
		NewMsgNode = GetMsgNode(MsgNode);
		UsartPrintf(USART_DEBUG, "Node[%d/%d]!!!\r\n", i, node_num);
		
		if(NewMsgNode)
		{
			UsartPrintf(USART_DEBUG, "NewMsgNode[%d]!!!\r\n", NewMsgNode->data.size);
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
				UsartPrintf(USART_DEBUG, "Receive ack message, Will Delete From queue!!!\r\n");
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
				MsgNode = NewMsgNode;
			}
		}
	}

	FINISH_MSGCHK:
	//释放信号量
	OSMutexPost(MsgMutex);
}

void MessageInsertQueue(unsigned char *pdata, uint16_t size, uint8_t uart_idx)
{
	struct node* Uart1MsgNode = NULL;
	
	Uart1MsgNode = CreateMsgNode();
	UsartPrintf(USART_DEBUG, "Uart1MsgNode = 0x%p\r\n\r\n", Uart1MsgNode);

	Uart1MsgNode->data.size = size;
	Uart1MsgNode->data.uart_idx = uart_idx;
	memcpy(Uart1MsgNode->data.payload, pdata, size);
	
	InsertNode(UartMsgNode, TAIL, Uart1MsgNode);
}




