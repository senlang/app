/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	
	*
	*	作者： 		
	*
	*	日期： 		
	*
	*	版本： 		
	*
	*	说明： 
	*
	*	修改记录：	
	************************************************************
	************************************************************
	************************************************************
**/

#include "stm32f10x.h"	//单片机头文件

#include "stm32_protocol.h"
#include "data_io.h"
#include "stm32_uart1.h"
#include "stm32_uart2.h"
#include "usart.h"
#include "motor.h"


//硬件驱动
#include "delay.h"
#include "usart.h"

//C库
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "ucos_ii.h"




static uint8_t  g_src_board_id = 0;  
static uint8_t g_board_status = 0;  
static uint8_t g_error_code = 0;  
static uint8_t test[2] = {1,2};  

unsigned char uart1_shared_rx_buf[MAX_PAYLOAD_LEN];  



static uint32_t buf_bitmap = 0;  
  
static unsigned char status_report_request_buf[STATUS_REPORT_REQUEST_PACKET_SIZE];  
static unsigned char push_medicine_request_buf[PUSH_MEDICINE_REQUEST_PACKET_SIZE];  


static unsigned char replenish_medicine_request_buf[REPLENISH_MEDICINE_REQUEST_PACKET_SIZE];  
static unsigned char board_test_buf[BOARD_TEST_REQUEST_PACKET_SIZE]; 
static unsigned char message_ack_buf[COMMAND_ACK_PACKET_SIZE]; 












static unsigned char txr_buf[MAX_PAYLOAD_LEN + 32];  




struct push_medicine_request_info_struct push_srtuct[TOTAL_PUSH_CNT];
int enqueue_push_index = 0;
int dequeue_push_index = 0;


struct replenish_medicine_request_info_struct replenish_srtuct[TOTAL_PUSH_CNT];

int enqueue_replenish_index = 0;
int dequeue_replenish_index = 0;


extern OS_EVENT *SemOfMotor;          //Motor控制信号量



void BoardId_Init(void)
{
	
	GPIO_InitTypeDef gpioInitStrcut;

	//使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOB, ENABLE);
	
	//IO配置
	gpioInitStrcut.GPIO_Mode = GPIO_Mode_IPU;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_8;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	//IO初始化
	GPIO_Init(GPIOB, &gpioInitStrcut);

	//IO配置
	gpioInitStrcut.GPIO_Mode = GPIO_Mode_IPU;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_13;
	//IO初始化
	GPIO_Init(GPIOC, &gpioInitStrcut);

	g_src_board_id = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)<<3 | GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)<<2 | 
		GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)<<1 | GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);


	
	UsartPrintf(USART_DEBUG, "g_src_board_id:0x%x\r\n", g_src_board_id); 
}



/*  
    累加校验和算法  
 */  
 static unsigned char add_checksum (unsigned char *buf, unsigned int len)  
{  
    unsigned int i;  
    unsigned char checksum = 0;  
  
    for (i = 0; i < len; ++i)  
    {  
        checksum += *(buf++);  
    }  
  
    return checksum;  
}  

void send_command_ack( void *input_data)  
{  
	uint8_t send_cmd_ack_data[COMMAND_ACK_PACKET_SIZE];
	struct cmd_ack_info_struct *cmd_ack_info = (struct cmd_ack_info_struct * )input_data;

	memset(send_cmd_ack_data, 0x00, COMMAND_ACK_PACKET_SIZE);
	
	send_cmd_ack_data[0] = START_CODE;
	send_cmd_ack_data[1] = COMMAND_ACK_PACKET_SIZE - 1;
	send_cmd_ack_data[2] = CMD_CMD_ACK;

	send_cmd_ack_data[3] = cmd_ack_info->board_id;
	
	send_cmd_ack_data[4] = cmd_ack_info->rsp_cmd_type;
	
	send_cmd_ack_data[5] = cmd_ack_info->status;

	send_cmd_ack_data[COMMAND_ACK_PACKET_SIZE - 1] = add_checksum(send_cmd_ack_data, COMMAND_ACK_PACKET_SIZE - 1);  
	
	UART1_IO_Send(send_cmd_ack_data, COMMAND_ACK_PACKET_SIZE);  
} 


void send_status_report_request(void)  
{  
    uint8_t send_statu_report_request_data[STATUS_REPORT_REQUEST_PACKET_SIZE];  

	memset(send_statu_report_request_data, 0x00, STATUS_REPORT_REQUEST_PACKET_SIZE);
	
	send_statu_report_request_data[0] = START_CODE;
	send_statu_report_request_data[1] = STATUS_REPORT_REQUEST_PACKET_SIZE;
	send_statu_report_request_data[2] = CMD_STATUS_REPORT_REQUEST;
	
	send_statu_report_request_data[3] = g_src_board_id;
	send_statu_report_request_data[4] = g_board_status;
	send_statu_report_request_data[5] = g_error_code;
	
	send_statu_report_request_data[6] = test[0];
	send_statu_report_request_data[7] = test[1];

	
    send_statu_report_request_data[8] = add_checksum(send_statu_report_request_data, STATUS_REPORT_REQUEST_PACKET_SIZE);  
	//UART1_IO_Send(send_statu_report_request_data, STATUS_REPORT_REQUEST_PACKET_SIZE);  
	
	UART2_IO_Send(send_statu_report_request_data, STATUS_REPORT_REQUEST_PACKET_SIZE);  
}  



void send_push_medicine_request( void *input_data)  
{  
	int i = 0;
	uint8_t send_push_medicine_request_data[PUSH_MEDICINE_REQUEST_PACKET_SIZE];
	struct push_medicine_paramter *push_medicine_info = (struct push_medicine_paramter * )input_data;


	memset(send_push_medicine_request_data, 0x00, PUSH_MEDICINE_REQUEST_PACKET_SIZE);
	
	send_push_medicine_request_data[0] = START_CODE;
	send_push_medicine_request_data[1] = IPUC + PUSH_MEDICINE_REQUEST_INFO_SIZE * push_medicine_info->push_cnt;
	send_push_medicine_request_data[2] = CMD_PUSH_MEDICINE_REQUEST;

	for(i = 0; i < push_medicine_info->push_cnt; i++)
	{
		send_push_medicine_request_data[3 + i * PUSH_MEDICINE_REQUEST_INFO_SIZE] = push_medicine_info->info[i].board_id;
		
		send_push_medicine_request_data[4 + i * PUSH_MEDICINE_REQUEST_INFO_SIZE] = push_medicine_info->info[i].medicine_track_number;
		
		send_push_medicine_request_data[5 + i * PUSH_MEDICINE_REQUEST_INFO_SIZE] = (push_medicine_info->info[i].push_time& 0xff00)>>8;
		send_push_medicine_request_data[6 + i * PUSH_MEDICINE_REQUEST_INFO_SIZE] = push_medicine_info->info[i].push_time;
	}
	send_push_medicine_request_data[IPUC + PUSH_MEDICINE_REQUEST_INFO_SIZE * push_medicine_info->push_cnt] = add_checksum(send_push_medicine_request_data, IPUC + PUSH_MEDICINE_REQUEST_INFO_SIZE * push_medicine_info->push_cnt);  

	UART2_IO_Send(send_push_medicine_request_data, IPUC + PUSH_MEDICINE_REQUEST_INFO_SIZE * push_medicine_info->push_cnt + CHECKSUM_SIZE);  
} 





void send_replinish_medicine_request( void *input_data)  
{  
	int i = 0;
	uint8_t send_replinish_medicine_request_data[REPLENISH_MEDICINE_REQUEST_PACKET_SIZE];
	struct replenish_medicine_paramter *replenish_medicine_info = (struct replenish_medicine_paramter * )input_data;

	memset(replenish_medicine_info, 0x00, REPLENISH_MEDICINE_REQUEST_PACKET_SIZE);
	
	send_replinish_medicine_request_data[0] = START_CODE;
	send_replinish_medicine_request_data[1] = IPUC + REPLENISH_MEDICINE_REQUEST_INFO_SIZE * replenish_medicine_info->push_cnt;
	send_replinish_medicine_request_data[2] = CMD_REPLENISH_MEDICINE_REQUEST;

	for(i = 0; i < replenish_medicine_info->push_cnt; i++)
	{
		send_replinish_medicine_request_data[3 + i * REPLENISH_MEDICINE_REQUEST_INFO_SIZE] = replenish_medicine_info->info[i].board_id;
		
		send_replinish_medicine_request_data[4 + i * REPLENISH_MEDICINE_REQUEST_INFO_SIZE] = replenish_medicine_info->info[i].medicine_track_number;
		
		send_replinish_medicine_request_data[5 + i * REPLENISH_MEDICINE_REQUEST_INFO_SIZE] = (replenish_medicine_info->info[i].push_time& 0xff00)>>8;
		send_replinish_medicine_request_data[6 + i * REPLENISH_MEDICINE_REQUEST_INFO_SIZE] = replenish_medicine_info->info[i].push_time;
	}
	send_replinish_medicine_request_data[IPUC + REPLENISH_MEDICINE_REQUEST_INFO_SIZE * replenish_medicine_info->push_cnt] = add_checksum(send_replinish_medicine_request_data, IPUC + REPLENISH_MEDICINE_REQUEST_INFO_SIZE * replenish_medicine_info->push_cnt);  

	UART2_IO_Send(send_replinish_medicine_request_data, IPUC + REPLENISH_MEDICINE_REQUEST_INFO_SIZE * replenish_medicine_info->push_cnt + CHECKSUM_SIZE);  
} 






/*单板测试*/
void send_board_test_request(void *input_data)  
{  
	int i = 0;
	uint8_t send_board_test_request_data[BOARD_TEST_REQUEST_PACKET_SIZE];
	struct test_request_info_struct *test_info = (struct test_request_info_struct *)input_data;

	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
	
	memset(send_board_test_request_data, 0x00, BOARD_TEST_REQUEST_PACKET_SIZE);
	
	send_board_test_request_data[0] = START_CODE;
	send_board_test_request_data[1] = BOARD_TEST_REQUEST_PACKET_SIZE - 1;
	send_board_test_request_data[2] = CMD_TEST_REQUEST;

	send_board_test_request_data[3] = test_info->board_id;
	
	send_board_test_request_data[4] = test_info->test_mode;
	
	send_board_test_request_data[5] = test_info->medicine_track_number;
	send_board_test_request_data[6] = (test_info->test_time & 0xff00)>>8;
	send_board_test_request_data[7] = (test_info->test_time & 0x00ff);


	UsartPrintf(USART_DEBUG, "time[0x%02x][0x%02x]\r\n", send_board_test_request_data[6], send_board_test_request_data[7]);


	send_board_test_request_data[BOARD_TEST_REQUEST_PACKET_SIZE - 1] = add_checksum(send_board_test_request_data, BOARD_TEST_REQUEST_PACKET_SIZE - 1);  

	UART2_IO_Send(send_board_test_request_data, BOARD_TEST_REQUEST_PACKET_SIZE);  

	
	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
} 



void push_test(void)
{
	int i = 0;
	struct push_medicine_paramter push_paramter;
	
	push_paramter.push_cnt = 3;

	for(i = 0; i < push_paramter.push_cnt; i++)
	{
		push_paramter.info[i].board_id = 0x0001;
		push_paramter.info[i].medicine_track_number = 16 + i;
		push_paramter.info[i].push_time = 80 + i * 10;
	}

	send_push_medicine_request((void *)&push_paramter);
}



void replenish_test(void)
{
	int i = 0;
	struct replenish_medicine_paramter replenish_paramter;
	
	replenish_paramter.push_cnt = 3;

	for(i = 0; i < replenish_paramter.push_cnt; i++)
	{
		replenish_paramter.info[i].board_id = 0x0001;
		replenish_paramter.info[i].medicine_track_number = 1 + i;
		replenish_paramter.info[i].push_time = 80 + i * 10;
	}

	send_replinish_medicine_request((void *)&replenish_paramter);
}

void test_test(void)
{
	int i = 0;
	struct test_request_info_struct test_request_info;
	
	test_request_info.board_id = 0x01;
	test_request_info.test_mode = MOTOR_RUN_FORWARD;
	test_request_info.medicine_track_number = 0x01;
	test_request_info.test_time = 200;


	send_board_test_request((void *)&test_request_info);
}



int board_send_message(int msg_type, void *input_data)
{
	switch(msg_type)
	{
		case STATUS_REPORT_REQUEST:
			send_status_report_request();
		break;

		case PUSH_MEDICINE_REQUEST:
			send_push_medicine_request(input_data);
		break;

		case REPLENISH_MEDICINE_COMPLETE_REQUEST:
			send_replinish_medicine_request(input_data);
		break;
		

		case CMD_ACK:
			send_command_ack(input_data);
		break;
		
		
		defualt:
		break;
	}
	return 0;
}


  
int up_shared_buf_copy(unsigned char *src, int len)
{  
	int chk_offset = 0;
	int i = 0;
	
	//if(len > RX_BD_MAX_DATA_SIZE)
	//return -1;
	
	do
	{
		memcpy(uart1_shared_rx_buf, src + chk_offset, len);

		UsartPrintf(USART_DEBUG, "start code = 0x%02x, cmd = 0x%02x!!\r\n", uart1_shared_rx_buf[0], uart1_shared_rx_buf[2]);
		if ((uart1_shared_rx_buf[0] == START_CODE)&&(uart1_shared_rx_buf[2] == CMD_CMD_ACK)) //收到状态上报响应
		{  
			buf_bitmap |= CMD_ACK_BUF;  
			memcpy(message_ack_buf, uart1_shared_rx_buf, sizeof(message_ack_buf));	
		} 
		else if ((uart1_shared_rx_buf[0] == START_CODE)&&(uart1_shared_rx_buf[2] == CMD_STATUS_REPORT_REQUEST)) //收到状态上报请求
		{  
			buf_bitmap |= STATUS_REPORT_REQUEST_BUF;  
			memcpy(status_report_request_buf, uart1_shared_rx_buf, sizeof(status_report_request_buf));  
		} 
		else if ((uart1_shared_rx_buf[0] == START_CODE)&&(uart1_shared_rx_buf[2] == CMD_TEST_REQUEST)) //收到状态上报响应
		{  
			buf_bitmap |= TEST_REQUEST_BUF;  
			memcpy(board_test_buf, uart1_shared_rx_buf, sizeof(board_test_buf));  
		}  
		else
		{
			UsartPrintf(USART_DEBUG, "Received Data is Error, drop!!\r\n");
			break;
		}
		
		
		chk_offset = uart1_shared_rx_buf[1];
		UsartPrintf(USART_DEBUG, "chk_offset = %d, len = %d!!\r\n", chk_offset, len);
		
	}while(chk_offset >0 && (chk_offset + 1) < len);
	
	return 0;
}  



void print_message_ack(struct cmd_ack_struct *cmd_ack)  
{  
    UsartPrintf(USART_DEBUG, "\r\n    ========= 消息应答包-打印开始=========\r\n");  

    UsartPrintf(USART_DEBUG, "\r\n    包消息类型:0x%02x", cmd_ack->cmd_type);  
    UsartPrintf(USART_DEBUG, "\r\n    包总长度:0x%02x", cmd_ack->packet_len);  
    UsartPrintf(USART_DEBUG, "\r\n    包校验和:0x%02x\r\n", cmd_ack->checksum);  
  
    UsartPrintf(USART_DEBUG, "\r\n    =========  消息应答包-打印结束=========\r\n");  
 }  

void parse_message_ack(struct cmd_ack_struct *cmd_ack)  
{  
  
	cmd_ack->start_code= message_ack_buf[0];  

	cmd_ack->packet_len= message_ack_buf[1];  
	cmd_ack->cmd_type= message_ack_buf[2];

	cmd_ack->ack.board_id = message_ack_buf[3];
  	cmd_ack->ack.rsp_cmd_type = message_ack_buf[4];
  	cmd_ack->ack.status = message_ack_buf[5];  
}  


void print_status_report_request(struct status_report_request_struct  *status_report_request)  
{  
    UsartPrintf(USART_DEBUG, "\r\n    ========= 状态上报包-打印开始=========\r\n");  

    UsartPrintf(USART_DEBUG, "\r\n    包消息类型:0x%02x", status_report_request->cmd_type);  
    UsartPrintf(USART_DEBUG, "\r\n    包总长度:0x%02x", status_report_request->packet_len);  
    UsartPrintf(USART_DEBUG, "\r\n    包校验和:0x%02x\r\n", status_report_request->checksum);  
  
    UsartPrintf(USART_DEBUG, "\r\n    =========  状态上报包-打印结束=========\r\n");  
 }  

void parse_status_report_request(struct status_report_request_struct *status_report_request)  
{  
  
	status_report_request->start_code= status_report_request_buf[0];  

	status_report_request->packet_len= status_report_request_buf[1];  
	status_report_request->cmd_type= status_report_request_buf[2];

	status_report_request->info.board_id = status_report_request_buf[3];
    
  	status_report_request->checksum = status_report_request_buf[8];  
}  

void print_push_medicine_request(struct push_medicine_request_struct *push_medicine_request)  
{  
  	uint8_t request_cnt = 0;
	uint8_t valid_cnt = 0;
	uint8_t i = 0;

	request_cnt = (push_medicine_request->packet_len - IPUC)/PUSH_MEDICINE_REQUEST_INFO_SIZE;

    UsartPrintf(USART_DEBUG, "\r\n    ========= 出货请求报文-打印开始=========\r\n");  
	UsartPrintf(USART_DEBUG, "\r\n	  包消息类型:0x%02x", push_medicine_request->cmd_type);  
	UsartPrintf(USART_DEBUG, "\r\n	  包总长度:0x%02x", push_medicine_request->packet_len); 
	for(i = 0; i < request_cnt; i++)
	{
	    UsartPrintf(USART_DEBUG, "\r\n    单板号:0x%04x", push_medicine_request->info[i].board_id); 
	    UsartPrintf(USART_DEBUG, "\r\n    单板运货道号:0x%04x", push_medicine_request->info[i].medicine_track_number); 
	    UsartPrintf(USART_DEBUG, "\r\n    单板运行时间:0x%04x", push_medicine_request->info[i].push_time); 
	}
	UsartPrintf(USART_DEBUG, "\r\n    包校验和:0x%02x\r\n", push_medicine_request->checksum);  
  
    UsartPrintf(USART_DEBUG, "\r\n    =========  出货请求报文-打印结束=========\r\n");  
 }  

void parse_push_medicine_request(struct push_medicine_request_struct *push_medicine_request)  
{  
  	uint8_t request_cnt = 0;
	uint8_t valid_cnt = 0;
	uint8_t i = 0;
	uint8_t check_sum = 0;
	struct cmd_ack_info_struct cmd_ack_info;
	cmd_ack_info.status = 0;
	
	push_medicine_request->start_code= push_medicine_request_buf[0];  

	push_medicine_request->packet_len= push_medicine_request_buf[1];  
	push_medicine_request->cmd_type= push_medicine_request_buf[2];
	
	check_sum = add_checksum(push_medicine_request_buf, push_medicine_request->packet_len);
	push_medicine_request->checksum = push_medicine_request_buf[push_medicine_request->packet_len];  
	
	if(check_sum != push_medicine_request->checksum)
	{
		
		UsartPrintf(USART_DEBUG, "check sum fail : 0x%02x, 0x%02x\r\n", check_sum, push_medicine_request->checksum);  
		send_command_ack(&cmd_ack_info);
		return;
	}
		
	request_cnt = (push_medicine_request->packet_len - IPUC)/PUSH_MEDICINE_REQUEST_INFO_SIZE;
	UsartPrintf(USART_DEBUG, "request_cnt: 0x%02x\r\n", request_cnt);  

	for(i = 0; i < request_cnt; i++)
	{
		push_medicine_request->info[valid_cnt].board_id = push_medicine_request_buf[3 + i * PUSH_MEDICINE_REQUEST_INFO_SIZE];
		UsartPrintf(USART_DEBUG, "board_id: 0x%02x, 0x%02x\r\n", push_medicine_request->info[valid_cnt].board_id, g_src_board_id);  
		
		//if(push_medicine_request->info[valid_cnt].board_id == g_src_board_id)
		{
			push_medicine_request->info[valid_cnt].medicine_track_number = push_medicine_request_buf[4 + i * PUSH_MEDICINE_REQUEST_INFO_SIZE];
			push_medicine_request->info[valid_cnt].push_time = push_medicine_request_buf[5 + i * PUSH_MEDICINE_REQUEST_INFO_SIZE]<<8|push_medicine_request_buf[6 + i * PUSH_MEDICINE_REQUEST_INFO_SIZE];

			if(enqueue_push_index < TOTAL_PUSH_CNT)
			{
				memcpy(&push_srtuct[enqueue_push_index], &push_medicine_request->info[valid_cnt], sizeof(struct push_medicine_request_info_struct));
				enqueue_push_index ++;
			}
			
			if(enqueue_push_index == TOTAL_PUSH_CNT)
			enqueue_push_index = 0;

			valid_cnt++;
		}
		
		if(valid_cnt)
		{
			OSSemPost(SemOfMotor);
			cmd_ack_info.status = 1;
		}
	}
	cmd_ack_info.board_id = g_src_board_id;
	cmd_ack_info.rsp_cmd_type = push_medicine_request->cmd_type;
	//send_command_ack(&cmd_ack_info);
	
	return;
}





void print_replenish_medicine_request(struct replenish_medicine_request_struct *replenish_medicine_request)  
{  
  	uint8_t request_cnt = 0;
	uint8_t valid_cnt = 0;
	uint8_t i = 0;

	request_cnt = (replenish_medicine_request->packet_len - IPUC)/REPLENISH_MEDICINE_REQUEST_INFO_SIZE;

    UsartPrintf(USART_DEBUG, "\r\n    ========= 补货请求报文-打印开始=========\r\n");  
	UsartPrintf(USART_DEBUG, "\r\n	  包消息类型:0x%02x", replenish_medicine_request->cmd_type);  
	UsartPrintf(USART_DEBUG, "\r\n	  包总长度:0x%02x", replenish_medicine_request->packet_len); 
	for(i = 0; i < request_cnt; i++)
	{
	    UsartPrintf(USART_DEBUG, "\r\n    单板号:0x%04x", replenish_medicine_request->info[i].board_id); 
	    UsartPrintf(USART_DEBUG, "\r\n    单板运货道号:0x%04x", replenish_medicine_request->info[i].medicine_track_number); 
	    UsartPrintf(USART_DEBUG, "\r\n    单板运行时间:0x%04x", replenish_medicine_request->info[i].push_time); 
	}
	UsartPrintf(USART_DEBUG, "\r\n    包校验和:0x%02x\r\n", replenish_medicine_request->checksum);  
  
    UsartPrintf(USART_DEBUG, "\r\n    =========  补货请求报文-打印结束=========\r\n");  
 }  

void parse_replenish_medicine_request(struct replenish_medicine_request_struct *replenish_medicine_request)  
{  
  	uint8_t request_cnt = 0;
	uint8_t valid_cnt = 0;
	uint8_t i = 0;
	uint8_t check_sum = 0;
	struct cmd_ack_info_struct cmd_ack_info;
	cmd_ack_info.status = 0;
	
	replenish_medicine_request->start_code= replenish_medicine_request_buf[0];  

	replenish_medicine_request->packet_len= replenish_medicine_request_buf[1];  
	replenish_medicine_request->cmd_type= replenish_medicine_request_buf[2];
	
	check_sum = add_checksum(replenish_medicine_request_buf, replenish_medicine_request->packet_len);
	replenish_medicine_request->checksum = replenish_medicine_request_buf[replenish_medicine_request->packet_len];  
	
	if(check_sum != replenish_medicine_request->checksum)
	{
		
		UsartPrintf(USART_DEBUG, "check sum fail : 0x%02x, 0x%02x\r\n", check_sum, replenish_medicine_request->checksum);  
		send_command_ack(&cmd_ack_info);
		return;
	}
		
	request_cnt = (replenish_medicine_request->packet_len - IPUC)/REPLENISH_MEDICINE_REQUEST_INFO_SIZE;
	UsartPrintf(USART_DEBUG, "request_cnt: 0x%02x\r\n", request_cnt);  

	for(i = 0; i < request_cnt; i++)
	{
		replenish_medicine_request->info[valid_cnt].board_id = replenish_medicine_request_buf[3 + i * REPLENISH_MEDICINE_REQUEST_INFO_SIZE];
		UsartPrintf(USART_DEBUG, "board_id: 0x%02x\r\n", replenish_medicine_request->info[valid_cnt].board_id);  
		
		if(replenish_medicine_request->info[valid_cnt].board_id == g_src_board_id)
		{
			replenish_medicine_request->info[valid_cnt].medicine_track_number = replenish_medicine_request_buf[4 + i * REPLENISH_MEDICINE_REQUEST_INFO_SIZE];
			replenish_medicine_request->info[valid_cnt].push_time = replenish_medicine_request_buf[5 + i * REPLENISH_MEDICINE_REQUEST_INFO_SIZE]<<8|replenish_medicine_request_buf[6 + i * REPLENISH_MEDICINE_REQUEST_INFO_SIZE];

			if(enqueue_replenish_index < TOTAL_PUSH_CNT)
			{
				memcpy(&replenish_srtuct[enqueue_replenish_index], &replenish_medicine_request->info[valid_cnt], sizeof(struct push_medicine_request_info_struct));
				enqueue_replenish_index ++;
			}
			
			if(enqueue_replenish_index == TOTAL_PUSH_CNT)
			enqueue_replenish_index = 0;

			valid_cnt++;
		}
		if(valid_cnt)
		{
			OSSemPost(SemOfMotor);
			cmd_ack_info.status = 1;
		}
	}
	cmd_ack_info.board_id = g_src_board_id;
	cmd_ack_info.rsp_cmd_type = replenish_medicine_request->cmd_type;
	send_command_ack(&cmd_ack_info);
	
	return;
}

void print_board_test_request(struct test_request_struct *test_request)  
{  
    UsartPrintf(USART_DEBUG, "\r\n    ========= 单板测试报文-打印开始=========\r\n");  
	UsartPrintf(USART_DEBUG, "\r\n	  包消息类型:0x%02x", test_request->cmd_type);  
	UsartPrintf(USART_DEBUG, "\r\n	  包总长度:0x%02x", test_request->packet_len); 

    UsartPrintf(USART_DEBUG, "\r\n    单板号:0x%02x", test_request->info.board_id); 
    UsartPrintf(USART_DEBUG, "\r\n    测试模式:0x%02x", test_request->info.test_mode); 
    UsartPrintf(USART_DEBUG, "\r\n    测试货道:0x%02x", test_request->info.medicine_track_number); 
	
    UsartPrintf(USART_DEBUG, "\r\n    测试时间:0x%04x", test_request->info.test_time); 
	
	UsartPrintf(USART_DEBUG, "\r\n    包校验和:0x%02x\r\n", test_request->checksum);  
  
    UsartPrintf(USART_DEBUG, "\r\n    =========  单板测试报文-打印结束=========\r\n");  
 }  




void parse_board_test_request(struct test_request_struct *test_request)  
{  
	uint8_t check_sum = 0;
	struct cmd_ack_info_struct cmd_ack_info;
	struct push_medicine_request_info_struct  motor_control;

	


	cmd_ack_info.status = 0;
	
	test_request->start_code= board_test_buf[0];  

	test_request->packet_len= board_test_buf[1];  
	test_request->cmd_type= board_test_buf[2];
	
	check_sum = add_checksum(board_test_buf, test_request->packet_len);
	test_request->checksum = board_test_buf[test_request->packet_len];  
	
	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
	
	if(check_sum != test_request->checksum)
	{
		
		UsartPrintf(USART_DEBUG, "check sum fail : 0x%02x, 0x%02x\r\n", check_sum, test_request->checksum);  
		send_command_ack(&cmd_ack_info);
		return;
	}
	
	test_request->info.board_id = board_test_buf[3];
	test_request->info.test_mode = board_test_buf[4];

	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);

	UsartPrintf(USART_DEBUG, "board_id: 0x%02x\r\n", test_request->info.board_id);  
	
	if(test_request->info.board_id == g_src_board_id)
	{
		motor_control.board_id = test_request->info.board_id;

		UsartPrintf(USART_DEBUG, "push_time[0x%02x][0x%02x]\r\n",board_test_buf[6], board_test_buf[7]);

		motor_control.medicine_track_number = test_request->info.medicine_track_number = board_test_buf[5];
		motor_control.push_time = test_request->info.test_time= board_test_buf[6]<<8|board_test_buf[7];

		UsartPrintf(USART_DEBUG, "test mode[%d]\r\n",test_request->info.test_mode);
		if(test_request->info.test_mode == MOTOR_FORWARD_TEST || test_request->info.test_mode == MOTOR_BACKWARD_TEST)
		{
			if(enqueue_push_index < TOTAL_PUSH_CNT)
				{
					memcpy(&push_srtuct[enqueue_push_index], &motor_control, sizeof(struct push_medicine_request_info_struct));
					enqueue_push_index ++;
				}
				
				if(enqueue_push_index == TOTAL_PUSH_CNT)
				enqueue_push_index = 0;

				UsartPrintf(USART_DEBUG, "send sem-------------\r\n");
				OSSemPost(SemOfMotor);
		}
		/*
		else if(test_request->info.test_mode == MOTOR_BACKWARD_TEST)
		{
			
			if(enqueue_replenish_index < TOTAL_PUSH_CNT)
			{
				memcpy(&replenish_srtuct[enqueue_replenish_index], &motor_control, sizeof(struct push_medicine_request_info_struct));
				enqueue_replenish_index ++;
			}
			
			if(enqueue_replenish_index == TOTAL_PUSH_CNT)
			enqueue_replenish_index = 0;
			
			OSSemPost(SemOfMotor);
		}
		*/
		
		else if(test_request->info.test_mode == TRACK_RUN_TEST)
		{
			if(MotorStatus.ConveyoeSta = 1)
			Conveyor_set(CONVEYOR_STOP);
			else
			Conveyor_set(CONVEYOR_RUN);
		}

		else if(test_request->info.test_mode == REFRIGERATION_RUN_TEST)
		{

		}

	}
	
	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
	
	cmd_ack_info.board_id = g_src_board_id;
	cmd_ack_info.rsp_cmd_type = test_request->cmd_type;

	
	cmd_ack_info.status = 1;
	send_command_ack((void *)&cmd_ack_info);
	
	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
	
	return;
}

void parse_up_rx_info(void)  
{  
	struct status_report_request_struct  status_report_request;
	struct cmd_ack_struct  cmd_ack;
    struct push_medicine_request_struct push_medicine_request;
    struct replenish_medicine_request_struct replenish_medicine_request;
    struct test_request_struct test_request;
	
	
    UsartPrintf(USART_DEBUG, "buf_bitmap : 0x%04x\r\n", buf_bitmap);  
    if (buf_bitmap & CMD_ACK_BUF)   
    { 
        parse_message_ack(&cmd_ack);  
        print_message_ack(&cmd_ack);  
        buf_bitmap &= ~CMD_ACK_BUF;  
    }
    else if (buf_bitmap & STATUS_REPORT_REQUEST_BUF)   
    { 
        parse_status_report_request(&status_report_request);  
        print_status_report_request(&status_report_request);  
        buf_bitmap &= ~STATUS_REPORT_REQUEST_BUF;  
    } 
	else if (buf_bitmap & PUSH_MEDICINE_REQUEST_BUF)   
    { 
        parse_push_medicine_request(&push_medicine_request);  
        print_push_medicine_request(&push_medicine_request);  
        buf_bitmap &= ~PUSH_MEDICINE_REQUEST_BUF;  
    }  
	else if (buf_bitmap & REPLENISH_MEDICINE_REQUEST_BUF)   
    { 
        parse_replenish_medicine_request(&replenish_medicine_request);  
        print_replenish_medicine_request(&replenish_medicine_request);  
        buf_bitmap &= ~REPLENISH_MEDICINE_REQUEST_BUF;  
    }	
	else if (buf_bitmap & TEST_REQUEST_BUF)   
    { 
        parse_board_test_request(&test_request);  
        print_board_test_request(&test_request);  
        buf_bitmap &= ~TEST_REQUEST_BUF;  
    }  

	
}  



