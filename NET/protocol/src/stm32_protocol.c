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
#include "protocol_func.h"

#include "data_io.h"
#include "stm32_uart1.h"
#include "stm32_uart2.h"
#include "usart.h"

//硬件驱动
#include "box.h"

//C库
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "ucos_ii.h"



//static uint8_t g_board_status = 0;  
//static uint8_t g_error_code = 0;  
//static uint8_t test[2] = {1,2}; 

uint16_t drag_push_time[BOARD_ID_MAX];  
uint16_t drag_push_time_calc_pre = 0;
uint16_t drag_push_time_calc = 0;

//static uint32_t buf_bitmap = 0;  
  

//static unsigned char txr_buf[MAX_PAYLOAD_LEN + 32];  


struct status_report_request_info_struct  heart_info;
struct motor_control_struct  motor_struct[TOTAL_PUSH_CNT];
struct track_work_struct track_struct[10][10];

int motor_enqueue_idx = 0;
int motor_dequeue_idx = 0;

int enqueue_replenish_index = 0;
int dequeue_replenish_index = 0;

uint16_t calibrate_track_selected = 255;
unsigned char calibrate_enable = 0;

extern OS_EVENT *SemOfMotor;          //Motor控制信号量
extern OS_EVENT *SemOfKey;          // 按键控制信号量
extern OS_EVENT *SemOfConveyor;        	//Motor控制信号量
extern OS_EVENT *SemOfTrack;        	//track 控制信号量
extern OS_EVENT *SemOfCalcTime;        	//触发货道时间统计信号量
extern OS_EVENT *SemOf485MsgSend;				//rs485消息轮询发送

extern uint8_t calc_track_start_idx;
extern uint8_t calc_track_count;


extern box_struct *knl_box_struct;
extern uint16_t g_push_time;


void BoardId_Init(void)
{
	
	GPIO_InitTypeDef gpioInitStrcut;

	//使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	
	//IO配置
	gpioInitStrcut.GPIO_Mode = GPIO_Mode_IPU;
	gpioInitStrcut.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	gpioInitStrcut.GPIO_Speed = GPIO_Speed_50MHz;
	//IO初始化
	GPIO_Init(GPIOE, &gpioInitStrcut);


	g_src_board_id = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0)<<3 | GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1)<<2 | 
		GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2)<<1 | GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3);

	memset(&heart_info, 0x00, sizeof(heart_info));
	heart_info.board_id = g_src_board_id;
	heart_info.board_status = FIRSTBOOT_STATUS;

	memset(track_struct, 0x00, sizeof(struct track_work_struct) * 10 * 10);
	
}

/*所有单板出货完成并已开门，1号单板执行*/
void mcu_push_medicine_open_door_complete(void)
{
	struct push_medicine_complete_request_info_struct  push_complete_info;
	
	memset(&push_complete_info, 0x00, sizeof(push_complete_info));
	push_complete_info.board_id = 0xff;
	push_complete_info.track_status= 0;
	board_send_message(PUSH_MEDICINE_COMPLETE_REQUEST, &push_complete_info);
}

/*所有单板出货完成并已关门，1号单板执行*/
void mcu_push_medicine_close_door_complete(void)
{
	struct push_medicine_complete_request_info_struct  push_complete_info;
	
	memset(&push_complete_info, 0x00, sizeof(push_complete_info));
	push_complete_info.board_id = 0xfe;
	push_complete_info.track_status= 0;
	board_send_message(PUSH_MEDICINE_COMPLETE_REQUEST, &push_complete_info);
}

/*出货失败，1号单板执行*/
void mcu_push_medicine_fail(void)
{
	struct push_medicine_complete_request_info_struct  push_complete_info;
	
	memset(&push_complete_info, 0x00, sizeof(push_complete_info));
	push_complete_info.board_id = 0xFD;
	push_complete_info.track_status= 0;
	push_complete_info.medicine_track_number= 0;
	board_send_message(PUSH_MEDICINE_COMPLETE_REQUEST, &push_complete_info);
}





/*各货道出货完成，所有单板都需要执行*/
//status: 0 完成，1 故障
void mcu_push_medicine_track_only(uint8_t board, uint8_t track_number, uint8_t status)
{
	struct push_medicine_complete_request_info_struct  push_complete_info;
	
	memset(&push_complete_info, 0x00, sizeof(push_complete_info));
	push_complete_info.board_id = board;
	push_complete_info.medicine_track_number = track_number;
	push_complete_info.track_status = status;
	board_send_message(PUSH_MEDICINE_COMPLETE_REQUEST, &push_complete_info);
}

void mcu_add_medicine_track_only(uint8_t board, uint8_t track_number)
{
	struct push_medicine_complete_request_info_struct  push_complete_info;
	
	memset(&push_complete_info, 0x00, sizeof(push_complete_info));
	push_complete_info.board_id = board;
	push_complete_info.medicine_track_number = track_number;
	push_complete_info.track_status = 0;
	board_send_message(MCU_REPLENISH_MEDICINE_COMPLETE_REQUEST, &push_complete_info);
}

void send_status_report_request(void *input_data)  
{  
    uint8_t send_statu_report_request_data[STATUS_REPORT_REQUEST_PACKET_SIZE];  

	memset(send_statu_report_request_data, 0x00, STATUS_REPORT_REQUEST_PACKET_SIZE);
	
	send_statu_report_request_data[0] = START_CODE;
	send_statu_report_request_data[1] = STATUS_REPORT_REQUEST_PACKET_SIZE - 1;
	send_statu_report_request_data[2] = CMD_STATUS_REPORT_REQUEST;

	#if 0
	send_statu_report_request_data[3] = g_src_board_id;
	send_statu_report_request_data[4] = g_board_status;
	send_statu_report_request_data[5] = g_error_code;
	
	send_statu_report_request_data[6] = test[0];
	send_statu_report_request_data[7] = test[1];
	#else
	memcpy(&send_statu_report_request_data[3], input_data, sizeof(struct status_report_request_info_struct));
	#endif
	
    send_statu_report_request_data[7] = add_checksum(send_statu_report_request_data, STATUS_REPORT_REQUEST_PACKET_SIZE);  

	if(g_src_board_id == 1)
	{
		UART1_IO_Send(send_statu_report_request_data, STATUS_REPORT_REQUEST_PACKET_SIZE);  
	}
	else
	{
		NotRetryMessageInsertQueue(send_statu_report_request_data, STATUS_REPORT_REQUEST_PACKET_SIZE, UART2_IDX);
	}
}  

void send_push_medicine_complete_request( void *input_data)  
{
	int i = 0;  
	uint8_t send_push_medicine_complete_request_data[PUSH_MEDICINE_COMPLETE_PACKET_SIZE];
	struct push_medicine_complete_request_info_struct *push_medicine_complete_info = (struct push_medicine_complete_request_info_struct *)input_data;


	memset(send_push_medicine_complete_request_data, 0x00, PUSH_MEDICINE_REQUEST_PACKET_SIZE);
	
	send_push_medicine_complete_request_data[0] = START_CODE;
	send_push_medicine_complete_request_data[1] = PUSH_MEDICINE_COMPLETE_PACKET_SIZE - 1;
	send_push_medicine_complete_request_data[2] = CMD_PUSH_MEDICINE_COMPLETE;

	send_push_medicine_complete_request_data[3] = push_medicine_complete_info->board_id;
	
	send_push_medicine_complete_request_data[4] = push_medicine_complete_info->medicine_track_number;
	
	send_push_medicine_complete_request_data[5] = push_medicine_complete_info->track_status;
	
	send_push_medicine_complete_request_data[PUSH_MEDICINE_COMPLETE_PACKET_SIZE - 1] = add_checksum(send_push_medicine_complete_request_data, PUSH_MEDICINE_COMPLETE_PACKET_SIZE - 1);  


	
	UsartPrintf(USART_DEBUG, "Start:");
	for(i = 0; i < PUSH_MEDICINE_COMPLETE_PACKET_SIZE; i++)
	{
		UsartPrintf(USART_DEBUG, "[0x%2x]", send_push_medicine_complete_request_data[i]);
	}
	UsartPrintf(USART_DEBUG, "End\r\n");

	if(g_src_board_id == 1)
	{
		UART1_IO_Send(send_push_medicine_complete_request_data, PUSH_MEDICINE_COMPLETE_PACKET_SIZE); 
		
		/*每个货道出货完成向1号板汇报*/
		//if(push_medicine_complete_info->medicine_track_number > TRACK_MAX)
		DelayMessageInsertQueue(send_push_medicine_complete_request_data, PUSH_MEDICINE_COMPLETE_PACKET_SIZE, UART1_IDX);
	}	
	else
	{
		//if(push_medicine_complete_info->medicine_track_number > TRACK_MAX)
		MessageInsertQueue(send_push_medicine_complete_request_data, PUSH_MEDICINE_COMPLETE_PACKET_SIZE, UART2_IDX);
	}

} 


void mcu_send_push_medicine_complete_request( void *input_data)  
{
	int i = 0;  
	uint8_t send_push_medicine_complete_request_data[PUSH_MEDICINE_COMPLETE_PACKET_SIZE];
	struct push_medicine_complete_request_info_struct *push_medicine_complete_info = (struct push_medicine_complete_request_info_struct *)input_data;


	memset(send_push_medicine_complete_request_data, 0x00, PUSH_MEDICINE_REQUEST_PACKET_SIZE);
	
	send_push_medicine_complete_request_data[0] = START_CODE;
	send_push_medicine_complete_request_data[1] = PUSH_MEDICINE_COMPLETE_PACKET_SIZE - 1;
	send_push_medicine_complete_request_data[2] = CMD_PUSH_MEDICINE_COMPLETE;

	send_push_medicine_complete_request_data[3] = push_medicine_complete_info->board_id;
	
	send_push_medicine_complete_request_data[4] = push_medicine_complete_info->medicine_track_number;
	
	send_push_medicine_complete_request_data[5] = push_medicine_complete_info->track_status;
	
	send_push_medicine_complete_request_data[PUSH_MEDICINE_COMPLETE_PACKET_SIZE - 1] = add_checksum(send_push_medicine_complete_request_data, PUSH_MEDICINE_COMPLETE_PACKET_SIZE - 1);  

	
	UsartPrintf(USART_DEBUG, "Start:");
	for(i = 0; i < PUSH_MEDICINE_COMPLETE_PACKET_SIZE; i++)
	{
		UsartPrintf(USART_DEBUG, "[0x%2x]", send_push_medicine_complete_request_data[i]);
	}
	UsartPrintf(USART_DEBUG, "\r\n: End");


	if(g_src_board_id == 1)
	{
		UART1_IO_Send(send_push_medicine_complete_request_data, PUSH_MEDICINE_COMPLETE_PACKET_SIZE);

		if(push_medicine_complete_info->medicine_track_number > TRACK_MAX)
		DelayMessageInsertQueue(send_push_medicine_complete_request_data, PUSH_MEDICINE_COMPLETE_PACKET_SIZE, UART1_IDX);
	}
	else
	{
		if(push_medicine_complete_info->medicine_track_number > TRACK_MAX)
		MessageInsertQueue(send_push_medicine_complete_request_data, PUSH_MEDICINE_COMPLETE_PACKET_SIZE, UART2_IDX);
	}
} 

void mcu_send_add_medicine_complete_request( void *input_data)  
{
	int i = 0;  
	uint8_t send_add_medicine_complete_request_data[ADD_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE];
	struct add_medicine_complete_request_info_struct *add_medicine_complete_info = (struct add_medicine_complete_request_info_struct *)input_data;


	memset(send_add_medicine_complete_request_data, 0x00, ADD_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE);
	
	send_add_medicine_complete_request_data[0] = START_CODE;
	send_add_medicine_complete_request_data[1] = ADD_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE - 1;
	send_add_medicine_complete_request_data[2] = CMD_MCU_ADD_MEDICINE_COMPLETE;

	send_add_medicine_complete_request_data[3] = add_medicine_complete_info->board_id;
	
	send_add_medicine_complete_request_data[4] = add_medicine_complete_info->medicine_track_number;
	
	send_add_medicine_complete_request_data[5] = add_medicine_complete_info->track_status;
	
	send_add_medicine_complete_request_data[ADD_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE - 1] = add_checksum(send_add_medicine_complete_request_data, ADD_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE - 1);  

	
	UsartPrintf(USART_DEBUG, "Start:");
	for(i = 0; i < ADD_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE; i++)
	{
		UsartPrintf(USART_DEBUG, "[0x%2x]", send_add_medicine_complete_request_data[i]);
	}
	UsartPrintf(USART_DEBUG, "\r\n: End");

	if(g_src_board_id == 1)
	{
		UART1_IO_Send(send_add_medicine_complete_request_data, ADD_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE); 
		DelayMessageInsertQueue(send_add_medicine_complete_request_data, ADD_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE, UART1_IDX);
	}
	else
	{
		MessageInsertQueue(send_add_medicine_complete_request_data, ADD_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE, UART2_IDX);
	}
} 



/*单板测试*/
void send_board_test_request(void *input_data)  
{  
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

	//UART2_IO_Send(send_board_test_request_data, BOARD_TEST_REQUEST_PACKET_SIZE);  
	MessageInsertQueue(send_board_test_request_data, BOARD_TEST_REQUEST_PACKET_SIZE, UART2_IDX);

	
	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
} 









/*货道校准测试*/
void calibrate_track_test_request(void *input_data)  
{  

	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
} 












/*补货完成测试*/
void replenish_complete_test_request(void *input_data)  
{  
	uint8_t replenish_complete_request_data[REPLENISH_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE];
	struct replenish_medicine_complete_request_info_struct *test_info = (struct replenish_medicine_complete_request_info_struct *)input_data;

	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
	
	memset(replenish_complete_request_data, 0x00, REPLENISH_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE);
	
	replenish_complete_request_data[0] = START_CODE;
	replenish_complete_request_data[1] = REPLENISH_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE - 1;
	replenish_complete_request_data[2] = CMD_ADD_MEDICINE_COMPLETE;

	replenish_complete_request_data[3] = test_info->board_id;


	replenish_complete_request_data[REPLENISH_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE - 1] = add_checksum(replenish_complete_request_data, REPLENISH_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE - 1);  

	//UART2_IO_Send(replenish_complete_request_data, REPLENISH_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE);  
	MessageInsertQueue(replenish_complete_request_data, REPLENISH_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE, UART2_IDX);
	
	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
} 

void send_track_runtime_report( void *input_data)  
{  
	uint8_t send_track_runtime[TRACK_RUNTIME_CALC_REPORT_PACKET_SIZE];
	struct track_cale_report_info_struct *track_runtime_info = (struct track_cale_report_info_struct* )input_data;

	memset(send_track_runtime, 0x00, TRACK_RUNTIME_CALC_REPORT_PACKET_SIZE);
	
	send_track_runtime[0] = START_CODE;
	send_track_runtime[1] = TRACK_RUNTIME_CALC_REPORT_PACKET_SIZE - 1;
	send_track_runtime[2] = CMD_TRACK_RUNTIME_REPORT;
	
	send_track_runtime[3] = track_runtime_info->board_id;

	send_track_runtime[4] = track_runtime_info->track_start_num;
	
	send_track_runtime[5] = (track_runtime_info->track_forward_time & 0xff00)>>8;
	send_track_runtime[6] = track_runtime_info->track_forward_time & 0xff;
	
	send_track_runtime[7] = (track_runtime_info->track_backward_time & 0xff00)>>8;;
	send_track_runtime[8] = track_runtime_info->track_backward_time & 0xff;
	
	send_track_runtime[TRACK_RUNTIME_CALC_REPORT_PACKET_SIZE - 1] = add_checksum(send_track_runtime, TRACK_RUNTIME_CALC_REPORT_PACKET_SIZE - 1);  

	if(g_src_board_id == 1)
	{
		UART1_IO_Send(send_track_runtime, TRACK_RUNTIME_CALC_REPORT_PACKET_SIZE); 
		DelayMessageInsertQueue(send_track_runtime, TRACK_RUNTIME_CALC_REPORT_PACKET_SIZE, UART1_IDX);
	}
	else
	{
		MessageInsertQueue(send_track_runtime, TRACK_RUNTIME_CALC_REPORT_PACKET_SIZE, UART2_IDX);
	}
} 
 


int board_send_message(int msg_type, void *input_data)
{
    //UsartPrintf(USART_DEBUG, "board_send_message:0x%02x\r\n",  msg_type);  
	switch(msg_type)
	{
		case STATUS_REPORT_REQUEST:
			send_status_report_request(input_data);
		break;

		case PUSH_MEDICINE_COMPLETE_REQUEST:
			send_push_medicine_complete_request(input_data);
		break;
			
		case MCU_REPLENISH_MEDICINE_COMPLETE_REQUEST:
			mcu_send_add_medicine_complete_request(input_data);
		break;

		case CMD_ACK:
			send_command_ack(input_data, UART1_IDX);
		break;

		default:
		break;
	}

	return 0;
}



void print_message_ack(struct msg_ack_struct *cmd_ack)  
{  
    UsartPrintf(USART_DEBUG, "\r\n    ========= 消息应答包-打印开始=========\r\n");  

    UsartPrintf(USART_DEBUG, "\r\n    包消息类型:0x%02x", cmd_ack->cmd_type);  
    UsartPrintf(USART_DEBUG, "\r\n    包总长度:0x%02x", cmd_ack->packet_len);  
    UsartPrintf(USART_DEBUG, "\r\n    包校验和:0x%02x\r\n", cmd_ack->checksum);  
  
    UsartPrintf(USART_DEBUG, "\r\n    =========  消息应答包-打印结束=========\r\n");  
 }  


uint8_t preparse_push_medicine_request(struct push_medicine_request_struct *push_medicine_request, uint8_t *buffer)  
{  
	uint8_t check_sum = 0;
    uint8_t board_id;  
    uint16_t push_time;  
	
	push_medicine_request->start_code= buffer[0];  
	push_medicine_request->packet_len= buffer[1];  
	push_medicine_request->cmd_type= buffer[2];
	
	check_sum = add_checksum(buffer, push_medicine_request->packet_len);
	push_medicine_request->checksum = buffer[push_medicine_request->packet_len];  
	
	if(check_sum != push_medicine_request->checksum)
	{
		
		UsartPrintf(USART_DEBUG, "check sum fail : 0x%02x, 0x%02x\r\n", check_sum, push_medicine_request->checksum);  
		return FALSE;
	} 

	push_medicine_request->info[0].board_id = buffer[3];
	push_medicine_request->info[0].medicine_track_number = buffer[4];
	push_medicine_request->info[0].push_time = buffer[5]<<8|buffer[6];
	push_medicine_request->info[0].drug_count = buffer[7];
	
	board_id = push_medicine_request->info[0].board_id;
 	push_time = push_medicine_request->info[0].push_time;

	//累加货道总运行时间，为每一个货道预留2s
	drag_push_time[board_id] += push_time + 20;
	UsartPrintf(USART_DEBUG, "Push Time Display: board[%d]track[%d]time[%d]cnt[%d]\r\n", 
		board_id, push_medicine_request->info[0].medicine_track_number, push_time, push_medicine_request->info[0].drug_count);  

	//0x02,0x08,0x20,0x01,0xff,0x00,0x00,0x00,0x2a
	if((push_medicine_request->info[0].board_id == 1) && 
		((push_medicine_request->info[0].medicine_track_number == 0)||(push_medicine_request->info[0].medicine_track_number == 0xff)) && 
		(push_medicine_request->info[0].push_time == 0))
		
	return TRUE;

	
	return FALSE;
}

uint8_t preparse_replenish_medicine_request(struct replenish_medicine_request_struct *replenish_medicine_request, uint8_t *buffer)  
{  
	uint8_t check_sum = 0;
	
	replenish_medicine_request->start_code= buffer[0];  
	replenish_medicine_request->packet_len= buffer[1];  
	replenish_medicine_request->cmd_type= buffer[2];
	
	check_sum = add_checksum(buffer, replenish_medicine_request->packet_len);
	replenish_medicine_request->checksum = buffer[replenish_medicine_request->packet_len];  
	
	if(check_sum != replenish_medicine_request->checksum)
	{
		
		UsartPrintf(USART_DEBUG, "check sum fail : 0x%02x, 0x%02x\r\n", check_sum, replenish_medicine_request->checksum);  
		return FALSE;
	} 

	replenish_medicine_request->info[0].board_id = buffer[3];
	replenish_medicine_request->info[0].medicine_track_number = buffer[4];
	replenish_medicine_request->info[0].push_time = buffer[5]<<8|buffer[6];

	if((replenish_medicine_request->info[0].board_id == 1) && 
		((replenish_medicine_request->info[0].medicine_track_number == 0)||(replenish_medicine_request->info[0].medicine_track_number == 0xff)) && 
		(replenish_medicine_request->info[0].push_time == 0))
	return TRUE;


	
	UsartPrintf(USART_DEBUG, "Add Time Display: board[%d]track[%d]time[%d]\r\n", 
		replenish_medicine_request->info[0].board_id, 
		replenish_medicine_request->info[0].medicine_track_number, 
		replenish_medicine_request->info[0].push_time);  
	
	return TRUE;
}

uint8_t up_packet_preparser(unsigned char *src, int len)  
{  
	uint8_t check_sum = 0;
	uint8_t retval = FALSE;
	//uint8_t pkt_len = 0;
	//uint8_t cmd_type = 0;
	
	//pkt_len = *(src + 1);//data
	//cmd_type = *(src + 2);
	check_sum = *(src + len);

	if(check_sum == add_checksum(src, len))
	retval = TRUE;
	
	return retval;
}


uint8_t IsACKMsg(unsigned char *src, int len)
{
	uint8_t retval = FALSE;
	
	retval = up_packet_preparser(src, len);
	if(retval == FALSE)
	return retval;
	
	retval = FALSE;
	if((*src == START_CODE)&&(*(src + 2) == CMD_MSG_ACK))
	retval = TRUE;
	
	return retval;
}

void up_packet_parser(unsigned char *src, int len)  
{  
	int chk_offset = 0;
	int pkt_len = 0;
	int board_id = 0;
	unsigned char *uart2_shared_rx_buf; 
	uint8_t check_sum = 0;
	int i = 0;
	//unsigned char forward_data[32] = {0};
	struct push_medicine_complete_struct push_medicine_complete_request;
	struct add_medicine_complete_struct add_medicine_complete_request;
	struct msg_ack_info_struct cmd_ack_info;

	UsartPrintf(USART_DEBUG, "HostBoard All Packet[%d]:", len);
	for(i = 0; i < len; i++)
	{
		UsartPrintf(USART_DEBUG, "0x%02x,", *(src + i));
	}
	UsartPrintf(USART_DEBUG, "\r\n");

	do
	{
		uart2_shared_rx_buf = src + chk_offset;
		
		if(*uart2_shared_rx_buf != START_CODE)
		{
			UsartPrintf(USART_DEBUG, "start code error: 0x%02x\r\n", *(uart2_shared_rx_buf));
			chk_offset++;
			goto	UPDATA_CONTINUE;
		}		

		if(len - chk_offset < IPUC)
		{
			UsartPrintf(USART_DEBUG, "Packet Len error: 0x%02x\r\n", len, chk_offset);
			chk_offset++;
			break;
		}	

		pkt_len = *(uart2_shared_rx_buf + 1) + 1;//data + checksum

		if (pkt_len > len - chk_offset)
		{
			chk_offset++;
			UsartPrintf(USART_DEBUG, "len error[%d %d], chk_offset:%d\r\n", pkt_len, len, chk_offset);
			goto	UPDATA_CONTINUE;
		}

		check_sum = add_checksum(uart2_shared_rx_buf, pkt_len - 1);
		if(check_sum != uart2_shared_rx_buf[pkt_len - 1])
		{
			UsartPrintf(USART_DEBUG, "checksum error: 0x%02x, 0x%02x\r\n", check_sum, uart2_shared_rx_buf[pkt_len - 1]);
			chk_offset ++;
			goto	UPDATA_CONTINUE;
		}
		
		UsartPrintf(USART_DEBUG, "One packet:");
		for(i = 0; i < pkt_len; i++)
		{
			UsartPrintf(USART_DEBUG, "0x%02x,", *(uart2_shared_rx_buf + i));
		}
		UsartPrintf(USART_DEBUG, "\r\n");

		if ((*(uart2_shared_rx_buf + 0) == START_CODE)&&(*(uart2_shared_rx_buf + 2) == CMD_MSG_ACK))
		{
			/*0x02,0x06,0xf0,0xa0,0x03,0x01,0x9c*/
			board_id = *(uart2_shared_rx_buf + 4);
			if(*(uart2_shared_rx_buf + 3) == CMD_PUSH_MEDICINE_REQUEST)
			knl_box_struct->board_push_ackmsg &= ~(1<<(board_id - 1));
			if(*(uart2_shared_rx_buf + 3) == CMD_REPLENISH_MEDICINE_REQUEST)
			knl_box_struct->board_add_ackmsg &= ~(1<<(board_id - 1));
			
			if(MessageAckCheck(uart2_shared_rx_buf, pkt_len) == 0) 
			UART1_IO_Send(uart2_shared_rx_buf, pkt_len);
		}
		else if ((*(uart2_shared_rx_buf + 0) == START_CODE)&&(*(uart2_shared_rx_buf + 2) == CMD_STATUS_REPORT_REQUEST))
		{
			/*0x02,0x07,0x10,0x03,0x02,0x00,0x2b,0x49*/
			struct status_report_request_info_struct *info = (struct status_report_request_info_struct *)(uart2_shared_rx_buf + 3);
			
			if(info && (PUSHING_STATUS == info->board_status))
			{
				UsartPrintf(USART_DEBUG, "Board[%d]Track[%d] is Pushing!!\r\n", info->board_id, info->medicine_track_number);
				g_push_time = 120;
				
				knl_box_struct->cur_boardidx = info->board_id;
				knl_box_struct->cur_trackidx = info->medicine_track_number;				
			}
			UART1_IO_Send(uart2_shared_rx_buf, pkt_len);
		}
		else if ((*(uart2_shared_rx_buf + 0) == START_CODE)&&(*(uart2_shared_rx_buf + 2) == CMD_PUSH_MEDICINE_COMPLETE)) //收到出货完成状态上报响应
		{	 
			/*0x02,0x06,0xa0,0x03,0xff,0x00,0xaa*/
			board_id = *(uart2_shared_rx_buf + 3);
			memcpy(&push_medicine_complete_request, uart2_shared_rx_buf, pkt_len);
			UsartPrintf(USART_DEBUG, "Board[%d]Track[%d] Push Finish!!\r\n", board_id, push_medicine_complete_request.info.medicine_track_number);

			cmd_ack_info.board_id = board_id;
			cmd_ack_info.rsp_cmd_type = CMD_PUSH_MEDICINE_COMPLETE;
			cmd_ack_info.status = 1;
			send_command_ack(&cmd_ack_info, UART2_IDX);

			//TODO:统一由1号单板处理，在收集齐所有单板状态后统一上报安卓板
			if(push_medicine_complete_request.info.medicine_track_number == 0xFF)
			{
				UsartPrintf(USART_DEBUG, "Board[%d]Track[%d] Have Finish!!\r\n", board_id, push_medicine_complete_request.info.medicine_track_number);
				knl_box_struct->board_push_finish &= ~(1<<(board_id - 1));
				UsartPrintf(USART_DEBUG, "Current push status[0x%x]!!\r\n", knl_box_struct->board_push_finish);

				if(knl_box_struct->board_push_finish)
				{
					for(i = 2; i <= BOARD_ID_MAX; i++)
					{
						if(knl_box_struct->board_push_finish & (1 << (i-1)))
						{
							UsartPrintf(USART_DEBUG, "Send Push Message to board[%d]!!\r\n", i);
							send_board_push_cmd(i, 0xFF);
							break;
						}
					}
				}
				else
				{
					UsartPrintf(USART_DEBUG, "All Board Finish!!\r\n");
				}
			}
			/*货道超时消息*/
			else if(push_medicine_complete_request.info.track_status == 1)
			{
				UsartPrintf(USART_DEBUG, "Board[%d]Track[%d] Have Push Fail!!\r\n", board_id, push_medicine_complete_request.info.medicine_track_number);
				knl_box_struct->board_push_finish = 0xffff;
				knl_box_struct->cur_boardidx = push_medicine_complete_request.info.board_id;
				knl_box_struct->cur_trackidx = push_medicine_complete_request.info.medicine_track_number;
			}
			
		}  
		else if ((*(uart2_shared_rx_buf + 0) == START_CODE)&&(*(uart2_shared_rx_buf + 2) == CMD_MCU_ADD_MEDICINE_COMPLETE)) //收到补货完成状态上报响应
		{  
			board_id = *(uart2_shared_rx_buf + 3);
			memcpy(&add_medicine_complete_request, uart2_shared_rx_buf, pkt_len);
			
			UsartPrintf(USART_DEBUG, "Board[%d]Track[%d] Add Finish!!\r\n", add_medicine_complete_request.info.board_id, add_medicine_complete_request.info.medicine_track_number);

			if(add_medicine_complete_request.info.medicine_track_number == 0xFF)
			knl_box_struct->board_add_finish &= ~(1<<(board_id - 1));
			
			cmd_ack_info.board_id = board_id;
			cmd_ack_info.rsp_cmd_type = CMD_MCU_ADD_MEDICINE_COMPLETE;
			cmd_ack_info.status = 1;
			send_command_ack(&cmd_ack_info, UART2_IDX);
		}
		else
		{
			UART1_IO_Send(uart2_shared_rx_buf, pkt_len);
			UsartPrintf(USART_DEBUG, "[up]Other Message [0x%02x]!!\r\n", *(uart2_shared_rx_buf + 2));
		}

		UsartPrintf(USART_DEBUG, "uart2 pkt_len = %d, chk_offset = %d, len = %d!!\r\n", pkt_len, chk_offset, len);
		chk_offset = chk_offset + pkt_len;
		UPDATA_CONTINUE:
		;
	}while(chk_offset > 0 && chk_offset < len);
		
}



void packet_parser(unsigned char *src, int len, int uart_idx)  
{  
	int chk_offset = 0;
	int pkt_len = 0;
	int board_id = 0;
	uint8_t *uart1_shared_rx_buf; 
	char cmd_type = 0;
	int i = 0;
	uint8_t forward_data[32] = {0};
	uint8_t protocol_data[32] = {0};
	uint8_t check_sum = 0;

    struct push_medicine_request_struct push_medicine_request;
    //struct replenish_medicine_request_struct replenish_medicine_request;

	
	UsartPrintf(USART_DEBUG, "All Packet[%d]:", len);
	for(i = 0; i < len; i++)
	{
		UsartPrintf(USART_DEBUG, "0x%02x,", *(src + i));
	}
	UsartPrintf(USART_DEBUG, "\r\n\r\n");

	do
	{
		uart1_shared_rx_buf = src + chk_offset;
		memset(protocol_data, 0x00, 32);
		
		//UsartPrintf(USART_DEBUG, "chk_offset[%d]\r\n", chk_offset);
		if(*uart1_shared_rx_buf != START_CODE)
		{
			UsartPrintf(USART_DEBUG, "start code error: 0x%02x\r\n", *(uart1_shared_rx_buf));
			chk_offset++;
			
			goto	NEXT_OFFSET;
		}

		//UsartPrintf(USART_DEBUG, "start code = 0x%02x, boardid = 0x%02x, cmd = 0x%02x!!\r\n", *(uart1_shared_rx_buf), *(uart1_shared_rx_buf + 3), *(uart1_shared_rx_buf + 2));
		
		pkt_len = *(uart1_shared_rx_buf + 1) + 1;//data + checksum
		cmd_type = *(uart1_shared_rx_buf + 2);
		board_id = *(uart1_shared_rx_buf + 3);

		if (pkt_len > len - chk_offset)
		{
			chk_offset++;
			UsartPrintf(USART_DEBUG, "pkt_len error[%d %d], chk_offset:%d\r\n", pkt_len, len, chk_offset);
			goto	NEXT_OFFSET;
		}

		check_sum = add_checksum(uart1_shared_rx_buf, pkt_len - 1);
		
		if(check_sum != uart1_shared_rx_buf[pkt_len - 1])
		{
			UsartPrintf(USART_DEBUG, "checksum error: 0x%02x, 0x%02x\r\n", check_sum, uart1_shared_rx_buf[pkt_len - 1]);
			chk_offset ++;
			goto	NEXT_OFFSET;
		}

		UsartPrintf(USART_DEBUG, "One Packet:");
		for(i = 0; i < pkt_len; i++)
		{
			UsartPrintf(USART_DEBUG, "0x%02x,", *(uart1_shared_rx_buf + i));
		}
		UsartPrintf(USART_DEBUG, "\r\n");

		if((*uart1_shared_rx_buf == START_CODE)&&(cmd_type == CMD_MSG_ACK)&&(*(uart1_shared_rx_buf + 4) == g_src_board_id))
		goto START_PARSER;


		/*来自uart1 的消息处理*/
		if((1 == g_src_board_id) && (uart_idx == UART1_IDX))
		{
			if(cmd_type == CMD_PUSH_MEDICINE_REQUEST)
			{
				/*请求出货指令转发*/
				preparse_push_medicine_request(&push_medicine_request, uart1_shared_rx_buf);
				if(push_medicine_request.info[0].medicine_track_number == 0 && push_medicine_request.info[0].push_time == 0)
				{
					knl_box_struct->board_push_finish = 0;
					//转发
					memcpy(forward_data, uart1_shared_rx_buf, pkt_len);
					for(i = 2; i <= BOARD_ID_MAX; i++)
					{
						//UsartPrintf(USART_DEBUG, "board_push_finish & (1 << (i-1)): 0x%x\r\n", knl_box_struct->board_push_finish & (1 << (i-1)));
						memcpy(forward_data, uart1_shared_rx_buf, pkt_len);
						*(forward_data + 3) = i;//board id
						*(forward_data + pkt_len - 1) = add_checksum(forward_data, pkt_len - 1);//message len
						UsartPrintf(USART_DEBUG, "PUSH_MEDICINE_REQUEST Forward: 0x%02x\r\n", *(forward_data + 3));

						RS485_Send_Data(forward_data, pkt_len);
						RTOS_TimeDlyHMSM(0, 0, 0, 20);	
						
						RS485_Send_Data(forward_data, pkt_len);
						RTOS_TimeDlyHMSM(0, 0, 0, 20);	
						
						//MessageInsertQueue(forward_data, pkt_len, UART2_IDX);
						//RTOS_TimeDlyHMSM(0, 0, 0, 100);			
						//send_query_message(i);
					}
				}
				else if(push_medicine_request.info[0].medicine_track_number == 0xff && push_medicine_request.info[0].push_time == 0)
				{
					UsartPrintf(USART_DEBUG, "board_push_finish: 0x%02x\r\n", knl_box_struct->board_push_finish);
				}
				else if(0 < board_id && board_id <= BOARD_ID_MAX)
				{	
					/*请求出货，更新全局变量bit位*/
					knl_box_struct->board_push_finish |= 1<<(board_id - 1);
					knl_box_struct->board_push_ackmsg |= 1<<(board_id - 1);
					UsartPrintf(USART_DEBUG, "board_id[%d]board_push_finish[0x%x]\r\n", 
						board_id, knl_box_struct->board_push_finish);
				}
			}
			else if(cmd_type == CMD_REPLENISH_MEDICINE_REQUEST)
			{
				/*请求补货完成指令转发*/
				if(preparse_push_medicine_request(&push_medicine_request, uart1_shared_rx_buf) == TRUE)
				{
					UsartPrintf(USART_DEBUG, "board_add_finish: 0x%02x\r\n", knl_box_struct->board_add_finish);
					if(push_medicine_request.info[0].medicine_track_number == 0 && push_medicine_request.info[0].push_time == 0)
					{
						knl_box_struct->board_add_finish = 0;
					}
					
					//转发
					memcpy(forward_data, uart1_shared_rx_buf, pkt_len);
					for(i = 2; i <= BOARD_ID_MAX; i++)
					{	
						UsartPrintf(USART_DEBUG, "board_add_finish & (1 << (i-1)): %d\r\n", knl_box_struct->board_add_finish & (1 << (i-1)));
						if(knl_box_struct->board_add_finish & (1 << (i-1)))
						{
							memcpy(forward_data, uart1_shared_rx_buf, pkt_len);
							*(forward_data + 3) = i;	//board id
							*(forward_data + pkt_len - 1) = add_checksum(forward_data, pkt_len - 1); //message len
							UsartPrintf(USART_DEBUG, "REPLENISH_MEDICINE_REQUEST Forward: 0x%02x\r\n", *(forward_data + 3));
		
							MessageInsertQueue(forward_data, pkt_len, UART2_IDX);
							RS485_Send_Data(forward_data, pkt_len);
							RTOS_TimeDlyHMSM(0, 0, 0, 100);
							
							send_query_message(i);
						}
					}				
				}
				else
				{
					knl_box_struct->board_add_finish |= 1<<(board_id - 1);
					knl_box_struct->board_add_ackmsg |= 1<<(board_id - 1);
					UsartPrintf(USART_DEBUG, "board_id[%d]board_add_finish[0x%x]\r\n", board_id, knl_box_struct->board_add_finish);
				}
			}
		}




		
		if(g_src_board_id == 1)/*1号板,HOST*/
		{
			if((board_id != g_src_board_id)&&(uart_idx == UART1_IDX))/*安卓机器发给其他单板的数据*/
			{
				RS485_Send_Data(uart1_shared_rx_buf, pkt_len);
				RTOS_TimeDlyHMSM(0, 0, 0, 100);
				send_query_message(board_id);
				UsartPrintf(USART_DEBUG, "ID Mismatch(%d->%d). Forward!!!!!!!\r\n", g_src_board_id, board_id);
				
				goto	NEXT_PACKET;
			}
			else if(uart_idx == UART2_IDX)/*其他单板发给安卓机的*/
			{
				UART1_IO_Send(uart1_shared_rx_buf, pkt_len);
				UsartPrintf(USART_DEBUG, "HostBoard From Device Board(%d->%d). Forward!!!!!!!\r\n", g_src_board_id, board_id);
				
				goto	NEXT_PACKET;
			}
		}
		else/*非1号板*/
		{
			if((board_id != g_src_board_id)&&(uart_idx == UART2_IDX))/*非1号单板，其他单板发送*/
			{
				UsartPrintf(USART_DEBUG, "To Device Board(%d->%d). Drop!!!!!!!\r\n", g_src_board_id, board_id);
				goto	NEXT_PACKET;
			}
			else if(uart_idx == UART1_IDX)/*非1号单板，其他单板发送*/
			{
				UsartPrintf(USART_DEBUG, "Warnning,cannot!!!!!!!!%d - %d!!\r\n", board_id, g_src_board_id);
				//goto	NEXT_PACKET;
			}
		}
		
		#if 0
		else/*非1号板*/
		{
			if((board_id != g_src_board_id)&&(uart_idx == UART2_IDX))/*非1号单板，其他单板发送*/
			{
				//只针对ID不匹配的消息丢弃
				UsartPrintf(USART_DEBUG, "[Other Message]Board Id not match, Drop!!!%d - %d!!\r\n", board_id, g_src_board_id);

				if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_PUSH_MEDICINE_REQUEST)) //收到状态上报响应
				{  
					parse_push_medicine_request(protocol_data, uart1_shared_rx_buf);  
					print_push_medicine_request(protocol_data);  
		
					UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_PUSH_MEDICINE_REQUEST!!\r\n");
				}  
				else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_REPLENISH_MEDICINE_REQUEST)) //收到状态上报响应
				{  
					parse_replenish_medicine_request(protocol_data, uart1_shared_rx_buf);	
					print_replenish_medicine_request(protocol_data);	
					
					UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_REPLENISH_MEDICINE_REQUEST!!\r\n");
				}		
				goto	NEXT_PACKET;
			}
			else if(uart_idx == UART1_IDX)/*非1号单板，其他单板发送*/
			{
				UsartPrintf(USART_DEBUG, "Warnning,cannot!!!!!!!!%d - %d!!\r\n", board_id, g_src_board_id);
				//goto	NEXT_PACKET;
			}
		}
		#endif

		
		//针对接收到的message处理
		START_PARSER:
		if((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_QUERY_MSG))//收到tx 轮询报文
		{
			//UsartPrintf(USART_DEBUG, "Recevied Query packet !!\r\n");
			OSSemPost(SemOf485MsgSend);
		}
		else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_MSG_ACK)) //收到消息应答报文
		{  
			UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_MSG_ACK!!\r\n");
			parse_message_ack(protocol_data, uart1_shared_rx_buf);
			MessageAckCheck(uart1_shared_rx_buf, pkt_len);
			//print_message_ack(protocol_data);  
		} 
		else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_PUSH_MEDICINE_REQUEST)) //收到出药请求
		{  
			UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_PUSH_MEDICINE_REQUEST!!\r\n");
			parse_push_medicine_request(protocol_data, uart1_shared_rx_buf);  
			print_push_medicine_request(protocol_data);  

		}  
		else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_REPLENISH_MEDICINE_REQUEST)) //收到补货请求
		{  
			UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_REPLENISH_MEDICINE_REQUEST!!\r\n");
			parse_replenish_medicine_request(protocol_data, uart1_shared_rx_buf);	
			print_replenish_medicine_request(protocol_data);	
			
		}   	
		else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_ADD_MEDICINE_COMPLETE)) //收到收到补货完成
		{  
			UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_ADD_MEDICINE_COMPLETE!!\r\n");
			parse_replenish_complete_request(protocol_data, uart1_shared_rx_buf);
			print_replenish_complete_request(protocol_data);  
		} 
		else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_TEST_REQUEST)) //收到接口测试报文
		{  
			UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_TEST_REQUEST!!\r\n");
			parse_board_test_request(protocol_data, uart1_shared_rx_buf);  
			//print_board_test_request(protocol_data);  
		} 
		else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_TRACK_RUNTIME_CALC)) //收到货道时间计算
		{  
			UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_STATUS_REPORT_REQUEST!!\r\n");
			parse_track_runtime_calc_request(protocol_data, uart1_shared_rx_buf);
			print_track_runtime_calc_request(protocol_data);  
		} 			
		else 
		{
			UsartPrintf(USART_DEBUG, "Other Message [0x%02x]!!\r\n", *(uart1_shared_rx_buf + 2));
		}

		
		NEXT_PACKET:
		UsartPrintf(USART_DEBUG, "uart1 pkt_len = %d, chk_offset = %d, len = %d!!\r\n", pkt_len, chk_offset, len);
		chk_offset = chk_offset + pkt_len;
		
		NEXT_OFFSET:
		;
	}while(chk_offset > 0 && chk_offset < len);
		
}



static int cmp(const void *a,const void *b)
{
    return *(uint16_t *)b - *(uint16_t *)a;
}


uint16_t GetMaxPushTime(void)
{
	uint16_t delay_s = 0;
	int i = 0;

	for(i = 0; i < BOARD_ID_MAX; i++)
	{
		if(drag_push_time[i])
		{
			UsartPrintf(USART_DEBUG, "Board[%d] Run[%d]-------------\r\n", i, drag_push_time[i]);
		}
	}
	qsort(drag_push_time, BOARD_ID_MAX, sizeof(drag_push_time[0]),cmp);
	delay_s = drag_push_time[0]/10;
	
	UsartPrintf(USART_DEBUG, "drag_push_time %d, %ds-------------\r\n", drag_push_time[0], delay_s);
	
	memset(&drag_push_time[0], 0x00, sizeof(drag_push_time));
	return delay_s;
}

void SetTrackTestTime(uint8_t track, uint8_t dir, uint16_t time)
{
	int i =0, j = 0;

	i = (track - 1)/10;
	j = (track - 1)%10;
	//for(i = 0; i < 10; i++)
	{
		//for(j = 0; j < 10; j++)
		{
			track_struct[i][j].push_time = time;
			track_struct[i][j].motor_run = dir;
    		track_struct[i][j].medicine_track_number = 10 * i + j + 1;
		}
	}
}



