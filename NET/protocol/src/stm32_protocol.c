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



uint8_t  g_src_board_id = 0;  
//static uint8_t g_board_status = 0;  
//static uint8_t g_error_code = 0;  
//static uint8_t test[2] = {1,2}; 

uint16_t drag_push_time[BOARD_ID_MAX];  
uint16_t drag_push_time_calc_pre = 0;
uint16_t drag_push_time_calc = 0;

uint8_t track_work = 0;


static uint32_t buf_bitmap = 0;  
  
static uint8_t status_report_request_buf[STATUS_REPORT_REQUEST_PACKET_SIZE];  
static uint8_t push_medicine_request_buf[PUSH_MEDICINE_REQUEST_PACKET_SIZE];  
static uint8_t replenish_medicine_request_buf[REPLENISH_MEDICINE_REQUEST_PACKET_SIZE];  
static uint8_t calibrate_track_request_buf[CILIBRATE_TRACK_REQUEST_PACKET_SIZE];
static uint8_t replenish_medicine_complete_request_buf[REPLENISH_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE];
static uint8_t track_runtime_calc_request_buf[TRACK_RUNTIME_CALC_REQUEST_PACKET_SIZE];  


static uint8_t board_test_buf[BOARD_TEST_REQUEST_PACKET_SIZE]; 
static uint8_t message_ack_buf[COMMAND_ACK_PACKET_SIZE]; 



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

extern uint8_t calc_track_start_idx;
extern uint8_t calc_track_count;
extern uint16_t board_push_finish;
extern uint16_t board_add_finish;

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
	
	UsartPrintf(USART_DEBUG, " Current Board ID:0x%x\r\n", g_src_board_id); 
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

/*各货道出货完成，所有单板都需要执行*/
void mcu_push_medicine_track_only(uint8_t board, uint8_t track_number)
{
	struct push_medicine_complete_request_info_struct  push_complete_info;
	
	memset(&push_complete_info, 0x00, sizeof(push_complete_info));
	push_complete_info.board_id = board;
	push_complete_info.track_status = 0;
	push_complete_info.medicine_track_number = track_number;
	board_send_message(PUSH_MEDICINE_COMPLETE_REQUEST, &push_complete_info);
}

void mcu_add_medicine_track_only(uint8_t board, uint8_t track_number)
{
	struct push_medicine_complete_request_info_struct  push_complete_info;
	
	memset(&push_complete_info, 0x00, sizeof(push_complete_info));
	push_complete_info.board_id = board;
	push_complete_info.track_status = 0;
	push_complete_info.medicine_track_number = track_number;
	board_send_message(MCU_REPLENISH_MEDICINE_COMPLETE_REQUEST, &push_complete_info);
}


void send_command_ack( void *input_data)  
{  
	uint8_t send_cmd_ack_data[COMMAND_ACK_PACKET_SIZE];
	struct msg_ack_info_struct *cmd_ack_info = (struct msg_ack_info_struct * )input_data;

	memset(send_cmd_ack_data, 0x00, COMMAND_ACK_PACKET_SIZE);
	
	send_cmd_ack_data[0] = START_CODE;
	send_cmd_ack_data[1] = COMMAND_ACK_PACKET_SIZE - 1;
	send_cmd_ack_data[2] = CMD_MSG_ACK;

	send_cmd_ack_data[3] = cmd_ack_info->rsp_cmd_type;
	
	send_cmd_ack_data[4] = g_src_board_id;
	
	send_cmd_ack_data[5] = cmd_ack_info->status;

	send_cmd_ack_data[COMMAND_ACK_PACKET_SIZE - 1] = add_checksum(send_cmd_ack_data, COMMAND_ACK_PACKET_SIZE - 1);  
	
	UART1_IO_Send(send_cmd_ack_data, COMMAND_ACK_PACKET_SIZE);  
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

	UART1_IO_Send(send_statu_report_request_data, STATUS_REPORT_REQUEST_PACKET_SIZE);  
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
	UsartPrintf(USART_DEBUG, "\r\n: End");

	UART1_IO_Send(send_push_medicine_complete_request_data, PUSH_MEDICINE_COMPLETE_PACKET_SIZE);  
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

	UART1_IO_Send(send_push_medicine_complete_request_data, PUSH_MEDICINE_COMPLETE_PACKET_SIZE);  
} 



void send_replinish_medicine_request( void *input_data)  
{  
	int i = 0;
	uint8_t send_replinish_medicine_request_data[REPLENISH_MEDICINE_REQUEST_PACKET_SIZE];
	struct replenish_medicine_paramter *replenish_medicine_info = (struct replenish_medicine_paramter * )input_data;

	memset(send_replinish_medicine_request_data, 0x00, REPLENISH_MEDICINE_REQUEST_PACKET_SIZE);
	
	send_replinish_medicine_request_data[0] = START_CODE;
	send_replinish_medicine_request_data[1] = IPUC + REPLENISH_MEDICINE_REQUEST_INFO_SIZE * replenish_medicine_info->push_cnt;
	send_replinish_medicine_request_data[2] = CMD_REPLENISH_MEDICINE_REQUEST;
	
	UsartPrintf(USART_DEBUG, "replenish_medicine_info->push_cnt[%d]\r\n", replenish_medicine_info->push_cnt);

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









/*货道校准测试*/
void calibrate_track_test_request(void *input_data)  
{  
	uint8_t calibrate_track_request_data[CILIBRATE_TRACK_REQUEST_PACKET_SIZE];
	struct calibrate_track_request_info_struct *test_info = (struct calibrate_track_request_info_struct *)input_data;

	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
	
	memset(calibrate_track_request_data, 0x00, CILIBRATE_TRACK_REQUEST_PACKET_SIZE);
	
	calibrate_track_request_data[0] = START_CODE;
	calibrate_track_request_data[1] = CILIBRATE_TRACK_REQUEST_PACKET_SIZE - 1;
	calibrate_track_request_data[2] = CMD_CALIBRATE_TRACK_REQUEST;

	calibrate_track_request_data[3] = test_info->board_id;
	calibrate_track_request_data[4] = test_info->medicine_track_number;


	calibrate_track_request_data[CILIBRATE_TRACK_REQUEST_PACKET_SIZE - 1] = add_checksum(calibrate_track_request_data, CILIBRATE_TRACK_REQUEST_PACKET_SIZE - 1);  

	UART2_IO_Send(calibrate_track_request_data, CILIBRATE_TRACK_REQUEST_PACKET_SIZE);  

	
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

	UART2_IO_Send(replenish_complete_request_data, REPLENISH_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE);  

	
	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
} 







void push_test(void)
{
	struct push_medicine_paramter push_paramter;
	int i;
	push_paramter.push_cnt = 1;

	for(i = 0; i < push_paramter.push_cnt; i++)
	{
		push_paramter.info[i].board_id = 0x0001;
		push_paramter.info[i].medicine_track_number = 1 + i;
		push_paramter.info[i].push_time = 80 + i * 10;
	}

	send_push_medicine_request((void *)&push_paramter);
}



void replenish_test(void)
{
	struct replenish_medicine_paramter replenish_paramter;
	int i;
	replenish_paramter.push_cnt = 1;

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
	struct test_request_info_struct test_request_info;
	
	test_request_info.board_id = 0x01;
	test_request_info.test_mode = MOTOR_RUN_FORWARD;
	test_request_info.medicine_track_number = 0x01;
	test_request_info.test_time = 200;


	send_board_test_request((void *)&test_request_info);
}



void calibrate_test(void)
{
	struct calibrate_track_request_info_struct calibrate_track_request_info;
	
	calibrate_track_request_info.board_id = 0x01;
	calibrate_track_request_info.medicine_track_number = 0x01;

	calibrate_track_test_request((void *)&calibrate_track_request_info);
}


void replenish_complete_test(void)
{
	struct replenish_medicine_complete_request_info_struct replenish_medicine_complete_request_info;
	
	replenish_medicine_complete_request_info.board_id = 0x01;

	replenish_complete_test_request((void *)&replenish_medicine_complete_request_info);
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

	
	UART1_IO_Send(send_track_runtime, TRACK_RUNTIME_CALC_REPORT_PACKET_SIZE);  
} 


int board_send_message(int msg_type, void *input_data)
{
    //UsartPrintf(USART_DEBUG, "board_send_message:0x%02x\r\n",  msg_type);  
	switch(msg_type)
	{
		case STATUS_REPORT_REQUEST:
			send_status_report_request(input_data);
		break;

		case PUSH_MEDICINE_REQUEST:
			send_push_medicine_request(input_data);
		break;

		case PUSH_MEDICINE_COMPLETE_REQUEST:
		case MCU_REPLENISH_MEDICINE_COMPLETE_REQUEST:
			send_push_medicine_complete_request(input_data);
		break;


		case REPLENISH_MEDICINE_COMPLETE_REQUEST:
			send_replinish_medicine_request(input_data);
		break;

		case CMD_ACK:
			send_command_ack(input_data);
		break;
		
		case CMD_TRACK_RUNTIME_CALC:
			send_command_ack(input_data);
		break;
		
		//defualt:
		//break;
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

void parse_message_ack(struct msg_ack_struct *cmd_ack)  
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

	status_report_request->checksum = status_report_request_buf[7];  
}  

void print_push_medicine_request(struct push_medicine_request_struct *push_medicine_request)  
{  
  	uint8_t request_cnt = 0;
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
	uint8_t try_cnt = 0;
	uint8_t x = 0;
	uint8_t y = 0;
	
	struct msg_ack_info_struct cmd_ack_info;
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

	while((motor_enqueue_idx == motor_dequeue_idx - 1) && (try_cnt <= 5))
	{
		try_cnt++;
	}
	
	if(try_cnt == 5)
	{
		send_command_ack(&cmd_ack_info);
		return;
	}
	
	for(i = 0; i < request_cnt; i++)
	{
		push_medicine_request->info[valid_cnt].board_id = push_medicine_request_buf[3 + i * PUSH_MEDICINE_REQUEST_INFO_SIZE];
		UsartPrintf(USART_DEBUG, "board_id: 0x%02x, 0x%02x\r\n", push_medicine_request->info[valid_cnt].board_id, g_src_board_id);  

		
		if(push_medicine_request->info[valid_cnt].board_id == g_src_board_id)
		{
			push_medicine_request->info[valid_cnt].medicine_track_number = push_medicine_request_buf[4 + i * PUSH_MEDICINE_REQUEST_INFO_SIZE];
			push_medicine_request->info[valid_cnt].push_time = push_medicine_request_buf[5 + i * PUSH_MEDICINE_REQUEST_INFO_SIZE]<<8|push_medicine_request_buf[6 + i * PUSH_MEDICINE_REQUEST_INFO_SIZE];

			if((push_medicine_request->info[valid_cnt].medicine_track_number != 0) && (push_medicine_request->info[valid_cnt].push_time != 0))
			{
				//motor_struct[motor_enqueue_idx].motor_run = MOTOR_RUN_FORWARD;
				//motor_struct[motor_enqueue_idx].motor_work_mode = CMD_PUSH_MEDICINE_REQUEST;
				//memcpy(&motor_struct[motor_enqueue_idx].info, &push_medicine_request->info[valid_cnt], sizeof(struct motor_control_info_struct));

				//motor_enqueue_idx++;
				//if(motor_enqueue_idx >= TOTAL_PUSH_CNT)
				//motor_enqueue_idx = 0;	

				x = (push_medicine_request->info[valid_cnt].medicine_track_number - 1)/10;
				y = (push_medicine_request->info[valid_cnt].medicine_track_number - 1)%10;
				UsartPrintf(USART_DEBUG, "medicine_track_number = %d, track_struct[%d][%d].push_time = %d\r\n", push_medicine_request->info[valid_cnt].medicine_track_number, x, y, track_struct[x][y].push_time);
				track_struct[x][y].motor_run = MOTOR_RUN_FORWARD;
				track_struct[x][y].medicine_track_number = push_medicine_request->info[valid_cnt].medicine_track_number;
				track_struct[x][y].push_time = push_medicine_request->info[valid_cnt].push_time;
				valid_cnt++;
				cmd_ack_info.status = 1;
			}
			else if((push_medicine_request->info[valid_cnt].medicine_track_number == 0) && (push_medicine_request->info[valid_cnt].push_time == 0))
			{
				//OSSemPost(SemOfConveyor);
				OSSemPost(SemOfTrack);
				
				cmd_ack_info.status = 1;

				track_work = MOTOR_RUN_FORWARD;
				UsartPrintf(USART_DEBUG, "Receive push complete!!!!!!!!!!!!\r\n");
			}
		}
		
		if(valid_cnt)
		{
			//OSSemPost(SemOfMotor);
		}
	}
	cmd_ack_info.board_id = g_src_board_id;
	cmd_ack_info.rsp_cmd_type = push_medicine_request->cmd_type;
	send_command_ack(&cmd_ack_info);
	
	return;
}


uint8_t preparse_push_medicine_request(struct push_medicine_request_struct *push_medicine_request, uint8_t *buffer)  
{  
	uint8_t check_sum = 0;
	
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

	if((push_medicine_request->info[0].board_id == 1) && 
		(push_medicine_request->info[0].medicine_track_number == 0) && 
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
		(replenish_medicine_request->info[0].medicine_track_number == 0) && 
		(replenish_medicine_request->info[0].push_time == 0))
	return TRUE;
	
	return TRUE;
}


void print_replenish_medicine_request(struct replenish_medicine_request_struct *replenish_medicine_request)  
{  
  	uint8_t request_cnt = 0;
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
	struct msg_ack_info_struct cmd_ack_info;
	uint8_t x = 0;
	uint8_t y = 0;

	
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

			if((replenish_medicine_request->info[valid_cnt].medicine_track_number != 0) && (replenish_medicine_request->info[valid_cnt].push_time != 0))
			{
				//motor_struct[motor_enqueue_idx].motor_run = MOTOR_RUN_BACKWARD;
				//motor_struct[motor_enqueue_idx].motor_work_mode = CMD_REPLENISH_MEDICINE_REQUEST;
				//memcpy(&motor_struct[motor_enqueue_idx].info, &replenish_medicine_request->info[valid_cnt], sizeof(struct motor_control_info_struct));
								
				//motor_enqueue_idx++;
				//if(motor_enqueue_idx >= TOTAL_PUSH_CNT)
				//motor_enqueue_idx = 0;


				x = (replenish_medicine_request->info[valid_cnt].medicine_track_number - 1)/10;
				y = (replenish_medicine_request->info[valid_cnt].medicine_track_number - 1)%10;
				track_struct[x][y].motor_run = MOTOR_RUN_FORWARD;
				track_struct[x][y].medicine_track_number = replenish_medicine_request->info[valid_cnt].medicine_track_number;
				track_struct[x][y].push_time = replenish_medicine_request->info[valid_cnt].push_time;


				valid_cnt++;

			}
			else if((replenish_medicine_request->info[valid_cnt].medicine_track_number == 0) && (replenish_medicine_request->info[valid_cnt].push_time == 0))
			{
				OSSemPost(SemOfTrack);
				cmd_ack_info.status = 1;
			
				track_work = MOTOR_RUN_BACKWARD;
				UsartPrintf(USART_DEBUG, "Receive replenish complete!!!!!!!!!!!!\r\n");
			}
		}


		if(valid_cnt)
		{
			//OSSemPost(SemOfMotor);
			cmd_ack_info.status = 1;
		}
	}
	cmd_ack_info.board_id = g_src_board_id;
	cmd_ack_info.rsp_cmd_type = replenish_medicine_request->cmd_type;
	send_command_ack(&cmd_ack_info);
	
	return;
}

void print_calibrate_request(struct calibrate_track_request_struct *calibrate_track_request)  
{  

    UsartPrintf(USART_DEBUG, "\r\n    ========= 货道校准报文-打印开始=========\r\n");  
	UsartPrintf(USART_DEBUG, "\r\n	  包消息类型:0x%02x", calibrate_track_request->cmd_type);  
	UsartPrintf(USART_DEBUG, "\r\n	  包总长度:0x%02x", calibrate_track_request->packet_len); 

    UsartPrintf(USART_DEBUG, "\r\n    单板号:0x%02x", calibrate_track_request->info.board_id); 
    UsartPrintf(USART_DEBUG, "\r\n    测试货道:0x%02x", calibrate_track_request->info.medicine_track_number); 
		
	UsartPrintf(USART_DEBUG, "\r\n    包校验和:0x%02x\r\n", calibrate_track_request->checksum);  
  
    UsartPrintf(USART_DEBUG, "\r\n    =========  货道校准报文-打印结束=========\r\n");  
}



void parse_calibrate_track_request(struct calibrate_track_request_struct *calibrate_track_request)  
{  
	uint8_t check_sum = 0;
	struct msg_ack_info_struct cmd_ack_info;
	struct motor_control_info_struct motor_control;
	
	cmd_ack_info.status = 0;
	memset(&motor_control, 0x00, sizeof(motor_control));
	
	calibrate_track_request->start_code = calibrate_track_request_buf[0];  
	calibrate_track_request->packet_len = calibrate_track_request_buf[1];  
	calibrate_track_request->cmd_type = calibrate_track_request_buf[2];
	
	check_sum = add_checksum(calibrate_track_request_buf, calibrate_track_request->packet_len);
	calibrate_track_request->checksum = calibrate_track_request_buf[calibrate_track_request->packet_len];  
	
	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
	
	if(check_sum != calibrate_track_request->checksum)
	{
		
		UsartPrintf(USART_DEBUG, "check sum fail : 0x%02x, 0x%02x\r\n", check_sum, calibrate_track_request->checksum);  
		send_command_ack(&cmd_ack_info);
		return;
	}
	
	calibrate_track_request->info.board_id = calibrate_track_request_buf[3];

	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);

	UsartPrintf(USART_DEBUG, "board_id: 0x%02x\r\n", calibrate_track_request->info.board_id);  
	
	if(calibrate_track_request->info.board_id == g_src_board_id)
	{
		motor_control.board_id = calibrate_track_request->info.board_id;		
		calibrate_track_request->info.medicine_track_number = calibrate_track_request_buf[4];
		calibrate_track_selected = calibrate_track_request->info.medicine_track_number;
		OSSemPost(SemOfKey);

		UsartPrintf(USART_DEBUG, "calibrate_track_selected[%d]\r\n", calibrate_track_selected);
	}
	
	
	cmd_ack_info.board_id = g_src_board_id;
	cmd_ack_info.rsp_cmd_type = calibrate_track_request->cmd_type;
	
	cmd_ack_info.status = 1;
	send_command_ack((void *)&cmd_ack_info);
	
	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
	
	return;
}






void print_replenish_complete_request(struct replenish_medicine_complete_struct *replenish_medicine_complete_request)  
{  
    UsartPrintf(USART_DEBUG, "\r\n    ========= 补货完成报文-打印开始=========\r\n");  
	UsartPrintf(USART_DEBUG, "\r\n	  包消息类型:0x%02x", replenish_medicine_complete_request->cmd_type);  
	UsartPrintf(USART_DEBUG, "\r\n	  包总长度:0x%02x", replenish_medicine_complete_request->packet_len); 

    UsartPrintf(USART_DEBUG, "\r\n    单板号:0x%02x", replenish_medicine_complete_request->info.board_id); 
		
	UsartPrintf(USART_DEBUG, "\r\n    包校验和:0x%02x\r\n", replenish_medicine_complete_request->checksum);  
  
    UsartPrintf(USART_DEBUG, "\r\n    =========  补货完成报文-打印结束=========\r\n");  
}


void parse_replenish_complete_request(struct replenish_medicine_complete_struct *replenish_medicine_complete_request)  
{  
	uint8_t check_sum = 0;
	struct msg_ack_info_struct cmd_ack_info;

	cmd_ack_info.status = 0;
	
	replenish_medicine_complete_request->start_code = replenish_medicine_complete_request_buf[0];  
	replenish_medicine_complete_request->packet_len = replenish_medicine_complete_request_buf[1];  
	replenish_medicine_complete_request->cmd_type = replenish_medicine_complete_request_buf[2];
	
	check_sum = add_checksum(replenish_medicine_complete_request_buf, replenish_medicine_complete_request->packet_len);
	replenish_medicine_complete_request->checksum = replenish_medicine_complete_request_buf[replenish_medicine_complete_request->packet_len];  
	
	//UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
	
	if(check_sum != replenish_medicine_complete_request->checksum)
	{
		
		UsartPrintf(USART_DEBUG, "check sum fail : 0x%02x, 0x%02x\r\n", check_sum, replenish_medicine_complete_request->checksum);  
		send_command_ack(&cmd_ack_info);
		return;
	}
	
	replenish_medicine_complete_request->info.board_id = replenish_medicine_complete_request_buf[3];

	//UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);

	//UsartPrintf(USART_DEBUG, "board_id: 0x%02x\r\n", replenish_medicine_complete_request->info.board_id);  
	
	if(replenish_medicine_complete_request->info.board_id == g_src_board_id)
	{
		calibrate_track_selected = 255;
		OSSemPost(SemOfKey);
	}
	
	//UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
	
	cmd_ack_info.board_id = g_src_board_id;
	cmd_ack_info.rsp_cmd_type = replenish_medicine_complete_request->cmd_type;
	
	cmd_ack_info.status = 1;
	send_command_ack((void *)&cmd_ack_info);
	
	//UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
	
	return;
}





void parse_track_runtime_calc_request(struct track_calc_request_struct *track_runtime_calc_request)  
{  
	struct msg_ack_info_struct cmd_ack_info;
	cmd_ack_info.status = 0;

	track_runtime_calc_request->start_code= track_runtime_calc_request_buf[0];  

	track_runtime_calc_request->packet_len= track_runtime_calc_request_buf[1];  
	track_runtime_calc_request->cmd_type= track_runtime_calc_request_buf[2];

	track_runtime_calc_request->info.board_id = track_runtime_calc_request_buf[3];
	
	track_runtime_calc_request->info.track_start_num = track_runtime_calc_request_buf[4];

	track_runtime_calc_request->info.track_count = track_runtime_calc_request_buf[5];

	track_runtime_calc_request->checksum = track_runtime_calc_request_buf[6]; 

	if(track_runtime_calc_request->info.track_start_num == 0xFF)
	{
		calc_track_start_idx = 1;
		calc_track_count = TRACK_MAX;
	}
	else
	{
		calc_track_start_idx = track_runtime_calc_request->info.track_start_num;
		calc_track_count = track_runtime_calc_request->info.track_count;
	}
	
	cmd_ack_info.board_id = g_src_board_id;
	cmd_ack_info.rsp_cmd_type = track_runtime_calc_request->cmd_type;
	
	cmd_ack_info.status = 1;
	send_command_ack((void *)&cmd_ack_info);

	OSSemPost(SemOfCalcTime);
}  

void print_track_runtime_calc_request(struct track_calc_request_struct *track_runtime_calc_request)  
{  

    UsartPrintf(USART_DEBUG, "\r\n    ========= 货道运行时长统计请求报文-打印开始=========\r\n");  
	UsartPrintf(USART_DEBUG, "\r\n	  包消息类型:0x%02x", track_runtime_calc_request->cmd_type);  
	UsartPrintf(USART_DEBUG, "\r\n	  包总长度:0x%02x", track_runtime_calc_request->packet_len); 
    UsartPrintf(USART_DEBUG, "\r\n    单板号:0x%04x", track_runtime_calc_request->info.board_id); 
    UsartPrintf(USART_DEBUG, "\r\n    单板起始运货道号:0x%04x", track_runtime_calc_request->info.track_start_num); 
    UsartPrintf(USART_DEBUG, "\r\n    单板货道数:0x%04x", track_runtime_calc_request->info.track_count); 
	UsartPrintf(USART_DEBUG, "\r\n    包校验和:0x%02x\r\n", track_runtime_calc_request->checksum);  
  
    UsartPrintf(USART_DEBUG, "\r\n    =========  货道运行时长统计请求报文-打印结束=========\r\n");  
 }  

  
int uart1_shared_buf_preparse(unsigned char *src, int len)
{  
	int chk_offset = 0;
	int pkt_len = 0;
	int board_id = 0;
	unsigned char *uart1_shared_rx_buf;  
	do
	{
		uart1_shared_rx_buf = src + chk_offset;

		UsartPrintf(USART_DEBUG, "start code = 0x%02x, cmd = 0x%02x!!\r\n", *(uart1_shared_rx_buf), *(uart1_shared_rx_buf + 2));
		
		pkt_len = *(uart1_shared_rx_buf + 1) + 1;//data + checksum
		board_id = *(uart1_shared_rx_buf + 3);
		if(board_id != g_src_board_id)
		{
			UsartPrintf(USART_DEBUG, "Board Mismatch, Forward to %d!!\r\n", board_id);
			UART2_IO_Send(uart1_shared_rx_buf, pkt_len);	
		}
		else
		{
			if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_STATUS_REPORT_REQUEST)) //收到状态上报请求
			{  
				buf_bitmap |= STATUS_REPORT_REQUEST_BUF;  
				memcpy(status_report_request_buf, uart1_shared_rx_buf, pkt_len);  
				UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_STATUS_REPORT_REQUEST!!\r\n");
			} 
			else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_PUSH_MEDICINE_REQUEST)) //收到状态上报响应
			{  
				buf_bitmap |= PUSH_MEDICINE_REQUEST_BUF;  
				memcpy(push_medicine_request_buf, uart1_shared_rx_buf, pkt_len);  
				UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_PUSH_MEDICINE_REQUEST!!\r\n");
			}  
			else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_REPLENISH_MEDICINE_REQUEST)) //收到状态上报响应
			{  
				buf_bitmap |= REPLENISH_MEDICINE_REQUEST_BUF;  
				memcpy(replenish_medicine_request_buf, uart1_shared_rx_buf, pkt_len);  
				UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_REPLENISH_MEDICINE_REQUEST!!\r\n");
			}  

			else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_CALIBRATE_TRACK_REQUEST)) //收到状态上报响应
			{  
				buf_bitmap |= CALIBRATE_TRACK_REQUEST_BUF;  
				memcpy(calibrate_track_request_buf, uart1_shared_rx_buf, pkt_len);  
				UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_CALIBRATE_TRACK_REQUEST!!\r\n");
			}  	
			
			else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_TEST_REQUEST)) //收到状态上报响应
			{  
				buf_bitmap |= TEST_REQUEST_BUF;  
				memcpy(board_test_buf, uart1_shared_rx_buf, pkt_len);  
				UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_TEST_REQUEST!!\r\n");
			}  			
			else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_MSG_ACK)) //收到状态上报响应
			{  
				buf_bitmap |= CMD_ACK_BUF;  
				memcpy(message_ack_buf, uart1_shared_rx_buf, pkt_len);	
				
				UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_MSG_ACK!!\r\n");
			} 

			else
			{
				UsartPrintf(USART_DEBUG, "Received Data is Error, drop!!\r\n");
				break;
			}

			UsartPrintf(USART_DEBUG, "buf_bitmap = 0x%08x!!\r\n", buf_bitmap);
		}
		
		chk_offset = chk_offset + pkt_len;
		//UsartPrintf(USART_DEBUG, "chk_offset = %d, len = %d!!\r\n", chk_offset, len);
	}while(chk_offset >0 && (chk_offset + 1) < len);
	
	return 0;
}  

uint8_t up_packet_preparser(unsigned char *src, int len)  
{  
	uint8_t check_sum = 0;
	uint8_t retval = FALSE;
	uint8_t pkt_len = 0;
	uint8_t cmd_type = 0;
	
	pkt_len = *(src + 1);//data
	cmd_type = *(src + 2);
	check_sum = *(src + len);

	if(check_sum == add_checksum(src, len))
	retval = TRUE;
	
	return retval;
}






void up_packet_parser(unsigned char *src, int len)  
{  
	int chk_offset = 0;
	int pkt_len = 0;
	int board_id = 0;
	unsigned char *uart1_shared_rx_buf; 
	//char cmd_type = 0;
	int i = 0;
	//unsigned char forward_data[32] = {0};
	struct push_medicine_complete_struct push_medicine_complete_request;

	do
	{
		uart1_shared_rx_buf = src + chk_offset;
		
		//UsartPrintf(USART_DEBUG, "chk_offset[%d]\r\n", chk_offset);
		UsartPrintf(USART_DEBUG, "up_packet_parser:");
		for(i = 0; i < len; i++)
		{
			UsartPrintf(USART_DEBUG, "0x%02x,", *(src + i));
		}
		UsartPrintf(USART_DEBUG, "\r\n:");
		
		if(*uart1_shared_rx_buf != START_CODE)
		{
			UsartPrintf(USART_DEBUG, "start code error: 0x%02x\r\n", *(uart1_shared_rx_buf));
			chk_offset++;
			goto	UPDATA_CONTINUE;
		}		

		if(len - chk_offset < IPUC)
		{
			UsartPrintf(USART_DEBUG, "Packet Len error: 0x%02x\r\n", len, chk_offset);
			chk_offset++;
			break;
		}	

		pkt_len = *(uart1_shared_rx_buf + 1) + 1;//data + checksum

		if(up_packet_preparser(uart1_shared_rx_buf, pkt_len - 1) == FALSE)
		{
			chk_offset++;
			goto	UPDATA_CONTINUE;
		}
		UART1_IO_Send(uart1_shared_rx_buf, pkt_len);

		memcpy(&push_medicine_complete_request, uart1_shared_rx_buf, pkt_len);

		if(1 == g_src_board_id)
		{
			if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_PUSH_MEDICINE_COMPLETE)) //收到出货完成状态上报响应
			{	 
				board_id = *(uart1_shared_rx_buf + 3);
				UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_PUSH_MEDICINE_COMPLETE, Board[%d], Track[%d]!!\r\n", push_medicine_complete_request.info.board_id, push_medicine_complete_request.info.medicine_track_number);
				//TODO:统一由1号单板处理，在收集齐所有单板状态后统一上报安卓板
				if(push_medicine_complete_request.info.medicine_track_number == 0xFF)
				board_push_finish &= ~(1<<(board_id - 1));
			}  
			else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_MCU_ADD_MEDICINE_COMPLETE)) //收到补货完成状态上报响应
			{  
				board_id = *(uart1_shared_rx_buf + 3);
				UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_MCU_ADD_MEDICINE_COMPLETE, Board[%d], Track[%d]!!\r\n", push_medicine_complete_request.info.board_id, push_medicine_complete_request.info.medicine_track_number);
				if(push_medicine_complete_request.info.medicine_track_number == 0xFF)
				board_add_finish &= ~(1<<board_id);
			}
			else 
			{
				UsartPrintf(USART_DEBUG, "Received Data, transfor!!\r\n");
				break;
			}
		}
		chk_offset = chk_offset + pkt_len;
		//UsartPrintf(USART_DEBUG, "chk_offset = %d, len = %d!!\r\n", chk_offset, len);
		UPDATA_CONTINUE:
	}while(chk_offset > 0 && chk_offset < len);
		
}



void packet_parser(unsigned char *src, int len)  
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

	struct status_report_request_struct  status_report_request;
	struct msg_ack_struct  cmd_ack;
    struct push_medicine_request_struct push_medicine_request;
    struct replenish_medicine_request_struct replenish_medicine_request;
    //struct test_request_struct test_request;
    struct calibrate_track_request_struct calibrate_track_request; 
    struct replenish_medicine_complete_struct replenish_medicine_complete_request; 
    struct track_calc_request_struct track_calc_request; 

	
	UsartPrintf(USART_DEBUG, "packet_parser:");
	for(i = 0; i < len; i++)
	{
		UsartPrintf(USART_DEBUG, "0x%02x,", *(src + i));
	}
	UsartPrintf(USART_DEBUG, "\r\n:");

	do
	{
		uart1_shared_rx_buf = src + chk_offset;
		memset(protocol_data, 0x00, 32);
		
		//UsartPrintf(USART_DEBUG, "chk_offset[%d]\r\n", chk_offset);
		if(*uart1_shared_rx_buf != START_CODE)
		{
			UsartPrintf(USART_DEBUG, "start code error: 0x%02x\r\n", *(uart1_shared_rx_buf));
			chk_offset++;
			
			goto	PARSER_CONTINUE;
		}

		UsartPrintf(USART_DEBUG, "start code = 0x%02x, boardid = 0x%02x, cmd = 0x%02x!!\r\n", *(uart1_shared_rx_buf), *(uart1_shared_rx_buf + 3), *(uart1_shared_rx_buf + 2));
		
		pkt_len = *(uart1_shared_rx_buf + 1) + 1;//data + checksum
		cmd_type = *(uart1_shared_rx_buf + 2);
		board_id = *(uart1_shared_rx_buf + 3);

		check_sum = add_checksum(uart1_shared_rx_buf, pkt_len - 1);
		
		if(check_sum != uart1_shared_rx_buf[pkt_len - 1])
		{
			UsartPrintf(USART_DEBUG, "checksum error: 0x%02x, 0x%02x\r\n", check_sum, uart1_shared_rx_buf[pkt_len - 1]);
			chk_offset ++;
			goto	PARSER_CONTINUE;
		}
		
		if(1 == g_src_board_id)
		{
			if(cmd_type == CMD_PUSH_MEDICINE_REQUEST)
			{
				if(preparse_push_medicine_request(&push_medicine_request, uart1_shared_rx_buf) == TRUE)
				{
					UsartPrintf(USART_DEBUG, "board_push_finish: 0x%02x\r\n", board_push_finish);
					
					if(push_medicine_request.info[0].push_time > drag_push_time[push_medicine_request.info[0].board_id])
					{
						drag_push_time[push_medicine_request.info[0].board_id] = push_medicine_request.info[0].push_time;
						drag_push_time_calc_pre = push_medicine_request.info[0].push_time;
					}
					//转发
					memcpy(forward_data, uart1_shared_rx_buf, pkt_len);
					for(i = 1; i <= BOARD_ID_MAX; i++)
					{
						UsartPrintf(USART_DEBUG, "board_push_finish & (1 << (i-1)): %d\r\n", board_push_finish & (1 << (i-1)));
						if(board_push_finish & (1 << (i-1)))
						{
							memcpy(forward_data, uart1_shared_rx_buf, pkt_len);
							*(forward_data + 3) = i;
							*(forward_data + pkt_len - 1) = add_checksum(forward_data, pkt_len - 1);
							
							UsartPrintf(USART_DEBUG, "PUSH_MEDICINE_REQUEST Forward: 0x%02x\r\n", *(forward_data + 3));
							UART2_IO_Send(forward_data, pkt_len);
							RTOS_TimeDly(50);
						}
					}
				}
				else
				{
					board_push_finish |= 1<<(board_id - 1);
				}
			}
			else if(cmd_type == CMD_REPLENISH_MEDICINE_REQUEST)
			{
				//if(preparse_replenish_medicine_request(&replenish_medicine_request, uart1_shared_rx_buf) == TRUE)
				if(preparse_push_medicine_request(&push_medicine_request, uart1_shared_rx_buf) == TRUE)
				{
					UsartPrintf(USART_DEBUG, "board_add_finish: 0x%02x\r\n", board_add_finish);
					//转发
					memcpy(forward_data, uart1_shared_rx_buf, pkt_len);
					for(i = 0; i < BOARD_ID_MAX; i++)
					{	
						UsartPrintf(USART_DEBUG, "board_add_finish & (1 << (i-1)): %d\r\n", board_add_finish & (1 << (i-1)));
						if(board_add_finish & (1 << (i-1)))
						{
							memcpy(forward_data, uart1_shared_rx_buf, pkt_len);
							*(forward_data + 3) = i;
							*(forward_data + pkt_len - 1) = add_checksum(forward_data, pkt_len - 1);
							UsartPrintf(USART_DEBUG, "REPLENISH_MEDICINE_REQUEST Forward: 0x%02x\r\n", *(forward_data + 3));
							UART2_IO_Send(forward_data, pkt_len);
							RTOS_TimeDly(50);
						}
					}				
				}
				else
				{
					board_add_finish |= 1<<(board_id - 1);
				}
			}
		}
		
		if(board_id != g_src_board_id && (board_id > 0 ) && (board_id < BOARD_ID_MAX))
		{
			UsartPrintf(USART_DEBUG, "Board Id not match, forward!!!%d - %d!!\r\n", board_id, g_src_board_id);
			UART2_IO_Send(uart1_shared_rx_buf, pkt_len);	
		}
		else
		{
			if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_PUSH_MEDICINE_REQUEST)) //收到状态上报响应
			{  
				memcpy(push_medicine_request_buf, uart1_shared_rx_buf, pkt_len);  

				parse_push_medicine_request(&push_medicine_request);  
				print_push_medicine_request(&push_medicine_request);  
  
				UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_PUSH_MEDICINE_REQUEST!!\r\n");
			}  
			else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_REPLENISH_MEDICINE_REQUEST)) //收到状态上报响应
			{  
				memcpy(replenish_medicine_request_buf, uart1_shared_rx_buf, pkt_len);  
				parse_replenish_medicine_request(&replenish_medicine_request);	
				print_replenish_medicine_request(&replenish_medicine_request);	
				
				UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_REPLENISH_MEDICINE_REQUEST!!\r\n");
			}  
			else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_CALIBRATE_TRACK_REQUEST)) //收到状态上报响应
			{  
				memcpy(calibrate_track_request_buf, uart1_shared_rx_buf, pkt_len);  
				parse_calibrate_track_request(&calibrate_track_request);  
				print_calibrate_request(&calibrate_track_request);	
				UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_CALIBRATE_TRACK_REQUEST!!\r\n");
			}  	
			else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_TEST_REQUEST)) //收到状态上报响应
			{  
				parse_board_test_request(protocol_data, uart1_shared_rx_buf);  
				print_board_test_request(protocol_data);  
				
				UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_TEST_REQUEST!!\r\n");
			}  
			else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_ADD_MEDICINE_COMPLETE)) //收到状态上报响应
			{  
				memcpy(replenish_medicine_complete_request_buf, uart1_shared_rx_buf, pkt_len);	
				parse_replenish_complete_request(&replenish_medicine_complete_request);  
				print_replenish_complete_request(&replenish_medicine_complete_request);  
				UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_ADD_MEDICINE_COMPLETE!!\r\n");
			} 
			else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_MSG_ACK)) //收到状态上报响应
			{  
				memcpy(message_ack_buf, uart1_shared_rx_buf, pkt_len);	
				parse_message_ack(&cmd_ack);  
				//print_message_ack(&cmd_ack);  
				UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_MSG_ACK!!\r\n");
			} 
			else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_STATUS_REPORT_REQUEST)) //收到状态上报请求
			{  
				memcpy(status_report_request_buf, uart1_shared_rx_buf, pkt_len);  
				parse_status_report_request(&status_report_request);  
				print_status_report_request(&status_report_request);  
				UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_STATUS_REPORT_REQUEST!!\r\n");
			} 
			else if ((*(uart1_shared_rx_buf + 0) == START_CODE)&&(*(uart1_shared_rx_buf + 2) == CMD_TRACK_RUNTIME_CALC)) //收到状态上报请求
			{  
				memcpy(track_runtime_calc_request_buf, uart1_shared_rx_buf, pkt_len);  
				parse_track_runtime_calc_request(&track_calc_request);  
				print_track_runtime_calc_request(&track_calc_request);  
				UsartPrintf(USART_DEBUG, "Preparse Recvie CMD_STATUS_REPORT_REQUEST!!\r\n");
			} 			
			else 
			{
				UsartPrintf(USART_DEBUG, "Received Data is Error, drop!!\r\n");
				break;
			}
		}
		
		chk_offset = chk_offset + pkt_len;
		//UsartPrintf(USART_DEBUG, "chk_offset = %d, len = %d!!\r\n", chk_offset, len);
		PARSER_CONTINUE:
	}while(chk_offset > 0 && chk_offset < len);
		
}



