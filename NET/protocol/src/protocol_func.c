/**
	************************************************************
	************************************************************
	************************************************************
	*	�ļ����� 	
	*
	*	���ߣ� 		
	*
	*	���ڣ� 		
	*
	*	�汾�� 		
	*
	*	˵���� 
	*
	*	�޸ļ�¼��	
	************************************************************
	************************************************************
	************************************************************
**/

#include "stm32f10x.h"	//��Ƭ��ͷ�ļ�
#include "box.h"

#include "stm32_protocol.h"
#include "protocol_func.h"


extern uint32_t time_passes;
extern uint32_t TrunkInitTime;


/*  
    �ۼ�У����㷨  
 */  
unsigned char add_checksum (unsigned char *buf, unsigned int len)  
{  
    unsigned int i;  
    unsigned char checksum = 0;  
  
    for (i = 0; i < len; ++i)  
    {  
        checksum += *(buf++);  
    }  
  
    return checksum;  
}  

void FactoryFuncTest(void)
{  	
	uint8_t delay = 10;
	uint16_t people_detect_time = 0;
	
	UsartPrintf(USART_DEBUG, "Push Belt test start\r\n");
	Belt_Set(PUSH_BELT, BELT_RUN);
	RTOS_TimeDlyHMSM(0, 0, delay, 0);
	Belt_Set(PUSH_BELT, BELT_STOP);
	UsartPrintf(USART_DEBUG, "Push Belt test stop\r\n");


	UsartPrintf(USART_DEBUG, "Collect Belt test start\r\n");
	Belt_Set(COLLECT_BELT, BELT_RUN);
	RTOS_TimeDlyHMSM(0, 0, delay, 0);
	Belt_Set(COLLECT_BELT, BELT_STOP);
	UsartPrintf(USART_DEBUG, "Push Belt test stop\r\n");
			

	UsartPrintf(USART_DEBUG, "Cooling  compressor test start\r\n");
	Coolingcompressor_Set(COOLING_ON);
	RTOS_TimeDlyHMSM(0, 0, delay, 0);
	Coolingcompressor_Set(COOLING_OFF);
	UsartPrintf(USART_DEBUG, "Cooling  compressor test stop\r\n");
			

	UsartPrintf(USART_DEBUG, "Cooling  fan test start\r\n");
	Coolingfan_Set(COOLING_ON);
	RTOS_TimeDlyHMSM(0, 0, delay, 0);
	Coolingfan_Set(COOLING_OFF);
	UsartPrintf(USART_DEBUG, "Cooling  fan test stop\r\n");

	

	UsartPrintf(USART_DEBUG, "Front Door test start\r\n");
	FrontDoor_Set(BOX_DOOR_OPEN);
	RTOS_TimeDlyHMSM(0, 0, delay, 0);
	FrontDoor_Set(BOX_DOOR_CLOSE);
	UsartPrintf(USART_DEBUG, "Front Door test stop\r\n");
	

	UsartPrintf(USART_DEBUG, "Back Door test start\r\n");
	BackDoor_Set(BOX_DOOR_OPEN);
	RTOS_TimeDlyHMSM(0, 0, delay, 0);
	BackDoor_Set(BOX_DOOR_CLOSE);
	UsartPrintf(USART_DEBUG, "Back Door test stop\r\n");


	UsartPrintf(USART_DEBUG, "Open Box Door test start\r\n");
	Door_Control_Set(MOTOR_RUN_BACKWARD);
	while(Door_Key_Detect(DOOR_OPEN) == SENSOR_NO_DETECT){
		RTOS_TimeDlyHMSM(0, 0, 0, 100);
		
		people_detect_time ++;
		if(people_detect_time >= 300)
		break;
	};
	Door_Control_Set(MOTOR_STOP);
	RTOS_TimeDlyHMSM(0, 0, delay * 2, 0);
	
	UsartPrintf(USART_DEBUG, "Close Box Door test start\r\n");
	people_detect_time = 0;
	Door_Control_Set(MOTOR_RUN_FORWARD);
	while(Door_Key_Detect(DOOR_CLOSE) == SENSOR_NO_DETECT)
	{
		if(Sensor_Detect() == SENSOR_DETECT)
		{
			UsartPrintf(USART_DEBUG, "Close Door Detect Somebody, Stop!!!!!!!!!!\r\n");
			Door_Control_Set(MOTOR_STOP);
		}
		else
		{
			Door_Control_Set(MOTOR_RUN_FORWARD);
			people_detect_time ++;
		}
		RTOS_TimeDlyHMSM(0, 0, 0, 100);

		if(people_detect_time >= 300)
		break;
	};
	Door_Control_Set(MOTOR_STOP);
	UsartPrintf(USART_DEBUG, "Box Door test stop\r\n");
}

void parse_board_test_request(uint8_t *outputdata, uint8_t *inputdata)  
{  
	uint8_t check_sum = 0;
	struct msg_ack_info_struct cmd_ack_info;
	struct motor_control_info_struct  motor_control;
	struct test_request_struct *test_request = (struct test_request_struct *)outputdata;

	cmd_ack_info.status = 0;
	
	test_request->start_code= inputdata[0];  
	test_request->packet_len= inputdata[1];  
	test_request->cmd_type= inputdata[2];
	
	check_sum = add_checksum(inputdata, test_request->packet_len);
	test_request->checksum = inputdata[test_request->packet_len];  
	
	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
	
	if(check_sum != test_request->checksum)
	{
		UsartPrintf(USART_DEBUG, "check sum fail : 0x%02x, 0x%02x\r\n", check_sum, test_request->checksum);  
		if(g_src_board_id == 1)
		send_command_ack(&cmd_ack_info, UART1_IDX);
		else
		send_command_ack(&cmd_ack_info, UART2_IDX);
		
		return;
	}
	
	cmd_ack_info.board_id = g_src_board_id;
	cmd_ack_info.rsp_cmd_type = test_request->cmd_type;
	cmd_ack_info.status = 1;

	if(g_src_board_id == 1)
	send_command_ack((void *)&cmd_ack_info, UART1_IDX);
	else
	send_command_ack((void *)&cmd_ack_info, UART2_IDX);
	
	test_request->info.board_id = inputdata[3];
	test_request->info.test_mode = inputdata[4];
	test_request->info.test_status = inputdata[5];

	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);

	UsartPrintf(USART_DEBUG, "board_id: 0x%02x\r\n", test_request->info.board_id);  
	
	if(test_request->info.board_id == g_src_board_id)
	{
		motor_control.board_id = test_request->info.board_id;

		UsartPrintf(USART_DEBUG, "push_time[0x%02x][0x%02x]\r\n",inputdata[7], inputdata[8]);

		motor_control.medicine_track_number = test_request->info.medicine_track_number = inputdata[6];
		motor_control.push_time = test_request->info.test_time= inputdata[7]<<8|inputdata[8];



		
		UsartPrintf(USART_DEBUG, "test mode[%d]\r\n",test_request->info.test_mode);
		if(test_request->info.test_mode == TRACK_TEST)
		{
			if(motor_control.push_time >= TRACK_MAX_RUN_TIME)
			motor_control.push_time = TRACK_MAX_RUN_TIME;
			
			if(test_request->info.test_status == 0)
			{
				SetTrackTestTime(motor_control.medicine_track_number, MOTOR_RUN_FORWARD, motor_control.push_time);
				//Track_run_only(MOTOR_RUN_FORWARD);
				//CleanTrackParam();
			}
			else
			{
				SetTrackTestTime(motor_control.medicine_track_number, MOTOR_RUN_BACKWARD, motor_control.push_time);
				//Track_run_only(MOTOR_RUN_BACKWARD);
				//CleanTrackParam();
			}
			OSSemPost(SemOfTrack);
		}
		else if(test_request->info.test_mode == PUSH_BELT_TEST)
		{
			if(test_request->info.test_status == BOX_TEXT_MODE_OPEN)
			{
				Belt_Set(PUSH_BELT, BELT_RUN);
			}
			else if(test_request->info.test_status == BOX_TEXT_MODE_CLOSE)
			{
				Belt_Set(PUSH_BELT, BELT_STOP);
			}
		}
		else if(test_request->info.test_mode == COLLECT_BELT_TEST)
		{
			if(test_request->info.test_status == BOX_TEXT_MODE_OPEN)
			{
				Belt_Set(COLLECT_BELT, BELT_RUN);
			}
			else if(test_request->info.test_status == BOX_TEXT_MODE_CLOSE)
			{
				Belt_Set(COLLECT_BELT, BELT_STOP);
			}
		}
		else if(test_request->info.test_mode == COMPRESSOR_TEST)
		{
			if(test_request->info.test_status == BOX_TEXT_MODE_OPEN)
			{
				
				Coolingcompressor_Set(COOLING_ON);
			}
			else if(test_request->info.test_status == BOX_TEXT_MODE_CLOSE)
			{
				Coolingcompressor_Set(COOLING_OFF);
			}

		}
		else if(test_request->info.test_mode == FAN_TEST)
		{
			if(test_request->info.test_status == BOX_TEXT_MODE_OPEN)
			{
				Coolingfan_Set(COOLING_ON);
			}
			else if(test_request->info.test_status == BOX_TEXT_MODE_CLOSE)
			{
				Coolingfan_Set(COOLING_OFF);
			}

		}
		else if(test_request->info.test_mode == FRONT_DOOR_TEST)
		{
			if(test_request->info.test_status == BOX_TEXT_MODE_OPEN)
			{
				FrontDoor_Set(BOX_DOOR_OPEN);
			}
			else if(test_request->info.test_status == BOX_TEXT_MODE_CLOSE)
			{
				FrontDoor_Set(BOX_DOOR_CLOSE);
			}
		}
		else if(test_request->info.test_mode == BACK_DOOR_TEST)
		{
			if(test_request->info.test_status == BOX_TEXT_MODE_OPEN)
			{
				BackDoor_Set(BOX_DOOR_OPEN);
			}
			else if(test_request->info.test_status == BOX_TEXT_MODE_CLOSE)
			{
				BackDoor_Set(BOX_DOOR_CLOSE);
			}
		}
		else if(test_request->info.test_mode == DRUG_DOOR_TEST)
		{
			uint16_t run_time = 0;
			if(test_request->info.test_status == BOX_TEXT_MODE_OPEN)
			{
				Door_Control_Set(MOTOR_RUN_BACKWARD);
				while(Door_Key_Detect(DOOR_OPEN) == SENSOR_NO_DETECT){
					RTOS_TimeDlyHMSM(0, 0, 0, 100);
					
					run_time += 1;					
					if(run_time >= 300)
					break;
				};
				Door_Control_Set(MOTOR_STOP);
			}
			else if(test_request->info.test_status == BOX_TEXT_MODE_CLOSE)
			{
				Door_Control_Set(MOTOR_RUN_FORWARD);
				while(Door_Key_Detect(DOOR_CLOSE) == SENSOR_NO_DETECT)
				{
					if(Sensor_Detect() == SENSOR_DETECT)
					{
						UsartPrintf(USART_DEBUG, "Close Door Detect Somebody, Stop!!!!!!!!!!\r\n");
						Door_Control_Set(MOTOR_STOP);
						RTOS_TimeDlyHMSM(0, 0, 10, 0);
					}
					else
					{
						Door_Control_Set(MOTOR_RUN_FORWARD);
						run_time += 1;					
					}
					RTOS_TimeDlyHMSM(0, 0, 0, 100);
					
					if(run_time >= 300)
					break;
				};
				Door_Control_Set(MOTOR_STOP);
			}
		}
		else if(test_request->info.test_mode == LIGHT_TEST)
		{
			if(test_request->info.test_status == BOX_TEXT_MODE_OPEN)
			{
				Light_Set(LIGHT_ON);
			}
			else if(test_request->info.test_status == BOX_TEXT_MODE_CLOSE)
			{
				Light_Set(LIGHT_OFF);
			}
		}
		else if(test_request->info.test_mode == LIFTER_TEST)
		{
			if(test_request->info.test_status == LIFTER_UP)
			{
				Lifter_Set(LIFTER_UP);
			}
			else if(test_request->info.test_status == LIFTER_FALL)
			{
				Lifter_Set(LIFTER_FALL);
			}
		}
		else if(test_request->info.test_mode == CALIBRATE_TRACK_TEST)
		{
			if(calibrate_track_selected == 255)
			{
				calibrate_track_selected = motor_control.medicine_track_number;
				OSSemPost(SemOfKey);
			}
			else
				calibrate_track_selected = 255;

			UsartPrintf(USART_DEBUG, "calibrate_track_selected[%d]\r\n", calibrate_track_selected);
		}
		else if(test_request->info.test_mode == FACTORY_TEST)
		{
			UsartPrintf(USART_DEBUG, "Factory test!!!!!!!!!!!!!\r\n");
			OSSemPost(SemOfFactoryTest);
		}

	}
			
	return;
}


void print_board_test_request(uint8_t *data)  
{  
	struct test_request_struct *test_request = (struct test_request_struct *)data;

    UsartPrintf(USART_DEBUG, "\r\n    ========= ������Ա���-��ӡ��ʼ=========\r\n");  
	UsartPrintf(USART_DEBUG, "\r\n	  ����Ϣ����:0x%02x", test_request->cmd_type);  
	UsartPrintf(USART_DEBUG, "\r\n	  ���ܳ���:0x%02x", test_request->packet_len); 

    UsartPrintf(USART_DEBUG, "\r\n    �����:0x%02x", test_request->info.board_id); 
    UsartPrintf(USART_DEBUG, "\r\n    ����ģʽ:0x%02x", test_request->info.test_mode); 
    UsartPrintf(USART_DEBUG, "\r\n    ���Ի���:0x%02x", test_request->info.medicine_track_number); 
	
    UsartPrintf(USART_DEBUG, "\r\n    ����ʱ��:0x%04x", test_request->info.test_time); 
	
	UsartPrintf(USART_DEBUG, "\r\n    ��У���:0x%02x\r\n", test_request->checksum);  
  
    UsartPrintf(USART_DEBUG, "\r\n    =========  ������Ա���-��ӡ����=========\r\n");  
}




void print_push_medicine_request(uint8_t *data)  
{  
  	uint8_t request_cnt = 0;
	uint8_t i = 0;
	struct push_medicine_request_struct *push_medicine_request = (struct push_medicine_request_struct *)data;

	request_cnt = (push_medicine_request->packet_len - IPUC)/PUSH_MEDICINE_REQUEST_INFO_SIZE;

    UsartPrintf(USART_DEBUG, "\r\n    ========= ����������-��ӡ��ʼ=========\r\n");  
	UsartPrintf(USART_DEBUG, "\r\n	  ����Ϣ����:0x%02x", push_medicine_request->cmd_type);  
	UsartPrintf(USART_DEBUG, "\r\n	  ���ܳ���:0x%02x", push_medicine_request->packet_len); 
	for(i = 0; i < request_cnt; i++)
	{
	    UsartPrintf(USART_DEBUG, "\r\n    �����:0x%04x", push_medicine_request->info[i].board_id); 
	    UsartPrintf(USART_DEBUG, "\r\n    �����˻�����:0x%04x", push_medicine_request->info[i].medicine_track_number); 
	    UsartPrintf(USART_DEBUG, "\r\n    ��������ʱ��:0x%04x", push_medicine_request->info[i].push_time); 
	}
	UsartPrintf(USART_DEBUG, "\r\n    ��У���:0x%02x\r\n", push_medicine_request->checksum);  
  
    UsartPrintf(USART_DEBUG, "\r\n    =========  ����������-��ӡ����=========\r\n");  
 }  

void parse_push_medicine_request(uint8_t *outputdata, uint8_t *inputdata)  
{  
	uint8_t check_sum = 0;
	uint8_t try_cnt = 0;
	uint8_t x = 0;
	uint8_t y = 0;
	struct msg_ack_info_struct cmd_ack_info;
	struct push_medicine_request_struct *push_medicine_request = (struct push_medicine_request_struct *)outputdata;
	uint8_t x_track_is_run = 0;

	cmd_ack_info.status = 0;
	push_medicine_request->start_code= inputdata[0];  

	push_medicine_request->packet_len= inputdata[1];  
	push_medicine_request->cmd_type= inputdata[2];
	
	check_sum = add_checksum(inputdata, push_medicine_request->packet_len);
	push_medicine_request->checksum = inputdata[push_medicine_request->packet_len];  
	
	cmd_ack_info.board_id = g_src_board_id;
	cmd_ack_info.rsp_cmd_type = push_medicine_request->cmd_type;
	
	if(check_sum != push_medicine_request->checksum)
	{
		UsartPrintf(USART_DEBUG, "check sum fail : 0x%02x, 0x%02x\r\n", check_sum, push_medicine_request->checksum);  

		if(g_src_board_id == 1)
		send_command_ack(&cmd_ack_info, UART1_IDX);
		else
		send_command_ack(&cmd_ack_info, UART2_IDX);
		
		return;
	}

	while((motor_enqueue_idx == motor_dequeue_idx - 1) && (try_cnt <= 5))
	{
		RTOS_TimeDlyHMSM(0, 0, 0, 100);
		try_cnt++;
	}
	
	if(try_cnt == 5)
	{
		if(g_src_board_id == 1)
		send_command_ack(&cmd_ack_info, UART1_IDX);
		else
		send_command_ack(&cmd_ack_info, UART2_IDX);
		
		return;
	}
	
	push_medicine_request->info[0].board_id = inputdata[3];
	UsartPrintf(USART_DEBUG, "board_id: 0x%02x, 0x%02x\r\n", push_medicine_request->info[0].board_id, g_src_board_id);  

	
	if(push_medicine_request->info[0].board_id == g_src_board_id)
	{
		push_medicine_request->info[0].medicine_track_number = inputdata[4];
		push_medicine_request->info[0].push_time = inputdata[5]<<8|inputdata[6];

		if((push_medicine_request->info[0].medicine_track_number != 0) && (push_medicine_request->info[0].push_time != 0))
		{
			//motor_struct[motor_enqueue_idx].motor_run = MOTOR_RUN_FORWARD;
			//motor_struct[motor_enqueue_idx].motor_work_mode = CMD_PUSH_MEDICINE_REQUEST;
			//memcpy(&motor_struct[motor_enqueue_idx].info, &push_medicine_request->info[valid_cnt], sizeof(struct motor_control_info_struct));

			//motor_enqueue_idx++;
			//if(motor_enqueue_idx >= TOTAL_PUSH_CNT)
			//motor_enqueue_idx = 0;	

			x = (push_medicine_request->info[0].medicine_track_number - 1)/10;
			y = (push_medicine_request->info[0].medicine_track_number - 1)%10;
			UsartPrintf(USART_DEBUG, "medicine_track_number = %d, track_struct[%d][%d].push_time = %d\r\n", push_medicine_request->info[0].medicine_track_number, x, y, track_struct[x][y].push_time);
			track_struct[x][y].motor_run = MOTOR_RUN_FORWARD;
			track_struct[x][y].medicine_track_number = push_medicine_request->info[0].medicine_track_number;
			track_struct[x][y].push_time = push_medicine_request->info[0].push_time;
			cmd_ack_info.status = 1;
			
			TrunkInitTime = time_passes;
		}
		/*02 07 20 1 0 0 xx*//*��������*/
		else if((push_medicine_request->info[0].medicine_track_number == 0) && (push_medicine_request->info[0].push_time == 0))
		{
			/*���μ�����ʱ���Ƿ�ȫΪ0����0���������������߳�*/
			/*��鵱ǰ�����Ƿ�Ҫ����*/
			for(x = 0; x < 10; x++)
			{
				for(y = 0; y < 10; y++)
				{
					if(track_struct[x][y].push_time)
					{
						x_track_is_run = 1;
						break;
					}
				}
			}
			
			if(x_track_is_run)
			OSSemPost(SemOfTrack);

			/*��ʼ���Ѵ��ʹ���ȡ���ڶ���*/
			if(1 == g_src_board_id)
			OSSemPost(SemOfConveyor);
			
			cmd_ack_info.status = 1;

			TrunkInitTime = 0;
			track_work = MOTOR_RUN_FORWARD;
			UsartPrintf(USART_DEBUG, "Receive push complete!!!!!!!!!!!!\r\n");
		}
	}

	
	cmd_ack_info.board_id = g_src_board_id;
	cmd_ack_info.rsp_cmd_type = push_medicine_request->cmd_type;


	if(g_src_board_id == 1)
	send_command_ack(&cmd_ack_info, UART1_IDX);
	else
	send_command_ack(&cmd_ack_info, UART2_IDX);
	
	return;
}


void send_command_ack( void *input_data, uint8_t uart_idx)  
{  
	uint8_t send_cmd_ack_data[COMMAND_ACK_PACKET_SIZE];
	struct msg_ack_info_struct *cmd_ack_info = (struct msg_ack_info_struct * )input_data;

	memset(send_cmd_ack_data, 0x00, COMMAND_ACK_PACKET_SIZE);
	
	send_cmd_ack_data[0] = START_CODE;
	send_cmd_ack_data[1] = COMMAND_ACK_PACKET_SIZE - 1;
	send_cmd_ack_data[2] = CMD_MSG_ACK;

	send_cmd_ack_data[3] = cmd_ack_info->rsp_cmd_type;
	
	send_cmd_ack_data[4] = cmd_ack_info->board_id;
	
	send_cmd_ack_data[5] = cmd_ack_info->status;

	send_cmd_ack_data[COMMAND_ACK_PACKET_SIZE - 1] = add_checksum(send_cmd_ack_data, COMMAND_ACK_PACKET_SIZE - 1);  

	UsartPrintf(USART_DEBUG, "send_command_ack uart idx:%d\r\n", uart_idx);

	if(uart_idx == UART1_IDX)
	{
		UART1_IO_Send(send_cmd_ack_data, COMMAND_ACK_PACKET_SIZE); 
	} 
	else if(uart_idx == UART2_IDX)
	{
		if(g_src_board_id == 1)
		RS485_Send_Data(send_cmd_ack_data, COMMAND_ACK_PACKET_SIZE);	
		else
		NotRetryMessageInsertQueue(send_cmd_ack_data, COMMAND_ACK_PACKET_SIZE, UART2_IDX);
	}
} 


void print_replenish_medicine_request(uint8_t *data)  
{  
  	uint8_t request_cnt = 0;
	uint8_t i = 0;
	struct replenish_medicine_request_struct *replenish_medicine_request = (struct replenish_medicine_request_struct *)data;
	

	request_cnt = (replenish_medicine_request->packet_len - IPUC)/REPLENISH_MEDICINE_REQUEST_INFO_SIZE;

    UsartPrintf(USART_DEBUG, "\r\n    ========= ����������-��ӡ��ʼ=========\r\n");  
	UsartPrintf(USART_DEBUG, "\r\n	  ����Ϣ����:0x%02x", replenish_medicine_request->cmd_type);  
	UsartPrintf(USART_DEBUG, "\r\n	  ���ܳ���:0x%02x", replenish_medicine_request->packet_len); 
	for(i = 0; i < request_cnt; i++)
	{
	    UsartPrintf(USART_DEBUG, "\r\n    �����:0x%04x", replenish_medicine_request->info[i].board_id); 
	    UsartPrintf(USART_DEBUG, "\r\n    �����˻�����:0x%04x", replenish_medicine_request->info[i].medicine_track_number); 
	    UsartPrintf(USART_DEBUG, "\r\n    ������з���:0x%04x", replenish_medicine_request->info[i].dirtion); 
		UsartPrintf(USART_DEBUG, "\r\n    �������ʱ��:0x%04x", replenish_medicine_request->info[i].push_time); 
	}
	UsartPrintf(USART_DEBUG, "\r\n    ��У���:0x%02x\r\n", replenish_medicine_request->checksum);  
  
    UsartPrintf(USART_DEBUG, "\r\n    =========  ����������-��ӡ����=========\r\n");  
 }  

void parse_replenish_medicine_request(uint8_t *outputdata, uint8_t *inputdata)
{  
	uint8_t i = 0;
	uint8_t check_sum = 0;
	struct msg_ack_info_struct cmd_ack_info;
	uint8_t x = 0;
	uint8_t y = 0;
	struct replenish_medicine_request_struct *replenish_medicine_request = (struct replenish_medicine_request_struct *)outputdata;

	
	cmd_ack_info.status = 0;

	
	replenish_medicine_request->start_code= inputdata[0];  

	replenish_medicine_request->packet_len= inputdata[1];  
	replenish_medicine_request->cmd_type= inputdata[2];
	
	check_sum = add_checksum(inputdata, replenish_medicine_request->packet_len);
	replenish_medicine_request->checksum = inputdata[replenish_medicine_request->packet_len];  


	cmd_ack_info.board_id = g_src_board_id;
	cmd_ack_info.rsp_cmd_type = replenish_medicine_request->cmd_type;
	
	if(check_sum != replenish_medicine_request->checksum)
	{	
		UsartPrintf(USART_DEBUG, "check sum fail : 0x%02x, 0x%02x\r\n", check_sum, replenish_medicine_request->checksum);  
		if(g_src_board_id == 1)
		send_command_ack(&cmd_ack_info, UART1_IDX);
		else
		send_command_ack(&cmd_ack_info, UART2_IDX);
		
		return;
	}
		
	//request_cnt = (replenish_medicine_request->packet_len - IPUC)/REPLENISH_MEDICINE_REQUEST_INFO_SIZE;
	//UsartPrintf(USART_DEBUG, "request_cnt: 0x%02x\r\n", request_cnt);  

	//for(i = 0; i < request_cnt; i++)
	{
		replenish_medicine_request->info[0].board_id = inputdata[3 + i * REPLENISH_MEDICINE_REQUEST_INFO_SIZE];
		UsartPrintf(USART_DEBUG, "board_id: 0x%02x\r\n", replenish_medicine_request->info[0].board_id);  
		
		if(replenish_medicine_request->info[0].board_id == g_src_board_id)
		{
			replenish_medicine_request->info[0].medicine_track_number = inputdata[4 + i * REPLENISH_MEDICINE_REQUEST_INFO_SIZE];
			replenish_medicine_request->info[0].dirtion = inputdata[5 + i * REPLENISH_MEDICINE_REQUEST_INFO_SIZE];
			replenish_medicine_request->info[0].push_time = inputdata[6 + i * REPLENISH_MEDICINE_REQUEST_INFO_SIZE]<<8|inputdata[7 + i * REPLENISH_MEDICINE_REQUEST_INFO_SIZE];

			if((replenish_medicine_request->info[0].medicine_track_number != 0) && (replenish_medicine_request->info[0].push_time != 0))
			{
				//motor_struct[motor_enqueue_idx].motor_run = MOTOR_RUN_BACKWARD;
				//motor_struct[motor_enqueue_idx].motor_work_mode = CMD_REPLENISH_MEDICINE_REQUEST;
				//memcpy(&motor_struct[motor_enqueue_idx].info, &replenish_medicine_request->info[valid_cnt], sizeof(struct motor_control_info_struct));
								
				//motor_enqueue_idx++;
				//if(motor_enqueue_idx >= TOTAL_PUSH_CNT)
				//motor_enqueue_idx = 0;

				x = (replenish_medicine_request->info[0].medicine_track_number - 1)/10;
				y = (replenish_medicine_request->info[0].medicine_track_number - 1)%10;
				track_struct[x][y].motor_run = replenish_medicine_request->info[0].dirtion;
				track_struct[x][y].medicine_track_number = replenish_medicine_request->info[0].medicine_track_number;
				track_struct[x][y].push_time = replenish_medicine_request->info[0].push_time;
				
				TrunkInitTime = time_passes;
			}
			else if((replenish_medicine_request->info[0].medicine_track_number == 0) && (replenish_medicine_request->info[0].push_time == 0))
			{
				OSSemPost(SemOfTrack);
				cmd_ack_info.status = 1;
			
				track_work = MOTOR_RUN_BACKWARD;
				TrunkInitTime = 0;
				UsartPrintf(USART_DEBUG, "Receive replenish complete!!!!!!!!!!!!\r\n");
			}
		}


	
		cmd_ack_info.status = 1;
	}
	cmd_ack_info.board_id = g_src_board_id;
	cmd_ack_info.rsp_cmd_type = replenish_medicine_request->cmd_type;
	
	if(g_src_board_id == 1)
	send_command_ack(&cmd_ack_info, UART1_IDX);
	else
	send_command_ack(&cmd_ack_info, UART2_IDX);
	
	return;
}


void print_replenish_complete_request(uint8_t *data)
{  
	struct replenish_medicine_complete_struct *replenish_medicine_complete_request = (struct replenish_medicine_complete_struct *)data;
	

    UsartPrintf(USART_DEBUG, "\r\n    ========= ������ɱ���-��ӡ��ʼ=========\r\n");  
	UsartPrintf(USART_DEBUG, "\r\n	  ����Ϣ����:0x%02x", replenish_medicine_complete_request->cmd_type);  
	UsartPrintf(USART_DEBUG, "\r\n	  ���ܳ���:0x%02x", replenish_medicine_complete_request->packet_len); 

    UsartPrintf(USART_DEBUG, "\r\n    �����:0x%02x", replenish_medicine_complete_request->info.board_id); 
		
	UsartPrintf(USART_DEBUG, "\r\n    ��У���:0x%02x\r\n", replenish_medicine_complete_request->checksum);  
  
    UsartPrintf(USART_DEBUG, "\r\n    =========  ������ɱ���-��ӡ����=========\r\n");  
}


void parse_replenish_complete_request(uint8_t *outputdata, uint8_t *inputdata)
{  
	uint8_t check_sum = 0;
	struct msg_ack_info_struct cmd_ack_info;
	struct replenish_medicine_complete_struct *replenish_medicine_complete_request = (struct replenish_medicine_complete_struct *)outputdata;

	cmd_ack_info.status = 0;
	
	replenish_medicine_complete_request->start_code = inputdata[0];  
	replenish_medicine_complete_request->packet_len = inputdata[1];  
	replenish_medicine_complete_request->cmd_type = inputdata[2];
	
	check_sum = add_checksum(inputdata, replenish_medicine_complete_request->packet_len);
	replenish_medicine_complete_request->checksum = inputdata[replenish_medicine_complete_request->packet_len];  
	
	//UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
	cmd_ack_info.board_id = g_src_board_id;
	cmd_ack_info.rsp_cmd_type = replenish_medicine_complete_request->cmd_type;
	if(check_sum != replenish_medicine_complete_request->checksum)
	{
		
		UsartPrintf(USART_DEBUG, "check sum fail : 0x%02x, 0x%02x\r\n", check_sum, replenish_medicine_complete_request->checksum);  
		if(g_src_board_id == 1)
			send_command_ack(&cmd_ack_info, UART1_IDX);
			else
			send_command_ack(&cmd_ack_info, UART2_IDX);
		return;
	}
	
	replenish_medicine_complete_request->info.board_id = inputdata[3];

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
	if(g_src_board_id == 1)
		send_command_ack((void *)&cmd_ack_info, UART1_IDX);
	else
	send_command_ack(&cmd_ack_info, UART2_IDX);
	//UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
	
	return;
}


void parse_track_runtime_calc_request(uint8_t *outputdata, uint8_t *inputdata)  
{  
	struct msg_ack_info_struct cmd_ack_info;
	struct track_calc_request_struct *track_runtime_calc_request = (struct track_calc_request_struct *)outputdata;


	
	cmd_ack_info.status = 0;

	track_runtime_calc_request->start_code= inputdata[0];  

	track_runtime_calc_request->packet_len= inputdata[1];  
	track_runtime_calc_request->cmd_type= inputdata[2];

	track_runtime_calc_request->info.board_id = inputdata[3];
	
	track_runtime_calc_request->info.track_start_num = inputdata[4];

	track_runtime_calc_request->info.track_count = inputdata[5];

	track_runtime_calc_request->checksum = inputdata[6]; 

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
	
	if(g_src_board_id == 1)
		send_command_ack((void *)&cmd_ack_info, UART1_IDX);
	else
	send_command_ack(&cmd_ack_info, UART2_IDX);

	OSSemPost(SemOfCalcTime);
}  

void print_track_runtime_calc_request(uint8_t *data)  
{  
	struct track_calc_request_struct *track_runtime_calc_request = (struct track_calc_request_struct *)data;

    UsartPrintf(USART_DEBUG, "\r\n    ========= ��������ʱ��ͳ��������-��ӡ��ʼ=========\r\n");  
	UsartPrintf(USART_DEBUG, "\r\n	  ����Ϣ����:0x%02x", track_runtime_calc_request->cmd_type);  
	UsartPrintf(USART_DEBUG, "\r\n	  ���ܳ���:0x%02x", track_runtime_calc_request->packet_len); 
    UsartPrintf(USART_DEBUG, "\r\n    �����:0x%04x", track_runtime_calc_request->info.board_id); 
    UsartPrintf(USART_DEBUG, "\r\n    ������ʼ�˻�����:0x%04x", track_runtime_calc_request->info.track_start_num); 
    UsartPrintf(USART_DEBUG, "\r\n    ���������:0x%04x", track_runtime_calc_request->info.track_count); 
	UsartPrintf(USART_DEBUG, "\r\n    ��У���:0x%02x\r\n", track_runtime_calc_request->checksum);  
  
    UsartPrintf(USART_DEBUG, "\r\n    =========  ��������ʱ��ͳ��������-��ӡ����=========\r\n");  
 }  


void parse_message_ack(uint8_t *outputdata, uint8_t *inputdata)  
{	
	struct msg_ack_struct *cmd_ack = (struct msg_ack_struct *)outputdata;

	cmd_ack->start_code= inputdata[0];  

	cmd_ack->packet_len= inputdata[1];  
	cmd_ack->cmd_type= inputdata[2];

	cmd_ack->ack.board_id = inputdata[3];
	cmd_ack->ack.rsp_cmd_type = inputdata[4];
	cmd_ack->ack.status = inputdata[5];	
}	


void print_status_report_request(uint8_t *data)	
{	
	struct status_report_request_struct  *status_report_request = (struct status_report_request_struct  *)data;

	UsartPrintf(USART_DEBUG, "\r\n    ========= ״̬�ϱ���-��ӡ��ʼ=========\r\n");  

	UsartPrintf(USART_DEBUG, "\r\n    ����Ϣ����:0x%02x", status_report_request->cmd_type);  
	UsartPrintf(USART_DEBUG, "\r\n    ���ܳ���:0x%02x", status_report_request->packet_len);  
	UsartPrintf(USART_DEBUG, "\r\n    ��У���:0x%02x\r\n", status_report_request->checksum);	

	UsartPrintf(USART_DEBUG, "\r\n    =========  ״̬�ϱ���-��ӡ����=========\r\n");  
}  









void send_query_message(uint8_t id)  
{  
	struct query_struct query_struct;
	
	memset(&query_struct, 0x00, sizeof(struct query_struct));
	query_struct.start_code = START_CODE;
	query_struct.packet_len = QUERY_REQUEST_PACKET_SIZE - 1;
	query_struct.cmd_type = CMD_QUERY_MSG;
	query_struct.board_id = id;
	
	query_struct.checksum = add_checksum((unsigned char *)&query_struct, QUERY_REQUEST_PACKET_SIZE - 1);  
	RS485_Send_QueryData((u8 *)(&query_struct), QUERY_REQUEST_PACKET_SIZE);
} 





