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
#include "box.h"

#include "stm32_protocol.h"
#include "protocol_func.h"




/*  
    累加校验和算法  
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
	
	if(0)//(check_sum != test_request->checksum)
	{
		UsartPrintf(USART_DEBUG, "check sum fail : 0x%02x, 0x%02x\r\n", check_sum, test_request->checksum);  
		send_command_ack(&cmd_ack_info);
		return;
	}
	
	test_request->info.board_id = inputdata[3];
	test_request->info.test_mode = inputdata[4];

	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);

	UsartPrintf(USART_DEBUG, "board_id: 0x%02x\r\n", test_request->info.board_id);  
	
	if(test_request->info.board_id == g_src_board_id)
	{
		motor_control.board_id = test_request->info.board_id;

		UsartPrintf(USART_DEBUG, "push_time[0x%02x][0x%02x]\r\n",inputdata[6], inputdata[7]);

		motor_control.medicine_track_number = test_request->info.medicine_track_number = inputdata[5];
		motor_control.push_time = test_request->info.test_time= inputdata[6]<<8|inputdata[7];

		UsartPrintf(USART_DEBUG, "test mode[%d]\r\n",test_request->info.test_mode);
		if(test_request->info.test_mode == TRACK_TEST)
		{
				if(test_request->info.test_status == 0)
				motor_struct[motor_enqueue_idx].motor_run = MOTOR_RUN_FORWARD;
				else
				motor_struct[motor_enqueue_idx].motor_run = MOTOR_RUN_BACKWARD;


				motor_struct[motor_enqueue_idx].motor_work_mode = CMD_TEST_REQUEST;
				
				memcpy(&motor_struct[motor_enqueue_idx].info, &motor_control, sizeof(struct motor_control_info_struct));

				UsartPrintf(USART_DEBUG, "board_id = %d\r\n",motor_struct[motor_enqueue_idx].info.board_id);
				UsartPrintf(USART_DEBUG, "medicine_track_number = %d\r\n",motor_struct[motor_enqueue_idx].info.medicine_track_number);
				UsartPrintf(USART_DEBUG, "push_time = %d\r\n",motor_struct[motor_enqueue_idx].info.push_time);

				
				motor_enqueue_idx++;
				if(motor_enqueue_idx >= TOTAL_PUSH_CNT)
				motor_enqueue_idx = 0;

				UsartPrintf(USART_DEBUG, "send sem,motor_enqueue_idx[%d]-------------\r\n", motor_enqueue_idx);
				OSSemPost(SemOfMotor);
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
				Belt_Set(COLLECT_BELT, BELT_RUN);
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
				Door_Set(BOX_DOOR_OPEN);
			}
			else if(test_request->info.test_status == BOX_TEXT_MODE_CLOSE)
			{
				Door_Set(BOX_DOOR_CLOSE);
			}

		}
		else if(test_request->info.test_mode == DRUG_DOOR_TEST)
		{
			if(test_request->info.test_status == BOX_TEXT_MODE_OPEN)
			{
				Door_Control_Set(MOTOR_RUN_FORWARD);
			}
			else if(test_request->info.test_status == BOX_TEXT_MODE_CLOSE)
			{
				Door_Control_Set(MOTOR_RUN_BACKWARD);
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

	}
	
	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
	
	cmd_ack_info.board_id = g_src_board_id;
	cmd_ack_info.rsp_cmd_type = test_request->cmd_type;

	
	cmd_ack_info.status = 1;
	send_command_ack((void *)&cmd_ack_info);
	
	UsartPrintf(USART_DEBUG, "%s[%d]\r\n", __FUNCTION__, __LINE__);
	
	return;
}


void print_board_test_request(uint8_t *data)  
{  
	struct test_request_struct *test_request = (struct test_request_struct *)data;

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







