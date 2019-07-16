#ifndef _STM32_PROTOCOL_H_
#define _STM32_PROTOCOL_H_

#include "stm32f10x.h"





#define STX_CODE_SIZE 1  
#define PACKET_LEN_SIZE 1  
#define USER_CMD_SIZE 1  
#define CHECKSUM_SIZE 1 

#define MAX_PUSH_CNT	16
#define TOTAL_PUSH_CNT	(100)

#define TRACK_MAX	96
#define BOARD_ID_MAX 16
#define PUSH_TIME_MAX 200


//ͨ�ñ��ĳ���
#define IPUC (STX_CODE_SIZE + PACKET_LEN_SIZE + USER_CMD_SIZE)  


/*������״̬�ϱ� */
#define STATUS_REPORT_REQUEST_INFO_SIZE 4 //��(1�ֽ�	1�ֽ�	1�ֽ�	1�ֽ�)   
#define STATUS_REPORT_REQUEST_PACKET_SIZE (IPUC + STATUS_REPORT_REQUEST_INFO_SIZE + CHECKSUM_SIZE)  


/*�������ָ��*/
#define PUSH_MEDICINE_REQUEST_INFO_SIZE 4 //��(1�ֽ�	1�ֽ�	2�ֽ�)   
#define PUSH_MEDICINE_REQUEST_PACKET_SIZE (IPUC + PUSH_MEDICINE_REQUEST_INFO_SIZE + CHECKSUM_SIZE)  


/*���󲹻�ָ��*/
#define REPLENISH_MEDICINE_REQUEST_INFO_SIZE 4 //��(1�ֽ�	1�ֽ�	2�ֽ�)   
#define REPLENISH_MEDICINE_REQUEST_PACKET_SIZE (IPUC + REPLENISH_MEDICINE_REQUEST_INFO_SIZE + CHECKSUM_SIZE)  

/*����У׼ָ��*/
#define CILIBRATE_TRACK_REQUEST_INFO_SIZE 2 //��(1�ֽ�	1�ֽ�)   
#define CILIBRATE_TRACK_REQUEST_PACKET_SIZE (IPUC + CILIBRATE_TRACK_REQUEST_INFO_SIZE + CHECKSUM_SIZE)  


/*���Զ���ָ��*/
#define BOARD_TEST_REQUEST_INFO_SIZE 5 //��(1�ֽ�	1�ֽ�	1�ֽ�	2�ֽ�)   
#define BOARD_TEST_REQUEST_PACKET_SIZE (IPUC + BOARD_TEST_REQUEST_INFO_SIZE + CHECKSUM_SIZE)  


/*���ò���ָ��*/
#define SETTINT_REQUEST_INFO_SIZE 3 //��(1�ֽ�	1�ֽ�	1�ֽ�)   
#define SETTING_REQUEST_PACKET_SIZE (IPUC + SETTINT_REQUEST_INFO_SIZE + CHECKSUM_SIZE)  


/*�������*/
#define PUSH_MEDICINE_COMPLETE_INFO_SIZE 3 //��(1�ֽ�	1�ֽ�	1�ֽ�)   
#define PUSH_MEDICINE_COMPLETE_PACKET_SIZE (IPUC + PUSH_MEDICINE_COMPLETE_INFO_SIZE + CHECKSUM_SIZE)  


/*�������*/
#define REPLENISH_MEDICINE_CONPLETE_REQUEST_INFO_SIZE 2 //��(1�ֽ�	�ֽ�)   
#define REPLENISH_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE (IPUC + REPLENISH_MEDICINE_CONPLETE_REQUEST_INFO_SIZE + CHECKSUM_SIZE)  

/*��������ʱ��ͳ������*/
#define TRACK_RUNTIME_CALC_REQUEST_INFO_SIZE 3 //��(3�ֽ�	�ֽ�)   
#define TRACK_RUNTIME_CALC_REQUEST_PACKET_SIZE (IPUC + TRACK_RUNTIME_CALC_REQUEST_INFO_SIZE + CHECKSUM_SIZE)  


/*��������ʱ��ͳ���ϱ�*/
#define TRACK_RUNTIME_CALC_REPORT_INFO_SIZE 6 //��(3�ֽ�	�ֽ�)   
#define TRACK_RUNTIME_CALC_REPORT_PACKET_SIZE (IPUC + TRACK_RUNTIME_CALC_REPORT_INFO_SIZE + CHECKSUM_SIZE)  



/*ָ��Ӧ��*/
#define COMMAND_ACK_INFO_SIZE 3 //��(1�ֽ�	1�ֽ�	1�ֽ�)   
#define COMMAND_ACK_PACKET_SIZE (IPUC + COMMAND_ACK_INFO_SIZE + CHECKSUM_SIZE)  


/*��ѯָ��*/
#define QUERY_REQUEST_INFO_SIZE 1 //��(2�ֽ�	2�ֽ�	2�ֽ�)   
#define QUERY_REQUEST_PACKET_SIZE (IPUC + QUERY_REQUEST_INFO_SIZE + CHECKSUM_SIZE)  

/*��Ƭ���ڲ��������*/
#define ADD_MEDICINE_CONPLETE_REQUEST_INFO_SIZE 3 //��(1�ֽ�	�ֽ�)   
#define ADD_MEDICINE_CONPLETE_REQUEST_PACKET_SIZE (IPUC + ADD_MEDICINE_CONPLETE_REQUEST_INFO_SIZE + CHECKSUM_SIZE)  





enum {  
    STATUS_REPORT_REQUEST = 0,  
	PUSH_MEDICINE_REQUEST,
	REPLENISH_MEDICINE_REQUEST,
	CILIBRATE_TRACK_REQUEST,
	TEST_BOARD_REQUEST,
	SETTING_REQUEST,
	PUSH_MEDICINE_COMPLETE_REQUEST,
	REPLENISH_MEDICINE_COMPLETE_REQUEST,
	MCU_REPLENISH_MEDICINE_COMPLETE_REQUEST,
	CMD_ACK,
};  



enum {   
    STATUS_REPORT_REQUEST_BUF = (1 << 0),  
	PUSH_MEDICINE_REQUEST_BUF= (1 << 1),
	REPLENISH_MEDICINE_REQUEST_BUF= (1 << 2),
	CALIBRATE_TRACK_REQUEST_BUF= (1 << 3),
	TEST_REQUEST_BUF= (1 << 4),
	SETTING_REQUEST_BUF= (1 << 5),
	PUSH_MEDICINE_COMPLETE_REQUEST_BUF= (1 << 6),
	REPLENISH_MEDICINE_COMPLETE_REQUEST_BUF= (1 << 7),

	CMD_ACK_BUF = (1 << 16),
};  



enum {   
	CMD_NOK = 0,
    CMD_OK = 1,
};  


typedef enum{  
	FIRSTBOOT_STATUS = 0,
	STANDBY_STATUS = 1,
    PUSHING_STATUS = 2,
	REPLENISHING_STATUS = 3,
	TESTING_STATUS = 4,
	FAULT_STATUS = 0xFF,
}BOARD_STATUS_CODE;  


typedef enum{  
	TRACK_FAULT = 1,
	CONVEYOR_FAULT = 2,
	REFRIGERATION_FAULT = 3,
	OTHER_FAULT = 0x0F,
}FAULT_CODE;  

/*
typedef enum{
	MOTOR_FORWARD_TEST = 1,	//����ǰ��
	MOTOR_BACKWARD_TEST = 2,//��������
	PUSH_BELT_TEST = 3,		//�������ʹ�
	COLLECT_BELT_TEST = 4,	//���մ��ʹ�
	COMPRESSOR_TEST,		//ѹ����
	FAN_TEST,				//����
	FRONT_DOOR_TEST,		//ǰ����
	BACK_DOOR_TEST,			//�����
	DRUG_DOOR_TEST,			//ȡ����
	LIGHT_TEST,				//����
	CALIBRATE_TRACK_TEST,	//�����ֶ�У׼
}BOARD_TEST_MODE;  
*/
typedef enum{
	TRACK_TEST = 1, //����ǰ��
	PUSH_BELT_TEST = 2, 	//�������ʹ�
	COLLECT_BELT_TEST = 3,	//���մ��ʹ�
	COMPRESSOR_TEST,		//ѹ����
	FAN_TEST,				//����
	FRONT_DOOR_TEST,		//ǰ����
	BACK_DOOR_TEST, 		//�����
	DRUG_DOOR_TEST, 		//ȡ����
	LIGHT_TEST, 			//����
	CALIBRATE_TRACK_TEST,	//�����ֶ�У׼
	FACTORY_TEST = 0x80,
	TEST_MODE_MAX,
}BOARD_TEST_MODE;  



#define START_CODE 0x02

#define CMD_STATUS_REPORT_REQUEST 0x10	//������״̬�ϱ� 

#define CMD_PUSH_MEDICINE_REQUEST	0x20	//�������ָ��

#define CMD_REPLENISH_MEDICINE_REQUEST 0x30	//���󲹻�ָ��

#define CMD_CALIBRATE_TRACK_REQUEST 0x40	//����У׼ָ��

#define CMD_TEST_REQUEST 0x50	//���Զ���ָ��

#define CMD_SETTING_REQUEST 0x60	//���ò���ָ��

#define CMD_ADD_MEDICINE_COMPLETE 0x70	//�������

#define CMD_TRACK_RUNTIME_CALC 0x80	//��������ʱ��ͳ��

#define CMD_PUSH_MEDICINE_COMPLETE 0xA0	//�������

#define CMD_TRACK_RUNTIME_REPORT 0xB0	//��������ʱ���ϱ�

#define CMD_MCU_ADD_MEDICINE_COMPLETE 0xC0	//���岹�����

#define CMD_MSG_ACK 0xF0	//ָ��Ӧ��


typedef struct _uart_msg_struct
{
	uint8_t start_code; 
	uint8_t packet_len;
	uint8_t cmd_type;
    uint8_t board_id;  
	uint8_t tmp[16];	 
}uart_msg_struct;

typedef struct _ack_struct  
{  
	uint8_t start_code; 
	uint8_t packet_len;
	uint8_t cmd_type;
	uint8_t pl_cmd_type;
    uint8_t pl_board_id;
	uint8_t tmp[8];	 
}ack_struct; 



/*������״̬�ϱ�*/

struct status_report_request_info_struct  
{  
    uint8_t board_id;  
    uint8_t board_status;  
    uint8_t board_error_code;  
    uint8_t medicine_track_number;  
}; 
  
struct status_report_request_struct  
{  
	uint8_t start_code; 
	uint8_t packet_len;
	uint8_t cmd_type;
	struct status_report_request_info_struct info;
	uint8_t checksum; 
};  


/*�������ָ��*/
struct motor_control_info_struct  
{  
    uint8_t board_id;  
    uint8_t medicine_track_number;  
    uint16_t push_time;  	
}; 


struct push_medicine_request_struct  
{  
	uint8_t start_code; 
	uint8_t packet_len;
	uint8_t cmd_type;
	struct motor_control_info_struct info[1];
	uint8_t checksum; 
};  

struct push_medicine_paramter
{  
	uint8_t push_cnt;
	struct motor_control_info_struct info[1];
};  
#if 0
struct push_medicine__struct  
{  
	uint8_t start_code; 
	uint8_t packet_len;
	uint8_t cmd_type;
	uint8_t cmd_type;;
	uint8_t checksum; 
};
#endif



/*���󲹻�ָ��*/
struct replenish_medicine_request_info_struct  
{  
    uint8_t board_id;  
    uint8_t medicine_track_number;  
    uint8_t dirtion;  
    uint16_t push_time;  	
}; 

struct replenish_medicine_request_struct  
{  
	uint8_t start_code; 
	uint8_t packet_len;
	uint8_t cmd_type;
	struct replenish_medicine_request_info_struct info[1];
	uint8_t checksum; 
};  

struct replenish_medicine_paramter
{  
	uint8_t push_cnt;
	struct replenish_medicine_request_info_struct info[1];
};  




/*����У׼ָ��*/
struct calibrate_track_request_info_struct  
{  
    uint8_t board_id;  
    uint8_t medicine_track_number;  
}; 

struct calibrate_track_request_struct  
{  
	uint8_t start_code; 
	uint8_t packet_len;
	uint8_t cmd_type;
	struct calibrate_track_request_info_struct info;
	uint8_t checksum; 
};  



/*����������Ϣ */
struct test_request_info_struct  
{  
    uint8_t board_id;  
    uint8_t test_mode;  
    uint8_t test_status;  
    uint8_t medicine_track_number;  
    uint16_t test_time;  	
}; 


struct test_request_struct  
{  
	uint8_t start_code; 
	uint8_t packet_len;
	uint8_t cmd_type;
	struct test_request_info_struct info;
	uint8_t checksum; 
}; 
/*�������*/
struct push_medicine_complete_request_info_struct  
{  
    uint8_t board_id;  
    uint8_t medicine_track_number;  
    uint8_t track_status;  
}; 

struct push_medicine_complete_struct  
{  
	uint8_t start_code; 
	uint8_t packet_len;
	uint8_t cmd_type;
	struct push_medicine_complete_request_info_struct info;
	uint8_t checksum; 
}; 


/*��Ƭ���ڲ�ά���ĳ������*/
struct add_medicine_complete_request_info_struct  
{  
    uint8_t board_id;  
    uint8_t medicine_track_number;  
    uint8_t track_status;  
}; 

struct add_medicine_complete_struct  
{  
	uint8_t start_code; 
	uint8_t packet_len;
	uint8_t cmd_type;
	struct add_medicine_complete_request_info_struct info;
	uint8_t checksum; 
}; 





/*������� */
struct replenish_medicine_complete_request_info_struct  
{  
    uint8_t board_id;  	
}; 

struct replenish_medicine_complete_struct  
{  
	uint8_t start_code; 
	uint8_t packet_len;
	uint8_t cmd_type;
	struct replenish_medicine_complete_request_info_struct info;
	uint8_t checksum; 
}; 



/*��ѯ������Ϣ */

struct request_query_info_struct  
{  
    uint8_t board_id;  	
}; 



struct request_query_ack_info_struct  
{  
    uint8_t board_id;  
    uint8_t board_status;  
    uint8_t board_error_code;  
    uint16_t tempreture; 
    uint8_t medicine_track_number; 
}; 


struct request_query_struct  
{  
	uint8_t start_code; 
	uint8_t packet_len;
	uint8_t cmd_type;
	struct request_query_info_struct info;
	uint8_t checksum; 
}; 

struct request_query_ack_struct  
{  
	uint8_t start_code; 
	uint8_t packet_len;
	uint8_t cmd_type;
	struct request_query_ack_info_struct ack;
	uint8_t checksum; 		
};

/*��������ʱ��ͳ��ָ��*/

struct track_cale_request_info_struct  
{  
    uint8_t board_id;  
    uint8_t track_start_num;  
    uint8_t track_count;  	
}; 


struct track_calc_request_struct  
{  
	uint8_t start_code; 
	uint8_t packet_len;
	uint8_t cmd_type;
	struct track_cale_request_info_struct info;
	uint8_t checksum; 
}; 



/*��������ʱ���ϱ�ָ��*/
struct track_cale_report_info_struct  
{  
    uint8_t board_id;  
    uint8_t track_start_num;   
    uint16_t track_forward_time;  
    uint16_t track_backward_time;  
}; 


struct track_calc_report_struct  
{  
	uint8_t start_code; 
	uint8_t packet_len;
	uint8_t cmd_type;
	struct track_cale_report_info_struct info;
	uint8_t checksum; 
}; 

/*ָ��Ӧ��*/
struct msg_ack_info_struct  
{  
    uint8_t rsp_cmd_type;
    uint8_t board_id;  
    uint8_t status; 
}; 

struct msg_ack_struct  
{  
	uint8_t start_code; 
	uint8_t packet_len;
	uint8_t cmd_type;
	struct msg_ack_info_struct ack;
	uint8_t checksum; 		
};



struct motor_control_struct  
{  
	uint8_t motor_run;
	uint8_t motor_work_mode;
	struct motor_control_info_struct info;
};  

struct track_work_struct  
{  
	uint8_t motor_run;
    uint8_t medicine_track_number;  
    uint16_t push_time;  
};  


struct track_trigger_calc_runtime{
    uint8_t track_num;
	uint8_t track_forward_runtime;//ǰ��ʱ��
    uint8_t track_backward_runtime;//����ʱ��
};

extern uint8_t track_work;

extern int motor_enqueue_idx;

extern int motor_dequeue_idx;

extern int enqueue_replenish_index;

extern int dequeue_replenish_index;

extern uint16_t calibrate_track_selected;

extern unsigned char calibrate_enable;

extern uint16_t calibrate_track_selected;

extern struct motor_control_struct  motor_struct[TOTAL_PUSH_CNT];

extern struct track_work_struct track_struct[10][10];


extern uint16_t calibrate_track_selected;








void parse_up_rx_info(void); 

void send_status_report_request(void *input_data); 
void push_test(void);
void replenish_test(void);
void test_test(void);
void calibrate_test(void);
void replenish_complete_test(void);



void BoardId_Init(void);
void packet_parser(unsigned char *src, int len);
void up_packet_parser(unsigned char *src, int len);
uint8_t up_packet_preparser(unsigned char *src, int len);

uint8_t IsACKMsg(unsigned char *src, int len);


int board_send_message(int msg_type, void *input_data);
void mcu_push_medicine_open_door_complete(void);
void mcu_push_medicine_close_door_complete(void);
void mcu_push_medicine_track_only(uint8_t board, uint8_t track_number);
void mcu_add_medicine_track_only(uint8_t board, uint8_t track_number);

void send_track_runtime_report( void *input_data);
uint16_t GetMaxPushTime(void);

void SetTrackTestTime(uint8_t track, uint8_t dir, uint16_t time);


#endif
