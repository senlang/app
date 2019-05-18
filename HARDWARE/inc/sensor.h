#ifndef _SENSOR_H_
#define _SENSOR_H_


typedef enum
{
	SENSOR_NO_DETECT = 0,	//开
	SENSOR_DETECT,		//关
}SENSOR_DETECT_ENUM;

typedef enum
{
	DOOR_OPEN = 0,	//��
	DOOR_CLOSE,		//��
}Door_Detect_ENUM;


void Sensor_Init(void);

_Bool Sensor_Status(void);

unsigned char Sensor_Detect(void);

void Door_Key_Init(void);

unsigned char Door_Key_Detect(unsigned char door_detect);

#endif
