#ifndef _SENSOR_H_
#define _SENSOR_H_


typedef enum
{
	SENSOR_NO_DETECT = 0,	//å¼€
	SENSOR_DETECT,		//å…³
}SENSOR_DETECT_ENUM;

typedef enum
{
	DOOR_OPEN = 0,	//¿ª
	DOOR_CLOSE,		//¹Ø
}Door_Detect_ENUM;


void Sensor_Init(void);

_Bool Sensor_Status(void);

unsigned char Sensor_Detect(void);

void Door_Key_Init(void);

unsigned char Door_Key_Detect(unsigned char door_detect);

#endif
