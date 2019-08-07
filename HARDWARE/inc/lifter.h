#ifndef _LIFTER_H_
#define _LIFTER_H_

typedef enum
{
	LIFTER_KEY_NOT_DETECT = 0,	//Éý
	LIFTER_KEY_DETECT,		//½µ
}Lifter_Detect_ENUM;


#define LIFTER_UP		0
#define LIFTER_FALL		1



void Lifter_Init(void);

void Lifter_Set(uint8_t status);



#endif
