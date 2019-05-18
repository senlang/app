#ifndef _DOOR_H_
#define _DOOR_H_

#define BOX_DOOR_OPEN		1
#define BOX_DOOR_CLOSE	0


void Door_Init(void);

void FrontDoor_Set(_Bool status);

void BackDoor_Set(_Bool status);

#endif
