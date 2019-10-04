#ifndef _DOOR_H_
#define _DOOR_H_

#define BOX_DOOR_OPEN		1
#define BOX_DOOR_CLOSE	0


void Door_Init(void);

void FrontRightDoor_Set(_Bool status);

void BackRightDoor_Set(_Bool status);

void FrontLeftDoor_Set(_Bool status);

void BackLeftDoor_Set(_Bool status);


#endif
