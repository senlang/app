#ifndef _COOLING_H_
#define _COOLING_H_


#define COOLING_ON		1
#define COOLING_OFF	0

void Cooling_Init(void);

void Coolingcompressor_Set(_Bool status);

void Coolingfan_Set(_Bool status);

#endif
