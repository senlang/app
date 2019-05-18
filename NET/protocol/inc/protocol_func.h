#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include "stm32f10x.h"

unsigned char add_checksum (unsigned char *buf, unsigned int len);  

void parse_board_test_request(uint8_t *outputdata, uint8_t *inputdata); 
void print_board_test_request(uint8_t *data);


#endif
