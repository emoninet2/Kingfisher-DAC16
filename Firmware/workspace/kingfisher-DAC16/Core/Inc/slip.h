/*
 * slip.h
 *
 *  Created on: Nov 21, 2024
 *      Author: habiburrahman
 */

#ifndef INC_SLIP_H_
#define INC_SLIP_H_

#include "main.h"


#define SLIP_END 		0xC0
#define SLIP_ESC 		0xDB
#define SLIP_ESC_END 	0xDC
#define SLIP_ESC_ESC 	0xDD



typedef struct {
    // Function pointers
	char (*recv_char)();
	void (*send_char)(char ch);

} SLIP_HandleTypeDef;




void slip_send_packet(SLIP_HandleTypeDef *slip, uint8_t * p, uint32_t len);
uint32_t slip_recv_packet(SLIP_HandleTypeDef *slip, uint8_t * p, uint32_t len);


#endif /* INC_SLIP_H_ */
