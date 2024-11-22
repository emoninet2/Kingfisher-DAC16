/*
 * DACx1416_port.h
 *
 *  Created on: Oct 31, 2024
 *      Author: habiburrahman
 */

#ifndef INC_DACX1416_PORT_H_
#define INC_DACX1416_PORT_H_

#include "main.h"


void DACx1416_nCS(uint8_t value);
void DACx1416_nLDAC(uint8_t value);
void DACx1416_nRESET(uint8_t value);
void DACx1416_nCLR(uint8_t value);
void DACx1416_tgl(uint8_t value);
void DACx1416_SPI_transmit(uint8_t *txBuffer,uint16_t size);
void DACx1416_SPI_receive(uint8_t *rxBuffer,uint16_t size);
void DACx1416_SPI_transmitReceive(uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size);
uint8_t DACx1416_calculate_crc8(uint8_t *data, uint32_t length);
#endif /* INC_DACX1416_PORT_H_ */
