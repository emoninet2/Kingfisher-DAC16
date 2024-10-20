/*
 * DACx1416.h
 *
 *  Created on: Oct 18, 2024
 *      Author: habiburrahman
 */

#ifndef INC_DACX1416_H_
#define INC_DACX1416_H_


void DACx1416_SPI_transmit(uint8_t *txBuffer,uint16_t size);
void DACx1416_SPI_receive(uint8_t *rxBuffer,uint16_t size);
void DACx1416_SPI_transmitReceive(uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size);


void DACx1416_write_register(uint8_t address, uint16_t data);
uint16_t DACx1416_read_register(uint8_t address);






#endif /* INC_DACX1416_H_ */
