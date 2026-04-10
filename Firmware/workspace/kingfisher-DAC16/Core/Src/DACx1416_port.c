/*
 * DACx1416_port.c
 *
 *  Created on: Oct 31, 2024
 *      Author: habiburrahman
 */

#include "DACx1416_port.h"
#include "main.h"
#include "string.h"

extern SPI_HandleTypeDef hspi1;

/*
 * TI DACx1416 / DAC81416 SPI CRC-8-ATM (HEC), polynomial x^8+x^2+x+1 (0x07).
 * Must match host Python driver and device; STM32 HAL CRC is not equivalent.
 */
uint8_t DACx1416_calculate_crc8(uint8_t *data, uint32_t length) {
	uint8_t crc = 0;
	for (uint32_t i = 0; i < length; i++) {
		crc ^= data[i];
		for (int j = 0; j < 8; j++) {
			if (crc & 0x80U) {
				crc = (uint8_t)((crc << 1) ^ 0x07U);
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}


void DACx1416_SPI_transmit(uint8_t *txBuffer, uint16_t size) {
    uint8_t rxBuffer[size];  // Dummy receive buffer

    // Transmit data and simultaneously receive data (full-duplex)
    if (HAL_SPI_TransmitReceive(&hspi1, txBuffer, rxBuffer, size, 1000) != HAL_OK) {
        // Communication error
        // Error_Handler();
    }
}


void DACx1416_SPI_receive(uint8_t *rxBuffer,uint16_t size){

	uint8_t txBuffer[size];  // Buffer with dummy data
	memset(txBuffer, 0x00, size);  // Fill with dummy bytes (0x00 or 0xFF)

	// Full-duplex transmission and reception (blocking mode)
	if (HAL_SPI_TransmitReceive(&hspi1, txBuffer, rxBuffer, size, 1000) != HAL_OK)
	{
		// Communication error
		//Error_Handler();
	}

}


void DACx1416_SPI_transmitReceive(uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size){


    // Full-duplex transmission and reception (blocking mode)
    if (HAL_SPI_TransmitReceive(&hspi1, txBuffer, rxBuffer, size, 1000) != HAL_OK)
    {
        // Communication error
        //Error_Handler();
    }

}

void DACx1416_nCS(uint8_t value){
	if(value){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // Set CS high
	}
	else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Set CS low
	}
}

void DACx1416_nLDAC(uint8_t value){
	if(value){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // Set CS high
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // Set CS low
	}
}
void DACx1416_nRESET(uint8_t value){
	if(value){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // Set CS high
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // Set CS low
	}
}
void DACx1416_nCLR(uint8_t value){
	if(value){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);   // Set CS high
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // Set CS low
	}
}

void DACx1416_tgl(uint8_t value){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, (value>>2) & 0x01);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (value>>1) & 0x01);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (value) & 0x01);
}
