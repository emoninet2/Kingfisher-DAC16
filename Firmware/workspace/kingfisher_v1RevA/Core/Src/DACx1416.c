/*
 * DACx1416.c
 *
 *  Created on: Oct 18, 2024
 *      Author: habiburrahman
 */
#include "main.h"
#include "DACx1416.h"

extern void SPI1_TransmitReceive(uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size);



extern SPI_HandleTypeDef hspi1;




void DACx1416_SPI_transmit(uint8_t *txBuffer,uint16_t size){



	// Full-duplex transmission and reception (blocking mode)
	if (HAL_SPI_Transmit(&hspi1, txBuffer, size, HAL_MAX_DELAY) != HAL_OK)
	{
		// Communication error
		Error_Handler();
	}



}


void DACx1416_SPI_receive(uint8_t *rxBuffer,uint16_t size){


	// Full-duplex transmission and reception (blocking mode)
	if (HAL_SPI_Receive(&hspi1, rxBuffer, size, HAL_MAX_DELAY) != HAL_OK)
	{
		// Communication error
		Error_Handler();
	}

}


void DACx1416_SPI_transmitReceive(uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size){


    // Full-duplex transmission and reception (blocking mode)
    if (HAL_SPI_TransmitReceive(&hspi1, txBuffer, rxBuffer, size, HAL_MAX_DELAY) != HAL_OK)
    {
        // Communication error
        Error_Handler();
    }

}




void DACx1416_write_register(uint8_t address, uint16_t data){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Set CS low (adjust GPIO pin as needed)
	uint8_t txBuffer[3] = {address & ~(1<<7) , (data >> 8) & 0xFF, data & 0xFF};
	uint8_t rxBuffer[3];
	SPI1_TransmitReceive(txBuffer, rxBuffer,  3);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // Set CS high (adjust GPIO pin as needed)
}

uint16_t DACx1416_read_register(uint8_t address){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Set CS low (adjust GPIO pin as needed)
	uint8_t txBuffer[3] = {address | (1<<7) , 0xFF, 0xFF};
	uint8_t rxBuffer[3];
	DACx1416_SPI_transmit(txBuffer, 3);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // Set CS high (adjust GPIO pin as needed)

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Set CS low (adjust GPIO pin as needed)
	DACx1416_SPI_receive(rxBuffer, 3);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // Set CS high (adjust GPIO pin as needed)


	return rxBuffer[1]<<8 | rxBuffer[2];

}
