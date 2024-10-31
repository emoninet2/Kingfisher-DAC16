/*
 * cmdParser.c
 *
 *  Created on: Oct 29, 2024
 *      Author: habiburrahman
 */

#include "main.h"
#include "cmdParser.h"
#include "DACx1416.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

extern DACx1416_HandleTypeDef dac;
extern SPI_HandleTypeDef hspi1;





void parseExtendedCmd(uint8_t *data, uint32_t len){
	uint8_t extCmd = data[0] & 0x3F;

	switch(extCmd){
	case EXTCMD_CONT_SPI_CRC_ON: {
		  hspi1.Instance = SPI1;
		  hspi1.Init.Mode = SPI_MODE_MASTER;
		  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
		  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
		  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
		  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
		  hspi1.Init.NSS = SPI_NSS_SOFT;
		  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
		  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
		  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
		  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
		  hspi1.Init.CRCPolynomial = 7;
		  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
		  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
		  if (HAL_SPI_Init(&hspi1) != HAL_OK)
		  {
		    Error_Handler();
		  }
		break;
	}
	case EXTCMD_CONT_SPI_CRC_OFF: {
		  hspi1.Instance = SPI1;
		  hspi1.Init.Mode = SPI_MODE_MASTER;
		  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
		  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
		  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
		  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
		  hspi1.Init.NSS = SPI_NSS_SOFT;
		  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
		  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
		  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
		  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		  hspi1.Init.CRCPolynomial = 7;
		  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
		  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
		  if (HAL_SPI_Init(&hspi1) != HAL_OK)
		  {
		    Error_Handler();
		  }

		break;
	}
	default:
		// Handle unexpected commands, if necessary
		break;
	}


}




void parseCmd(uint8_t *data, uint32_t len) {

	uint8_t cmd = (data[0]>>6) & 0x3;

	switch(cmd){
	case CMD_READ_DAC_REGISTER: {//read register

		uint8_t address = data[0] & 0x3F;
		uint16_t regVal = DACx1416_read_register(&dac,address);
		uint8_t response[2];
		response[0] = regVal >> 8;
		response[1] = regVal & 0xFF;
		if (CDC_Transmit_FS(response, 2) != USBD_OK) {
					Error_Handler();
		}
		break;
	}
	case CMD_WRITE_DAC_REGISTER: { //write register

		uint8_t address = data[0] & 0x3F;
		uint16_t value = data[1]<<8 | data[2];
		DACx1416_write_register(&dac, address, value);



		break;
	}
	case CMD_WRITE_STREAM_DAC_REGISTERS: { //stream write register


		break;
	}

	case CMD_EXTENDED_COMMANDS: { //parse extended commands
		parseExtendedCmd(data,  len);
		break;
	}
	default:
		// Handle unexpected commands, if necessary
		break;
	}


}




