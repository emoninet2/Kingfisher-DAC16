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
#include "slip.h"



extern  DACx1416 dacUnit;
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




void parseCmd(SLIP_HandleTypeDef *slip, uint8_t *data, uint32_t len) {

	uint8_t cmd = (data[0]>>6) & 0x3;

	switch(cmd){
	case 0b00: { //write register

		uint8_t rxBuffer[4];
		if(len == 3){
			dacUnit.port.nCS(0);
			dacUnit.port.SPI_transmitReceive(data, rxBuffer, 3);
			dacUnit.port.nCS(1);
//			dacUnit.port.nCS(0);
//			dacUnit.port.SPI_receive(rxBuffer, 3);
//			dacUnit.port.nCS(1);
			//slip_send_packet(slip, rxBuffer, 3);
		}
		else if(len == 4){
			dacUnit.port.nCS(0);
			dacUnit.port.SPI_transmitReceive(data, rxBuffer, 4);
			dacUnit.port.nCS(1);
//			dacUnit.port.nCS(0);
//			dacUnit.port.SPI_receive(rxBuffer, 4);
//			dacUnit.port.nCS(1);
			//slip_send_packet(slip, rxBuffer, 4);
		}



		break;
	}

	case 0b10: {//read register

		uint8_t address = data[0] & 0x3F;
		uint16_t regVal;

		uint8_t txBuffer[4] = { address | (1 << 7), 0xFF, 0xFF , 0};
		uint8_t rxBuffer[4];

		if(len == 1){

			dacUnit.port.nCS(0);
			dacUnit.port.SPI_transmitReceive(txBuffer, rxBuffer, 3);
			dacUnit.port.nCS(1);
			dacUnit.port.nCS(0);
			dacUnit.port.SPI_receive(rxBuffer, 3);
			dacUnit.port.nCS(1);
			slip_send_packet(slip, rxBuffer, 3);

		}
		else if(len == 2){

			//uint8_t crc8 = dacUnit.port.calculate_crc8(txBuffer, 3);
			txBuffer[0] = address | (1 << 7);
			txBuffer[1] = 0xFF;
			txBuffer[2] = 0xFF;
			txBuffer[3] = data[1];
			dacUnit.port.nCS(0);
			dacUnit.port.SPI_transmitReceive(txBuffer, rxBuffer, 4);
			dacUnit.port.nCS(1);
			dacUnit.port.nCS(0);
			dacUnit.port.SPI_receive(rxBuffer, 4);
			dacUnit.port.nCS(1);
			slip_send_packet(slip, rxBuffer, 4);

		}

		//DACx1416_read_register(&dacUnit,address, &regVal);




		break;
	}


	case CMD_EXTENDED_COMMANDS: { //parse extended commands
		//parseExtendedCmd(data,  len);
		break;
	}
	default:
		// Handle unexpected commands, if necessary
		break;
	}


}




