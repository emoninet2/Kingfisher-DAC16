/*
 * cmdParser.c
 *
 *  Created on: Oct 29, 2024
 *      Author: habiburrahman
 */

#include "main.h"
#include "cmdParser.h"
#include "DACx1416.h"
#include "slip.h"

extern DACx1416 dacUnit;

static void parseExtendedCmd(SLIP_HandleTypeDef *slip, uint8_t *data, uint32_t len)
{
	if (len < 1U) {
		return;
	}

	uint8_t extCmd = data[0] & 0x3F;

	switch (extCmd) {
	case EXTCMD_HARDWARE_RESET: {
		/* Active-low nRESET: assert low, hold, release (see DACx1416 timing). */
		dacUnit.port.nCS(1);
		dacUnit.port.nRESET(0);
		HAL_Delay(2);
		dacUnit.port.nRESET(1);
		HAL_Delay(1);
		uint8_t ack = 0x00;
		slip_send_packet(slip, &ack, 1U);
		break;
	}
	default:
		break;
	}
}

void parseCmd(SLIP_HandleTypeDef *slip, uint8_t *data, uint32_t len)
{
	uint8_t cmd = (data[0] >> 6) & 0x3U;

	switch (cmd) {
	case 0b00: { /* write register */
		uint8_t rxBuffer[4];
		if (len == 3U) {
			dacUnit.port.nCS(0);
			dacUnit.port.SPI_transmitReceive(data, rxBuffer, 3);
			dacUnit.port.nCS(1);
			slip_send_packet(slip, rxBuffer, 3);
		} else if (len == 4U) {
			dacUnit.port.nCS(0);
			dacUnit.port.SPI_transmitReceive(data, rxBuffer, 4);
			dacUnit.port.nCS(1);
			slip_send_packet(slip, rxBuffer, 4);
		}
		break;
	}

	case 0b10: { /* read register */
		uint8_t address = data[0] & 0x3F;
		uint8_t txBuffer[4] = { address | (1 << 7), 0xFF, 0xFF, 0 };
		uint8_t rxBuffer[4];

		if (len == 1U) {
			dacUnit.port.nCS(0);
			dacUnit.port.SPI_transmitReceive(txBuffer, rxBuffer, 3);
			dacUnit.port.nCS(1);
			dacUnit.port.nCS(0);
			dacUnit.port.SPI_receive(rxBuffer, 3);
			dacUnit.port.nCS(1);
			slip_send_packet(slip, rxBuffer, 3);
		} else if (len == 2U) {
			txBuffer[0] = address | (1 << 7);
			txBuffer[1] = 0xFF;
			txBuffer[2] = 0xFF;
			txBuffer[3] = data[1];
			dacUnit.port.nCS(0);
			dacUnit.port.SPI_transmitReceive(txBuffer, rxBuffer, 4);
			dacUnit.port.nCS(1);
			uint8_t txSecond[4] = { 0x00, 0x00, 0x00, 0x00 };
			txSecond[3] = dacUnit.port.calculate_crc8(txSecond, 3);
			dacUnit.port.nCS(0);
			dacUnit.port.SPI_transmitReceive(txSecond, rxBuffer, 4);
			dacUnit.port.nCS(1);
			slip_send_packet(slip, rxBuffer, 4);
		}
		break;
	}

	case CMD_EXTENDED_COMMANDS:
		parseExtendedCmd(slip, data, len);
		break;

	default:
		break;
	}
}
