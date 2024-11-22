/*
 * DACx1416.c
 *
 *  Created on: Oct 31, 2024
 *      Author: habiburrahman
 */

#include "DACx1416.h"
#include "main.h"
#include "stdlib.h"

DACx1416_error_t DACx1416_initialize(DACx1416 *dac) {


	dac->port.nCS(0);
	dac->port.nCLR(0);
	dac->port.nLDAC(0);
	dac->port.nRESET(0);

	dac->port.nCS(1);
	dac->port.nCLR(1);
	dac->port.nLDAC(1);
	dac->port.nRESET(1);




//	DACx1416_get_spiConfig(dac, &dac->spiConfig);
//
//
//	DACx1416_get_status(dac, &dac->status);
//	DACx1416_get_device_id(dac, &dac->deviceID);
//	DACx1416_get_genConfig(dac, &dac->genConfig);
//	DACx1416_get_brdConfig(dac, &dac->brdcstConfig);
//	DACx1416_get_syncConfig(dac, &dac->syncConfig);
//	DACx1416_get_toggConfig(dac, &dac->toggConfig);
//	DACx1416_get_pwdwn(dac, &dac->chPwrDwn);
//	DACx1416_get_dacRange(dac, &dac->dacRange);

	return DACx1416_OK;
}

DACx1416_error_t DACx1416_write_register(DACx1416 *dac,
		DACx1416_register_t addr, uint16_t *data) {
	if(!dac->spiConfig.crc_en){
		uint8_t txBuffer[3] = { addr & ~(1 << 7), (*data >> 8) & 0xFF, *data
				& 0xFF };
		uint8_t rxBuffer[3];
		dac->port.nCS(0);
		dac->port.SPI_transmitReceive(txBuffer, rxBuffer, 3);
		dac->port.nCS(1);
	} else {
		uint8_t txBuffer[4] = { addr & ~(1 << 7), (*data >> 8) & 0xFF, *data
				& 0xFF, 0 };
		uint8_t rxBuffer[4];
		uint8_t crc8 = dac->port.calculate_crc8(txBuffer, 3);
		txBuffer[3] = crc8;
		dac->port.nCS(0);
		dac->port.SPI_transmitReceive(txBuffer, rxBuffer, 4);
		dac->port.nCS(1);





	}
	return DACx1416_OK;
}

DACx1416_error_t DACx1416_stream_write_registers(DACx1416 *dac,
		DACx1416_register_t addr, uint16_t *data, uint32_t len) {
	if(!dac->spiConfig.str_en){
		if(!dac->spiConfig.crc_en){
			// Calculate the buffer length once
			const uint32_t totalLen = 1 + 2 * len;

			// Using a dynamically allocated buffer to reduce stack usage for large lengths
			uint8_t *txBuffer = (uint8_t*) malloc(totalLen * sizeof(uint8_t));
			if (!txBuffer) {
				return DACx1416_ERROR; // Assuming an error type for memory allocation failure
			}

			// Initialize the command address (write mode, setting MSB of `addr` to 0)
			txBuffer[0] = addr & ~(1 << 7);

			// Populate the buffer with data in a streamlined way
			for (uint32_t i = 0; i < len; ++i) {
				uint16_t value = data[i];
				txBuffer[2 * i + 1] = (uint8_t) (value >> 8);  // High byte
				txBuffer[2 * i + 2] = (uint8_t) value;         // Low byte
			}

			// Transmit data over SPI
			dac->port.nCS(0);  // Assert chip select
			dac->port.SPI_transmit(txBuffer, totalLen); // Send the entire buffer
			dac->port.nCS(1);  // Deassert chip select

			// Free allocated memory
			free(txBuffer);
		} else {
			//TO BE WRITTEN for STREAM WRITE WITH CRC
		}
	}

	return DACx1416_OK;
}

DACx1416_error_t DACx1416_read_register(DACx1416 *dac, DACx1416_register_t addr,
		uint16_t *data) {
	if(!dac->spiConfig.crc_en){
		uint8_t txBuffer[3] = { addr | (1 << 7), 0xFF, 0xFF };
		uint8_t rxBuffer[3];
		dac->port.nCS(0);
		dac->port.SPI_transmitReceive(txBuffer, rxBuffer, 3);
		dac->port.nCS(1);
		dac->port.nCS(0);
		dac->port.SPI_receive(rxBuffer, 3);
		dac->port.nCS(1);
		*data = rxBuffer[1] << 8 | rxBuffer[2];
		return DACx1416_OK;
	} else {
		uint8_t txBuffer[4] = { addr | (1 << 7), 0xFF, 0xFF, 0 };
		uint8_t rxBuffer[4];
		uint8_t crc8 = dac->port.calculate_crc8(txBuffer, 3);
		txBuffer[3] = crc8;
		dac->port.nCS(0);
		dac->port.SPI_transmitReceive(txBuffer, rxBuffer, 4);
		dac->port.nCS(1);
		dac->port.nCS(0);
		dac->port.SPI_receive(rxBuffer, 4);
		dac->port.nCS(1);
		crc8 = dac->port.calculate_crc8(rxBuffer, 3);

		if (crc8 == rxBuffer[3]) {
			*data = rxBuffer[1] << 8 | rxBuffer[2];
			return DACx1416_OK;
		} else {
			return DACx1416_ERROR;
		}
	}
}

DACx1416_error_t DACx1416_get_device_id(DACx1416 *dac,
		DACx1416_deviceID_t *deviceID) {
	uint16_t deviceIdRegValue = 0;
	DACx1416_error_t error = DACx1416_read_register(dac, DACx1416_REG_DEVICEID,
			&deviceIdRegValue);

	if (error != DACx1416_OK) {
		return error;  // Return the specific error from DACx1416_read_register
	}

	deviceID->deviceId = (deviceIdRegValue >> DACx1416_DEVICE_ID_GP)
			& DACx1416_DEVICE_ID_MASK;
	deviceID->versionId = deviceIdRegValue & DACx1416_VERSION_ID_MASK;

	return DACx1416_OK;
}

DACx1416_error_t DACx1416_get_status(DACx1416 *dac, DACx1416_status_t *status) {
	uint16_t statusRegVal = 0;
	DACx1416_error_t error = DACx1416_read_register(dac, DACx1416_REG_STATUS,
			&statusRegVal);

	if (error != DACx1416_OK) {
		return error;  // Return the specific error from DACx1416_read_register
	}

	status->crc_alarm = (statusRegVal >> DACx1416_STATUS_CRC_ALARM_BP) & 0x01;
	status->dac_busy = (statusRegVal >> DACx1416_STATUS_DAC_BUSY_BP) & 0x01;
	status->temp_alarm = statusRegVal & 0x01;
	return DACx1416_OK;
}

DACx1416_error_t DACx1416_get_spiConfig(DACx1416 *dac,
		DACx1416_spiconfig_t *spiConfig) {
	uint16_t spiConfigRegVal = 0;
	DACx1416_error_t error = DACx1416_read_register(dac, DACx1416_REG_SPICONFIG,
			&spiConfigRegVal);

	if (error != DACx1416_OK) {
		return error;  // Return the specific error from DACx1416_read_register
	}

	spiConfig->tempalm_en =
			(spiConfigRegVal >> DACx1416_SPICONFIG_TEMPALM_EN_BP) & 0x01;
	spiConfig->dacbusy_en =
			(spiConfigRegVal >> DACx1416_SPICONFIG_DACBUSY_EN_BP) & 0x01;
	spiConfig->crcalm_en = (spiConfigRegVal >> DACx1416_SPICONFIG_CRCALM_EN_BP)
			& 0x01;
	spiConfig->softtoggle_en = (spiConfigRegVal
			>> DACx1416_SPICONFIG_SOFTTOGGL_EN_BP) & 0x01;
	spiConfig->dev_pwdwn = (spiConfigRegVal >> DACx1416_SPICONFIG_DEV_PWDWN_BP)
			& 0x01;
	spiConfig->crc_en = (spiConfigRegVal >> DACx1416_SPICONFIG_CRC_EN_BP)
			& 0x01;
	spiConfig->str_en = (spiConfigRegVal >> DACx1416_SPICONFIG_STRM_EN_BP)
			& 0x01;
	spiConfig->sdo_en = (spiConfigRegVal >> DACx1416_SPICONFIG_SDO_EN_BP)
			& 0x01;
	spiConfig->fsdo = (spiConfigRegVal >> DACx1416_SPICONFIG_FSDO_EN_BP) & 0x01;
	return DACx1416_OK;
}
DACx1416_error_t DACx1416_get_genConfig(DACx1416 *dac,
		DACx1416_genconfig_t *genConfig) {
	uint16_t genConfigRegVal = 0;
	DACx1416_error_t error = DACx1416_read_register(dac, DACx1416_REG_GENCONFIG,
			&genConfigRegVal);

	if (error != DACx1416_OK) {
		return error;  // Return the specific error from DACx1416_read_register
	}

	genConfig->ref_pwdwn = (genConfigRegVal >> DACx1416_GENCONFIG_REF_PWDN_BP)
			& 0x01;
	genConfig->DAC_14_15_diff_en = (genConfigRegVal
			>> DACx1416_GENCONFIG_DAC_14_15_DIFF_EN_BP) & 0x01;
	genConfig->DAC_12_13_diff_en = (genConfigRegVal
			>> DACx1416_GENCONFIG_DAC_12_13_DIFF_EN_BP) & 0x01;
	genConfig->DAC_10_11_diff_en = (genConfigRegVal
			>> DACx1416_GENCONFIG_DAC_10_11_DIFF_EN_BP) & 0x01;
	genConfig->DAC_8_9_diff_en = (genConfigRegVal
			>> DACx1416_GENCONFIG_DAC_8_9_DIFF_EN_BP) & 0x01;
	genConfig->DAC_6_7_diff_en = (genConfigRegVal
			>> DACx1416_GENCONFIG_DAC_6_7_DIFF_EN_BP) & 0x01;
	genConfig->DAC_4_5_diff_en = (genConfigRegVal
			>> DACx1416_GENCONFIG_DAC_4_5_DIFF_EN_BP) & 0x01;
	genConfig->DAC_2_3_diff_en = (genConfigRegVal
			>> DACx1416_GENCONFIG_DAC_2_3_DIFF_EN_BP) & 0x01;
	genConfig->DAC_0_1_diff_en = (genConfigRegVal
			>> DACx1416_GENCONFIG_DAC_0_1_DIFF_EN_BP) & 0x01;
	return DACx1416_OK;
}
DACx1416_error_t DACx1416_get_brdConfig(DACx1416 *dac,
		DACx1416_brdConfig *brdConfig) {
	uint16_t brdcstConfigRegVal = 0;
	DACx1416_error_t error = DACx1416_read_register(dac, DACx1416_REG_BRDCONFIG,
			&brdcstConfigRegVal);

	if (error != DACx1416_OK) {
		return error;  // Return the specific error from DACx1416_read_register
	}

	for (int i = 0; i < 16; i++) {
		brdConfig->brdcast_en[i] = (brdcstConfigRegVal >> i) & 0x01;
	}

	return DACx1416_OK;
}
DACx1416_error_t DACx1416_get_syncConfig(DACx1416 *dac,
		DACx1416_syncConfig *syncConfig) {
	uint16_t syncConfigRegVal = 0;
	DACx1416_error_t error = DACx1416_read_register(dac,
			DACx1416_REG_SYNCCONFIG, &syncConfigRegVal);

	if (error != DACx1416_OK) {
		return error;  // Return the specific error from DACx1416_read_register
	}

	for (int i = 0; i < 16; i++) {
		syncConfig->sync_en[i] = (syncConfigRegVal >> i) & 0x01;
	}

	return DACx1416_OK;
}
DACx1416_error_t DACx1416_get_toggConfig(DACx1416 *dac,
		DACx1416_toggConfig *toggConfig) {
	uint16_t toggConfigRegVal[2];

	DACx1416_error_t error = DACx1416_read_register(dac,
			DACx1416_REG_TOGGCONFIG0, &toggConfigRegVal[0]);
	error |= DACx1416_read_register(dac, DACx1416_REG_TOGGCONFIG1,
			&toggConfigRegVal[1]);

	if (error != DACx1416_OK) {
		return error;  // Return the specific error from DACx1416_read_register
	}

	// Loop for the first 8 toggles in regVal0
	for (int i = 0; i < 8; i++) {
		toggConfig->toggle_en[15 - i] = (toggConfigRegVal[0] >> (14 - 2 * i))
				& 0x03;
	}

	// Loop for the next 8 toggles in regVal1
	for (int i = 0; i < 8; i++) {
		toggConfig->toggle_en[7 - i] = (toggConfigRegVal[1] >> (14 - 2 * i))
				& 0x03;
	}

	return DACx1416_OK;

}
DACx1416_error_t DACx1416_get_pwdwn(DACx1416 *dac, DACx1416_dacPwdwn *dacPwdwn) {
	uint16_t dacPwrdwnRegVal = 0;
	DACx1416_error_t error = DACx1416_read_register(dac, DACx1416_REG_DACPWDWN,
			&dacPwrdwnRegVal);

	if (error != DACx1416_OK) {
		return error;  // Return the specific error from DACx1416_read_register
	}

	for (int i = 0; i < 16; i++) {
		dacPwdwn->dac_pwdwn[i] = (dacPwrdwnRegVal >> i) & 0x01;
	}

	return DACx1416_OK;
}
DACx1416_error_t DACx1416_get_dacRange(DACx1416 *dac,
		DACx1416_dacRange *dacRange) {
	uint16_t dacRangeRegVals[4];

	DACx1416_error_t error = DACx1416_OK;
	// Read all four registers
	error |= DACx1416_read_register(dac, DACx1416_REG_DACRANGE0,
			&dacRangeRegVals[0]);
	error |= DACx1416_read_register(dac, DACx1416_REG_DACRANGE1,
			&dacRangeRegVals[1]);
	error |= DACx1416_read_register(dac, DACx1416_REG_DACRANGE2,
			&dacRangeRegVals[2]);
	error |= DACx1416_read_register(dac, DACx1416_REG_DACRANGE3,
			&dacRangeRegVals[3]);

	if (error != DACx1416_OK) {
		return error;  // Return the specific error from DACx1416_read_register
	}

	// Extract each 4-bit range value
	for (int i = 0; i < 16; i++) {
		dacRange->dacRange[15 - i] = (dacRangeRegVals[i / 4]
				>> (12 - 4 * (i % 4))) & 0x0F;
	}

	return DACx1416_OK;
}

DACx1416_error_t DACx1416_set_spiConfig(DACx1416 *dac,
		DACx1416_spiconfig_t *spiConfig) {
	uint16_t spiConfigRegVal = (spiConfig->tempalm_en
			<< DACx1416_SPICONFIG_TEMPALM_EN_BP)
			| (spiConfig->dacbusy_en << DACx1416_SPICONFIG_DACBUSY_EN_BP)
			| (spiConfig->crcalm_en << DACx1416_SPICONFIG_CRCALM_EN_BP)
			| (spiConfig->softtoggle_en << DACx1416_SPICONFIG_SOFTTOGGL_EN_BP)
			| (spiConfig->dev_pwdwn << DACx1416_SPICONFIG_DEV_PWDWN_BP)
			| (spiConfig->crc_en << DACx1416_SPICONFIG_CRC_EN_BP)
			| (spiConfig->str_en << DACx1416_SPICONFIG_STRM_EN_BP)
			| (spiConfig->sdo_en << DACx1416_SPICONFIG_SDO_EN_BP)
			| (spiConfig->fsdo << DACx1416_SPICONFIG_FSDO_EN_BP);

	if (DACx1416_write_register(dac, DACx1416_REG_SPICONFIG, &spiConfigRegVal)
			!= DACx1416_OK) {
		return DACx1416_ERROR; // Return the specific error from DACx1416_read_register
	}
	return DACx1416_OK;

}
DACx1416_error_t DACx1416_set_genConfig(DACx1416 *dac,
		DACx1416_genconfig_t *genConfig) {
	uint16_t genConfigRegVal = (genConfig->ref_pwdwn
			<< DACx1416_GENCONFIG_REF_PWDN_BP)
			| (genConfig->DAC_14_15_diff_en
					<< DACx1416_GENCONFIG_DAC_14_15_DIFF_EN_BP)
			| (genConfig->DAC_12_13_diff_en
					<< DACx1416_GENCONFIG_DAC_12_13_DIFF_EN_BP)
			| (genConfig->DAC_10_11_diff_en
					<< DACx1416_GENCONFIG_DAC_10_11_DIFF_EN_BP)
			| (genConfig->DAC_8_9_diff_en
					<< DACx1416_GENCONFIG_DAC_8_9_DIFF_EN_BP)
			| (genConfig->DAC_6_7_diff_en
					<< DACx1416_GENCONFIG_DAC_6_7_DIFF_EN_BP)
			| (genConfig->DAC_4_5_diff_en
					<< DACx1416_GENCONFIG_DAC_4_5_DIFF_EN_BP)
			| (genConfig->DAC_2_3_diff_en
					<< DACx1416_GENCONFIG_DAC_2_3_DIFF_EN_BP)
			| (genConfig->DAC_0_1_diff_en
					<< DACx1416_GENCONFIG_DAC_0_1_DIFF_EN_BP);

	if (DACx1416_write_register(dac, DACx1416_REG_GENCONFIG, &genConfigRegVal)
			!= DACx1416_OK) {
		return DACx1416_ERROR; // Return the specific error from DACx1416_read_register
	}
	return DACx1416_OK;
}
DACx1416_error_t DACx1416_set_brdConfig(DACx1416 *dac,
		DACx1416_brdConfig *brdConfig) {
	uint16_t brdcstConfigRegVal = 0;

	for (int i = 0; i < 16; i++) {
		brdcstConfigRegVal |= (brdConfig->brdcast_en[i] << i);
	}
	if (DACx1416_write_register(dac, DACx1416_REG_BRDCONFIG,
			&brdcstConfigRegVal) != DACx1416_OK) {
		return DACx1416_ERROR; // Return the specific error from DACx1416_read_register
	}
	return DACx1416_OK;

}
DACx1416_error_t DACx1416_set_syncConfig(DACx1416 *dac,
		DACx1416_syncConfig *syncConfig) {
	uint16_t syncConfigRegVal = 0;

	for (int i = 0; i < 16; i++) {
		syncConfigRegVal |= (syncConfig->sync_en[i] << i);
	}
	if (DACx1416_write_register(dac, DACx1416_REG_SYNCCONFIG, &syncConfigRegVal)
			!= DACx1416_OK) {
		return DACx1416_ERROR; // Return the specific error from DACx1416_read_register
	}
	return DACx1416_OK;
}
DACx1416_error_t DACx1416_set_toggConfig(DACx1416 *dac,
		DACx1416_toggConfig *toggConfig) {
	uint16_t toggConfigRegVal[2] = { 0, 0 };

	// Loop for the first 8 toggles in regVal0
	for (int i = 0; i < 8; i++) {
		toggConfigRegVal[0] |= (toggConfig->toggle_en[15 - i] & 0x03)
				<< (14 - 2 * i);
	}

	// Loop for the next 8 toggles in regVal1
	for (int i = 0; i < 8; i++) {
		toggConfigRegVal[1] |= (toggConfig->toggle_en[7 - i] & 0x03)
				<< (14 - 2 * i);
	}

	if (DACx1416_write_register(dac, DACx1416_REG_TOGGCONFIG0,
			&toggConfigRegVal[0]) != DACx1416_OK) {
		return DACx1416_ERROR; // Return the specific error from DACx1416_read_register
	}

	if (DACx1416_write_register(dac, DACx1416_REG_TOGGCONFIG1,
			&toggConfigRegVal[1]) != DACx1416_OK) {
		return DACx1416_ERROR; // Return the specific error from DACx1416_read_register
	}

	return DACx1416_OK;
}
DACx1416_error_t DACx1416_set_pwdwn(DACx1416 *dac, DACx1416_dacPwdwn *dacPwdwn) {
	uint16_t dacPwrdwnRegVal = 0;

	for (int i = 0; i < 16; i++) {
		dacPwrdwnRegVal |= (dacPwdwn->dac_pwdwn[i] << i);
	}
	if (DACx1416_write_register(dac, DACx1416_REG_DACPWDWN, &dacPwrdwnRegVal)
			!= DACx1416_OK) {
		return DACx1416_ERROR; // Return the specific error from DACx1416_read_register
	}
	return DACx1416_OK;
}
DACx1416_error_t DACx1416_set_dacRange(DACx1416 *dac,
		DACx1416_dacRange *dacRange) {

	uint16_t dacRangeRegVals[4] = { 0, 0, 0, 0 };
	for (int i = 0; i < 16; i++) {
		dacRangeRegVals[i / 4] |= (dacRange->dacRange[15 - i] & 0x0F)
				<< (12 - 4 * (i % 4));
	}

	if (DACx1416_write_register(dac, DACx1416_REG_DACRANGE0,
			&dacRangeRegVals[0]) != DACx1416_OK) {
		return DACx1416_ERROR; // Return the specific error from DACx1416_read_register
	}

	if (DACx1416_write_register(dac, DACx1416_REG_DACRANGE1,
			&dacRangeRegVals[1]) != DACx1416_OK) {
		return DACx1416_ERROR; // Return the specific error from DACx1416_read_register
	}

	if (DACx1416_write_register(dac, DACx1416_REG_DACRANGE2,
			&dacRangeRegVals[2]) != DACx1416_OK) {
		return DACx1416_ERROR; // Return the specific error from DACx1416_read_register
	}

	if (DACx1416_write_register(dac, DACx1416_REG_DACRANGE3,
			&dacRangeRegVals[3]) != DACx1416_OK) {
		return DACx1416_ERROR; // Return the specific error from DACx1416_read_register
	}

	return DACx1416_OK;
}

DACx1416_error_t DACx1416_set_trigger(DACx1416 *dac,
		DACx1416_trigger_t triggerType) {
	uint16_t triggerRegVal = 0;
	switch (triggerType) {
	case DACx1416_TRIG_ALM_RESET:
		triggerRegVal = (1 << DACx1416_TRIG_ALM_RESET_BP);
		break;
	case DACx1416_TRIG_AB_TOG2:
		triggerRegVal = (1 << DACx1416_TRIG_AB_TOG2_BP);
		break;
	case DACx1416_TRIG_AB_TOG1:
		triggerRegVal = (1 << DACx1416_TRIG_AB_TOG1_BP);
		break;
	case DACx1416_TRIG_AB_TOG0:
		triggerRegVal = (1 << DACx1416_TRIG_AB_TOG0_BP);
		break;
	case DACx1416_TRIG_LDAC:
		triggerRegVal = (1 << DACx1416_TRIG_LDAC_BP);
		break;
	case DACx1416_TRIG_SOFT_RESET:
		triggerRegVal = DACx1416_TRIG_SOFT_RESEET_VALUE;
		break;
	default:
		break;
	}

	if (DACx1416_write_register(dac, DACx1416_REG_TRIGGER, &triggerRegVal)
			!= DACx1416_OK) {
		return DACx1416_ERROR; // Return the specific error from DACx1416_read_register
	}
	return DACx1416_OK;

}
DACx1416_error_t DACx1416_set_broadcast_value(DACx1416 *dac, uint16_t *value) {
	if (DACx1416_write_register(dac, DACx1416_REG_BRDCAST, value)
			!= DACx1416_OK) {
		return DACx1416_ERROR; // Return the specific error from DACx1416_read_register
	}
	return DACx1416_OK;
}
DACx1416_error_t DACx1416_set_dac_value(DACx1416 *dac, DACx1416_DAC_t ch,
		uint16_t *dacValue) {
	if (DACx1416_write_register(dac, DACx1416_REG_DAC0 + ch, dacValue)
			!= DACx1416_OK) {
		return DACx1416_ERROR; // Return the specific error from DACx1416_read_register
	}
	return DACx1416_OK;
}
DACx1416_error_t DACx1416_set_diff_offset(DACx1416 *dac,
		DACx1416_diff_offset *diffOffset) {
	uint16_t diffOffsetRegVals[4] = { 0, 0, 0, 0 };
	diffOffsetRegVals[0] = (diffOffset->dac_14_15 << 8)
			| (diffOffset->dac_12_13);
	diffOffsetRegVals[1] = (diffOffset->dac_10_11 << 8) | (diffOffset->dac_8_9);
	diffOffsetRegVals[2] = (diffOffset->dac_6_7 << 8) | (diffOffset->dac_4_5);
	diffOffsetRegVals[3] = (diffOffset->dac_2_3 << 8) | (diffOffset->dac_0_1);

	if (DACx1416_write_register(dac, DACx1416_REG_OFFSET0,
			&diffOffsetRegVals[0]) != DACx1416_OK) {
		return DACx1416_ERROR; // Return the specific error from DACx1416_read_register
	}

	if (DACx1416_write_register(dac, DACx1416_REG_OFFSET1,
			&diffOffsetRegVals[1]) != DACx1416_OK) {
		return DACx1416_ERROR; // Return the specific error from DACx1416_read_register
	}

	if (DACx1416_write_register(dac, DACx1416_REG_OFFSET2,
			&diffOffsetRegVals[2]) != DACx1416_OK) {
		return DACx1416_ERROR; // Return the specific error from DACx1416_read_register
	}

	if (DACx1416_write_register(dac, DACx1416_REG_OFFSET3,
			&diffOffsetRegVals[3]) != DACx1416_OK) {
		return DACx1416_ERROR; // Return the specific error from DACx1416_read_register
	}

	return DACx1416_OK;
}

DACx1416_error_t DACx1416_tempAlarmEnable(DACx1416 *dac, uint8_t enable) {
	dac->spiConfig.tempalm_en = enable;
	return DACx1416_set_spiConfig(dac, &dac->spiConfig);
}

DACx1416_error_t DACx1416_dacBusyEnable(DACx1416 *dac, uint8_t enable) {
	dac->spiConfig.dacbusy_en = enable;
	return DACx1416_set_spiConfig(dac, &dac->spiConfig);
}

DACx1416_error_t DACx1416_crcAlarmEnable(DACx1416 *dac, uint8_t enable) {
	dac->spiConfig.crcalm_en = enable;
	return DACx1416_set_spiConfig(dac, &dac->spiConfig);
}

DACx1416_error_t DACx1416_softToggleEnable(DACx1416 *dac, uint8_t enable) {
	dac->spiConfig.softtoggle_en = enable;
	return DACx1416_set_spiConfig(dac, &dac->spiConfig);
}

DACx1416_error_t DACx1416_devicePowerDown(DACx1416 *dac, uint8_t enable) {
	dac->spiConfig.dev_pwdwn = enable;
	return DACx1416_set_spiConfig(dac, &dac->spiConfig);
}

DACx1416_error_t DACx1416_crcEnable(DACx1416 *dac, uint8_t enable) {
	dac->spiConfig.crc_en = enable;
	return DACx1416_set_spiConfig(dac, &dac->spiConfig);
}

DACx1416_error_t DACx1416_streamEnable(DACx1416 *dac, uint8_t enable) {
	dac->spiConfig.str_en = enable;
	return DACx1416_set_spiConfig(dac, &dac->spiConfig);
}

DACx1416_error_t DACx1416_sdoEnable(DACx1416 *dac, uint8_t enable) {
	dac->spiConfig.sdo_en = enable;
	return DACx1416_set_spiConfig(dac, &dac->spiConfig);
}

DACx1416_error_t DACx1416_fsdoEnable(DACx1416 *dac, uint8_t enable) {
	dac->spiConfig.fsdo = enable;
	return DACx1416_set_spiConfig(dac, &dac->spiConfig);
}

DACx1416_error_t DACx1416_internalRefEnable(DACx1416 *dac, uint8_t enable) {
	dac->genConfig.ref_pwdwn = enable;
	return DACx1416_set_genConfig(dac, &dac->genConfig);
}

DACx1416_error_t DACx1416_diffChannelEnable(DACx1416 *dac,
		DACx1416_diffDAC_t diffCh, uint8_t enable) {

	switch (diffCh) {
	case DACx1416_DIFF_DAC_0_1: {
		dac->genConfig.DAC_0_1_diff_en = enable;
		return DACx1416_set_genConfig(dac, &dac->genConfig);
		break;
	}
	case DACx1416_DIFF_DAC_2_3: {
		dac->genConfig.DAC_2_3_diff_en = enable;
		return DACx1416_set_genConfig(dac, &dac->genConfig);
		break;
	}
	case DACx1416_DIFF_DAC_4_5: {
		dac->genConfig.DAC_4_5_diff_en = enable;
		return DACx1416_set_genConfig(dac, &dac->genConfig);
		break;
	}
	case DACx1416_DIFF_DAC_6_7: {
		dac->genConfig.DAC_6_7_diff_en = enable;
		return DACx1416_set_genConfig(dac, &dac->genConfig);
		break;
	}
	case DACx1416_DIFF_DAC_8_9: {
		dac->genConfig.DAC_8_9_diff_en = enable;
		return DACx1416_set_genConfig(dac, &dac->genConfig);
		break;
	}
	case DACx1416_DIFF_DAC_10_11: {
		dac->genConfig.DAC_10_11_diff_en = enable;
		return DACx1416_set_genConfig(dac, &dac->genConfig);
		break;
	}
	case DACx1416_DIFF_DAC_12_13: {
		dac->genConfig.DAC_12_13_diff_en = enable;
		return DACx1416_set_genConfig(dac, &dac->genConfig);
		break;
	}
	case DACx1416_DIFF_DAC_14_15: {
		dac->genConfig.DAC_14_15_diff_en = enable;
		return DACx1416_set_genConfig(dac, &dac->genConfig);
		break;
	}
	default:
		return DACx1416_ERROR;
		break;
	}

}

DACx1416_error_t DACx1416_channelBroadcastEnable(DACx1416 *dac,
		DACx1416_DAC_t Ch, uint8_t enable) {
	dac->brdcstConfig.brdcast_en[Ch] = enable;
	return DACx1416_set_brdConfig(dac, &dac->brdcstConfig);
}

DACx1416_error_t DACx1416_channelsyncModeEnable(DACx1416 *dac,
		DACx1416_DAC_t Ch, uint8_t enable) {
	dac->syncConfig.sync_en[Ch] = enable;
	return DACx1416_set_syncConfig(dac, &dac->syncConfig);
}

DACx1416_error_t DACx1416_channelToggleMode(DACx1416 *dac, DACx1416_DAC_t Ch,
		DACx1416_toggleMode_t mode) {
	dac->toggConfig.toggle_en[Ch] = mode;
	return DACx1416_set_toggConfig(dac, &dac->toggConfig);
}

DACx1416_error_t DACx1416_channelEnable(DACx1416 *dac, DACx1416_DAC_t Ch,
		uint8_t enable) {
	dac->chPwrDwn.dac_pwdwn[Ch] = !enable;
	return DACx1416_set_pwdwn(dac, &dac->chPwrDwn);
	//DACx1416_get_pwdwn(DACx1416 *dac, DACx1416_dacPwdwn *dacPwdwn)
}

DACx1416_error_t DACx1416_channelRange(DACx1416 *dac, DACx1416_DAC_t Ch,
		DACx1416_dacRange_t range) {
	dac->dacRange.dacRange[Ch] = range;
	return DACx1416_set_dacRange(dac, &dac->dacRange);
}

DACx1416_error_t DACx1416_softAlarmReset(DACx1416 *dac) {
	return DACx1416_set_trigger(dac, DACx1416_TRIG_ALM_RESET);
}

DACx1416_error_t DACx1416_softToggle(DACx1416 *dac, uint8_t toggleBit) {
	switch (toggleBit) {
	case 0: {
		return DACx1416_set_trigger(dac, DACx1416_TRIG_AB_TOG0);
		break;
	}

	case 1: {
		return DACx1416_set_trigger(dac, DACx1416_TRIG_AB_TOG1);
		break;
	}
	case 2: {
		return DACx1416_set_trigger(dac, DACx1416_TRIG_AB_TOG2);
		break;
	}
	default:
		return DACx1416_ERROR;
		break;
	}
}

DACx1416_error_t DACx1416_softReset(DACx1416 *dac) {
	return DACx1416_set_trigger(dac, DACx1416_TRIG_SOFT_RESET);
}

DACx1416_error_t DACx1416_broadcastValue(DACx1416 *dac, DACx1416_DAC_t Ch,
		uint16_t value) {
	return DACx1416_set_broadcast_value(dac, &value);
}

DACx1416_error_t DACx1416_channelValue(DACx1416 *dac, DACx1416_DAC_t Ch,
		uint16_t value) {
	return DACx1416_set_dac_value(dac, Ch, &value);
}

DACx1416_error_t DACx1416_diffChannelOffset(DACx1416 *dac,
		DACx1416_diffDAC_t diffCh, uint8_t value) {
	switch (diffCh) {
	case DACx1416_DIFF_DAC_14_15:
		dac->diffOffset.dac_14_15 = value;
		break;
	case DACx1416_DIFF_DAC_12_13:
		dac->diffOffset.dac_12_13 = value;
		break;
	case DACx1416_DIFF_DAC_10_11:
		dac->diffOffset.dac_10_11 = value;
		break;
	case DACx1416_DIFF_DAC_8_9:
		dac->diffOffset.dac_8_9 = value;
		break;
	case DACx1416_DIFF_DAC_6_7:
		dac->diffOffset.dac_6_7 = value;
		break;
	case DACx1416_DIFF_DAC_4_5:
		dac->diffOffset.dac_4_5 = value;
		break;
	case DACx1416_DIFF_DAC_2_3:
		dac->diffOffset.dac_2_3 = value;
		break;
	case DACx1416_DIFF_DAC_0_1:
		dac->diffOffset.dac_0_1 = value;
		break;
	default:
		return DACx1416_ERROR;  // Return error if the diffCh is invalid
	}
	return DACx1416_set_diff_offset(dac, &dac->diffOffset);
}

