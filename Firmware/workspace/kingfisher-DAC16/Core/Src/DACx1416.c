/*
 * DACx1416.c
 *
 *  Created on: Oct 18, 2024
 *      Author: habiburrahman
 */
#include "main.h"
#include "string.h"
#include "DACx1416.h"



extern volatile uint8_t dacUseCRC;
extern SPI_HandleTypeDef hspi1;
extern volatile uint8_t dacTransferComplete;

void DACx1416_nCS(bool value){
	if(value){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // Set CS high
	}
	else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Set CS low
	}
}

void DACx1416_nLDAC(bool value){
	if(value){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // Set CS high
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // Set CS low
	}
}
void DACx1416_nRESET(bool value){
	if(value){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // Set CS high
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // Set CS low
	}
}
void DACx1416_nCLR(bool value){
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




//void SPI1_TransmitReceive(uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size)
//{
//    // Pull CS low to start the transmission
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Set CS low (adjust GPIO pin as needed)
//
//    // Full-duplex transmission and reception (blocking mode)
//
//    if(dacTransferComplete){
//    	dacTransferComplete = 0;
//    	if (HAL_SPI_TransmitReceive_IT(&hspi1, txBuffer, rxBuffer, size) != HAL_OK)
//		{
//			// Communication error
//			Error_Handler();
//		}
//    }
//
//
//
//    // Pull CS high to end the transmission
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // Set CS high (adjust GPIO pin as needed)
//}
//
//
//void DACx1416_SPI_transmit(uint8_t *txBuffer, uint16_t size) {
//    uint8_t rxBuffer[size];  // Dummy receive buffer (must be static if used in interrupt)
//
//    // Initiate interrupt-based transmit-receive (full-duplex)
//    if(dacTransferComplete){
//    	dacTransferComplete = 0;
//    	if (HAL_SPI_TransmitReceive_IT(&hspi1, txBuffer, rxBuffer, size) != HAL_OK)
//		{
//			// Communication error
//			//Error_Handler();
//		}
//    	while (!dacTransferComplete)
//		{
//			// Optional: You could add a timeout check here if needed
//		}
//    }
//}
//
//
//void DACx1416_SPI_receive(uint8_t *rxBuffer, uint16_t size) {
//    uint8_t txBuffer[size];  // Dummy transmit buffer (must be static if used in interrupt)
//    memset(txBuffer, 0x00, size);  // Fill with dummy bytes (0x00 or 0xFF)
//
//    // Initiate interrupt-based receive (full-duplex)
//    if(dacTransferComplete){
//    	dacTransferComplete = 0;
//    	if (HAL_SPI_TransmitReceive_IT(&hspi1, txBuffer, rxBuffer, size) != HAL_OK)
//		{
//			// Communication error
//			//Error_Handler();
//		}
//
//    	while (!dacTransferComplete)
//		{
//			// Optional: You could add a timeout check here if needed
//		}
//    }
//}
//
//void DACx1416_SPI_transmitReceive(uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size) {
//    // Initiate interrupt-based transmit-receive (full-duplex)
//    if(dacTransferComplete){
//    	dacTransferComplete = 0;
//    	if (HAL_SPI_TransmitReceive_IT(&hspi1, txBuffer, rxBuffer, size) != HAL_OK)
//		{
//			// Communication error
//			//Error_Handler();
//		}
//    	while (!dacTransferComplete)
//		{
//			// Optional: You could add a timeout check here if needed
//		}
//    }
//}


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



void DACx1416_write_register_old(uint8_t address, uint16_t data){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // Set CS low (adjust GPIO pin as needed)
	uint8_t txBuffer[3] = {address & ~(1<<7) , (data >> 8) & 0xFF, data & 0xFF};
	uint8_t rxBuffer[3];
	SPI1_TransmitReceive(txBuffer, rxBuffer,  3);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // Set CS high (adjust GPIO pin as needed)
}

uint16_t DACx1416_read_register_old(uint8_t address){
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




void DACx1416_write_register(DACx1416_HandleTypeDef *dac, DACx1416_register_t reg, uint16_t data){
	if (!dacUseCRC){
		uint8_t txBuffer[3] = {reg & ~(1<<7) , (data >> 8) & 0xFF, data & 0xFF};
		uint8_t rxBuffer[3];
		dac->nCS(0);
		dac->SPI_transmitReceive(txBuffer, rxBuffer, 3);
		dac->nCS(1);
	}
	else{
		uint8_t txBuffer[4] = {reg & ~(1<<7) , (data >> 8) & 0xFF, data & 0xFF, 0};
		uint8_t rxBuffer[4];
		uint8_t crc8 = calculate_crc8(txBuffer, 3);
		txBuffer[3] = crc8;
		dac->nCS(0);
		dac->SPI_transmitReceive(txBuffer, rxBuffer, 4);
		dac->nCS(1);
	}


}
uint16_t DACx1416_read_register(DACx1416_HandleTypeDef *dac, DACx1416_register_t reg){
	if (!dacUseCRC){
		uint8_t txBuffer[3] = {reg | (1<<7) , 0xFF, 0xFF};
		uint8_t rxBuffer[3];
		dac->nCS(0);
		dac->SPI_transmitReceive(txBuffer, rxBuffer, 3);
		dac->nCS(1);
		dac->nCS(0);
		dac->SPI_receive(rxBuffer, 3);
		dac->nCS(1);
		return rxBuffer[1]<<8 | rxBuffer[2];
	}
	else{
		uint8_t txBuffer[4] = {reg | (1<<7) , 0xFF, 0xFF, 0};
		uint8_t rxBuffer[4];
		uint8_t crc8 = calculate_crc8(txBuffer, 3);
		txBuffer[3] = crc8;
		dac->nCS(0);
		dac->SPI_transmitReceive(txBuffer, rxBuffer, 4);
		dac->nCS(1);
		dac->nCS(0);
		dac->SPI_receive(rxBuffer, 4);
		dac->nCS(1);
		return rxBuffer[1]<<8 | rxBuffer[2];
	}


}



DACx1416_deviceID_t DACx1416_get_device_id(DACx1416_HandleTypeDef *dac){
	DACx1416_deviceID_t deviceID;
	uint16_t deviceIdVal = DACx1416_read_register(dac,DACx1416_REG_DEVICEID);
	deviceID.deviceId = (deviceIdVal>>2) & 0x3FFF;
	deviceID.versionId = deviceIdVal & 0x0003;
	return deviceID;
}

DACx1416_status_t DACx1416_get_status(DACx1416_HandleTypeDef *dac){
	DACx1416_status_t status;
	uint16_t statusRegVal = DACx1416_read_register(dac, DACx1416_REG_STATUS);

	status.crc_alarm = (statusRegVal>>2) & 0x01;
	status.dac_busy = (statusRegVal>>1) & 0x01;
	status.temp_alarm = statusRegVal & 0x01;

	return status;
}



DACx1416_spiconfig_t DACx1416_get_spiConfig(DACx1416_HandleTypeDef *dac){
	DACx1416_spiconfig_t spiConfig;
	uint16_t regVal = DACx1416_read_register(dac, DACx1416_REG_SPICONFIG);

	spiConfig.tempalm_en = (regVal >> 11) & 0x01;
	spiConfig.dacbusy_en = (regVal >> 10) & 0x01;
	spiConfig.crcalm_en = (regVal >> 9) & 0x01;
	spiConfig.softtoggle_en = (regVal >> 6) & 0x01;
	spiConfig.dev_pwdwn = (regVal >> 5) & 0x01;
	spiConfig.crc_en = (regVal >> 4) & 0x01;
	spiConfig.str_en = (regVal >> 3) & 0x01;
	spiConfig.sdo_en = (regVal >> 2) & 0x01;
	spiConfig.fsdo = (regVal >> 1) & 0x01;


	return spiConfig;
}
void DACx1416_set_spiConfig(DACx1416_HandleTypeDef *dac, DACx1416_spiconfig_t spiConfig){

	uint16_t regVal = (spiConfig.tempalm_en<<11) | (spiConfig.dacbusy_en<<10) | (spiConfig.crcalm_en<<9)  |
			(spiConfig.softtoggle_en<<6)  | (spiConfig.dev_pwdwn<<5)  | (spiConfig.crc_en<<4)  |
			(spiConfig.str_en<<3)  | (spiConfig.sdo_en<<2)  | (spiConfig.fsdo<<1)  ;

	DACx1416_write_register(dac, DACx1416_REG_SPICONFIG, regVal);

}


DACx1416_genconfig_t DACx1416_get_genConfig(DACx1416_HandleTypeDef *dac){
	DACx1416_genconfig_t genConfig;
	uint16_t regVal = DACx1416_read_register(dac, DACx1416_REG_GENCONFIG);


	genConfig.ref_pwdwn = (regVal >>14) & 0x01;
	genConfig.DAC_14_15_diff_en = (regVal >>7) & 0x01;
	genConfig.DAC_12_13_diff_en = (regVal >>6) & 0x01;
	genConfig.DAC_10_11_diff_en = (regVal >>5) & 0x01;
	genConfig.DAC_8_9_diff_en = (regVal >>4) & 0x01;
	genConfig.DAC_6_7_diff_en = (regVal >>3) & 0x01;
	genConfig.DAC_4_5_diff_en = (regVal >>2) & 0x01;
	genConfig.DAC_2_3_diff_en = (regVal >>1) & 0x01;
	genConfig.DAC_0_1_diff_en = (regVal >>0) & 0x01;

	return genConfig;

}
void DACx1416_set_genConfig(DACx1416_HandleTypeDef *dac, DACx1416_genconfig_t genConfig){

	uint16_t regVal = (genConfig.ref_pwdwn<<14) | (genConfig.DAC_14_15_diff_en<<7) | (genConfig.DAC_12_13_diff_en<<6) |
			(genConfig.DAC_10_11_diff_en<<5) |(genConfig.DAC_8_9_diff_en<<4) |(genConfig.DAC_6_7_diff_en<<3) |
			(genConfig.DAC_4_5_diff_en<<2) |(genConfig.DAC_2_3_diff_en<<1) |(genConfig.DAC_0_1_diff_en<<0) ;
	DACx1416_write_register(dac, DACx1416_REG_GENCONFIG, regVal);

}


DACx1416_brdConfig DACx1416_get_brdConfig(DACx1416_HandleTypeDef *dac){
	DACx1416_brdConfig brdConfig;
	uint16_t regVal = DACx1416_read_register(dac, DACx1416_REG_BRDCONFIG);

	for (int i=0;i<16;i++){
		brdConfig.brdcast_en[i] = (regVal>>i) & 0x01;
	}

	return brdConfig;


}
void DACx1416_set_brdConfig(DACx1416_HandleTypeDef *dac, DACx1416_brdConfig brdConfig){

	uint16_t regVal = 0;

	for (int i=0;i<16;i++){
		regVal |= (brdConfig.brdcast_en[i]<<i);
	}

	DACx1416_write_register(dac, DACx1416_REG_BRDCONFIG, regVal);

}




DACx1416_syncConfig DACx1416_get_syncConfig(DACx1416_HandleTypeDef *dac){
	DACx1416_syncConfig syncConfig;
	uint16_t regVal = DACx1416_read_register(dac, DACx1416_REG_SYNCCONFIG);

	for (int i=0;i<16;i++){
		syncConfig.sync_en[i] = (regVal>>i) & 0x01;
	}

	return syncConfig;



}
void DACx1416_set_syncConfig(DACx1416_HandleTypeDef *dac, DACx1416_syncConfig syncConfig){
	uint16_t regVal = 0;

	for (int i=0;i<16;i++){
		regVal |= (syncConfig.sync_en[i]<<i);
	}

	DACx1416_write_register(dac, DACx1416_REG_SYNCCONFIG, regVal);
}



//DACx1416_toggConfig_t DACx1416_get_toggConfig(DACx1416_HandleTypeDef *dac){
//	DACx1416_toggConfig_t toggConfig;
//	uint16_t regVal0 = DACx1416_read_register(dac, DACx1416_REG_TOGGCONFIG0);
//	uint16_t regVal1 = DACx1416_read_register(dac, DACx1416_REG_TOGGCONFIG1);
//
//
//	toggConfig.toggle_en[15] = (regVal0 >> 14) & 0x03;
//	toggConfig.toggle_en[14] = (regVal0 >> 12) & 0x03;
//	toggConfig.toggle_en[13] = (regVal0 >> 10) & 0x03;
//	toggConfig.toggle_en[12] = (regVal0 >> 8) & 0x03;
//	toggConfig.toggle_en[11] = (regVal0 >> 6) & 0x03;
//	toggConfig.toggle_en[10] = (regVal0 >> 4) & 0x03;
//	toggConfig.toggle_en[9] = (regVal0 >> 2) & 0x03;
//	toggConfig.toggle_en[8] = (regVal0) & 0x03;
//	toggConfig.toggle_en[7] = (regVal1 >> 14) & 0x03;
//	toggConfig.toggle_en[6] = (regVal1 >> 12) & 0x03;
//	toggConfig.toggle_en[5] = (regVal1 >> 19) & 0x03;
//	toggConfig.toggle_en[4] = (regVal1 >> 8) & 0x03;
//	toggConfig.toggle_en[3] = (regVal1 >> 6) & 0x03;
//	toggConfig.toggle_en[2] = (regVal1 >> 4) & 0x03;
//	toggConfig.toggle_en[1] = (regVal1 >> 2) & 0x03;
//	toggConfig.toggle_en[0] = (regVal1) & 0x03;
//
//	return toggConfig;
//
//
//
//}
//
//
//void DACx1416_set_toggConfig(DACx1416_HandleTypeDef *dac, DACx1416_toggConfig_t toggConfig) {
//    uint16_t regVal0 = 0;
//    uint16_t regVal1 = 0;
//
//    // Configure the first 8 channels in regVal0
//    regVal0 |= (toggConfig.toggle_en[15] & 0x03) << 14;
//    regVal0 |= (toggConfig.toggle_en[14] & 0x03) << 12;
//    regVal0 |= (toggConfig.toggle_en[13] & 0x03) << 10;
//    regVal0 |= (toggConfig.toggle_en[12] & 0x03) << 8;
//    regVal0 |= (toggConfig.toggle_en[11] & 0x03) << 6;
//    regVal0 |= (toggConfig.toggle_en[10] & 0x03) << 4;
//    regVal0 |= (toggConfig.toggle_en[9] & 0x03) << 2;
//    regVal0 |= (toggConfig.toggle_en[8] & 0x03);
//
//    // Configure the next 8 channels in regVal1
//    regVal1 |= (toggConfig.toggle_en[7] & 0x03) << 14;
//    regVal1 |= (toggConfig.toggle_en[6] & 0x03) << 12;
//    regVal1 |= (toggConfig.toggle_en[5] & 0x03) << 10;
//    regVal1 |= (toggConfig.toggle_en[4] & 0x03) << 8;
//    regVal1 |= (toggConfig.toggle_en[3] & 0x03) << 6;
//    regVal1 |= (toggConfig.toggle_en[2] & 0x03) << 4;
//    regVal1 |= (toggConfig.toggle_en[1] & 0x03) << 2;
//    regVal1 |= (toggConfig.toggle_en[0] & 0x03);
//
//    // Write the values to the respective DAC registers
//    DACx1416_write_register(dac, DACx1416_REG_TOGGCONFIG0, regVal0);
//    DACx1416_write_register(dac, DACx1416_REG_TOGGCONFIG1, regVal1);
//}



DACx1416_toggConfig DACx1416_get_toggConfig(DACx1416_HandleTypeDef *dac) {
	DACx1416_toggConfig toggConfig;
	uint16_t regVal[2];
    regVal[0] = DACx1416_read_register(dac, DACx1416_REG_TOGGCONFIG0);
    regVal[1] = DACx1416_read_register(dac, DACx1416_REG_TOGGCONFIG1);

    // Loop for the first 8 toggles in regVal0
    for (int i = 0; i < 8; i++) {
        toggConfig.toggle_en[15 - i] = (regVal[0] >> (14 - 2 * i)) & 0x03;
    }

    // Loop for the next 8 toggles in regVal1
    for (int i = 0; i < 8; i++) {
        toggConfig.toggle_en[7 - i] = (regVal[1] >> (14 - 2 * i)) & 0x03;
    }

    return toggConfig;
}

void DACx1416_set_toggConfig(DACx1416_HandleTypeDef *dac, DACx1416_toggConfig toggConfig) {
	uint16_t regVal[2];
    regVal[0] = 0;
    regVal[1] = 0;

    // Loop for the first 8 toggles in regVal0
    for (int i = 0; i < 8; i++) {
        regVal[0] |= (toggConfig.toggle_en[15 - i] & 0x03) << (14 - 2 * i);
    }

    // Loop for the next 8 toggles in regVal1
    for (int i = 0; i < 8; i++) {
        regVal[1] |= (toggConfig.toggle_en[7 - i] & 0x03) << (14 - 2 * i);
    }

    // Write the values to the respective DAC registers
    DACx1416_write_register(dac, DACx1416_REG_TOGGCONFIG0, regVal[0]);
    DACx1416_write_register(dac, DACx1416_REG_TOGGCONFIG1, regVal[1]);
}


DACx1416_dacPwdwn DACx1416_get_pwdwn(DACx1416_HandleTypeDef *dac){
	DACx1416_dacPwdwn dacPwdwn;
	uint16_t regVal = DACx1416_read_register(dac, DACx1416_REG_DACPWDWN);

	for (int i=0;i<16;i++){
		dacPwdwn.dac_pwdwn[i] = (regVal>>i) & 0x01;
	}

	return dacPwdwn;



}
void DACx1416_set_pwdwn(DACx1416_HandleTypeDef *dac, DACx1416_dacPwdwn dacPwdwn){
	uint16_t regVal = 0;

	for (int i=0;i<16;i++){
		regVal |= (dacPwdwn.dac_pwdwn[i]<<i);
	}

	DACx1416_write_register(dac, DACx1416_REG_DACPWDWN, regVal);
}


//DACx1416_dacRange DACx1416_get_dacRange(DACx1416_HandleTypeDef *dac){
//	DACx1416_dacRange dacRange;
//	uint16_t regVal0 = 0;
//	uint16_t regVal1 = 0;
//	uint16_t regVal2 = 0;
//	uint16_t regVal3 = 0;
//
//	dacRange.dacRange[15] = (regVal0>>12) & 0x0F;
//	dacRange.dacRange[14] = (regVal0>>8) & 0x0F;
//	dacRange.dacRange[13] = (regVal0>>4) & 0x0F;
//	dacRange.dacRange[12] = (regVal0) & 0x0F;
//
//	dacRange.dacRange[11] = (regVal1>>12) & 0x0F;
//	dacRange.dacRange[10] = (regVal1>>8) & 0x0F;
//	dacRange.dacRange[9] = (regVal1>>4) & 0x0F;
//	dacRange.dacRange[8] = (regVal1) & 0x0F;
//
//	dacRange.dacRange[7] = (regVal2>>12) & 0x0F;
//	dacRange.dacRange[6] = (regVal2>>8) & 0x0F;
//	dacRange.dacRange[5] = (regVal2>>4) & 0x0F;
//	dacRange.dacRange[4] = (regVal2) & 0x0F;
//
//	dacRange.dacRange[3] = (regVal3>>12) & 0x0F;
//	dacRange.dacRange[2] = (regVal3>>8) & 0x0F;
//	dacRange.dacRange[1] = (regVal3>>4) & 0x0F;
//	dacRange.dacRange[0] = (regVal3) & 0x0F;
//
//	return dacRange;
//
//}
//
//
//
//
//void DACx1416_set_dacRange(DACx1416_HandleTypeDef *dac, DACx1416_dacRange dacRange) {
//    uint16_t regVal0 = 0;
//    uint16_t regVal1 = 0;
//    uint16_t regVal2 = 0;
//    uint16_t regVal3 = 0;
//
//    // Pack values into regVal0 for channels 15 to 12
//    regVal0 |= (dacRange.dacRange[15] & 0x0F) << 12;
//    regVal0 |= (dacRange.dacRange[14] & 0x0F) << 8;
//    regVal0 |= (dacRange.dacRange[13] & 0x0F) << 4;
//    regVal0 |= (dacRange.dacRange[12] & 0x0F);
//
//    // Pack values into regVal1 for channels 11 to 8
//    regVal1 |= (dacRange.dacRange[11] & 0x0F) << 12;
//    regVal1 |= (dacRange.dacRange[10] & 0x0F) << 8;
//    regVal1 |= (dacRange.dacRange[9] & 0x0F) << 4;
//    regVal1 |= (dacRange.dacRange[8] & 0x0F);
//
//    // Pack values into regVal2 for channels 7 to 4
//    regVal2 |= (dacRange.dacRange[7] & 0x0F) << 12;
//    regVal2 |= (dacRange.dacRange[6] & 0x0F) << 8;
//    regVal2 |= (dacRange.dacRange[5] & 0x0F) << 4;
//    regVal2 |= (dacRange.dacRange[4] & 0x0F);
//
//    // Pack values into regVal3 for channels 3 to 0
//    regVal3 |= (dacRange.dacRange[3] & 0x0F) << 12;
//    regVal3 |= (dacRange.dacRange[2] & 0x0F) << 8;
//    regVal3 |= (dacRange.dacRange[1] & 0x0F) << 4;
//    regVal3 |= (dacRange.dacRange[0] & 0x0F);
//
//    // Write the packed values to the respective DAC registers
//    DACx1416_write_register(dac, DACx1416_REG_DACRANGE0, regVal0);
//    DACx1416_write_register(dac, DACx1416_REG_DACRANGE1, regVal1);
//    DACx1416_write_register(dac, DACx1416_REG_DACRANGE2, regVal2);
//    DACx1416_write_register(dac, DACx1416_REG_DACRANGE3, regVal3);
//}



DACx1416_dacRange DACx1416_get_dacRange(DACx1416_HandleTypeDef *dac) {
    DACx1416_dacRange dacRange;
    uint16_t regVals[4];

    // Read all four registers
    regVals[0] = DACx1416_read_register(dac, DACx1416_REG_DACRANGE0);
    regVals[1] = DACx1416_read_register(dac, DACx1416_REG_DACRANGE1);
    regVals[2] = DACx1416_read_register(dac, DACx1416_REG_DACRANGE2);
    regVals[3] = DACx1416_read_register(dac, DACx1416_REG_DACRANGE3);

    // Extract each 4-bit range value
    for (int i = 0; i < 16; i++) {
        dacRange.dacRange[15 - i] = (regVals[i / 4] >> (12 - 4 * (i % 4))) & 0x0F;
    }

    return dacRange;
}

void DACx1416_set_dacRange(DACx1416_HandleTypeDef *dac, DACx1416_dacRange dacRange) {
    uint16_t regVals[4] = {0, 0, 0, 0};

    // Pack each 4-bit value into the correct register and position
    for (int i = 0; i < 16; i++) {
        regVals[i / 4] |= (dacRange.dacRange[15 - i] & 0x0F) << (12 - 4 * (i % 4));
    }

    // Write each packed value to the respective DAC register
    DACx1416_write_register(dac, DACx1416_REG_DACRANGE0, regVals[0]);
    DACx1416_write_register(dac, DACx1416_REG_DACRANGE1, regVals[1]);
    DACx1416_write_register(dac, DACx1416_REG_DACRANGE2, regVals[2]);
    DACx1416_write_register(dac, DACx1416_REG_DACRANGE3, regVals[3]);
}


void DACx1416_set_trigger(DACx1416_HandleTypeDef *dac, DACx1416_trigger_t triggerType){

	uint16_t regVal = 0;
	switch(triggerType){
	case DACx1416_TRIG_ALM_RESET:
		regVal = (1<<8);
		break;
	case DACx1416_TRIG_AB_TOG2:
		regVal = (1<<7);
		break;
	case DACx1416_TRIG_AB_TOG1:
		regVal = (1<<6);
		break;
	case DACx1416_TRIG_AB_TOG0:
		regVal = (1<<5);
		break;
	case DACx1416_TRIG_LDAC:
		regVal = (1<<4);
		break;
	case DACx1416_TRIG_SOFT_RESEET:
		regVal = 0b1010;
		break;
	default:
		break;
	}

	DACx1416_write_register(dac, DACx1416_REG_TRIGGER, regVal);

}


void DACx1416_set_broadcast_value(DACx1416_HandleTypeDef *dac, uint16_t value){
	DACx1416_write_register(dac, DACx1416_REG_BRDCAST, value);
}

void DACx1416_set_dac_value(DACx1416_HandleTypeDef *dac, DACx1416_DAC_t ch, uint16_t value){
	DACx1416_write_register(dac, DACx1416_REG_DAC0 + ch, value);
}


void DACx1416_set_diff_offset(DACx1416_HandleTypeDef *dac, DACx1416_diff_offset diffOffset){
	uint16_t regVal[4];

	regVal[0] = (diffOffset.dac_14_15 << 8) |  (diffOffset.dac_12_13) ;
	regVal[1] = (diffOffset.dac_10_11 << 8) |  (diffOffset.dac_8_9) ;
	regVal[2] = (diffOffset.dac_6_7 << 8) |  (diffOffset.dac_4_5) ;
	regVal[3] = (diffOffset.dac_2_3 << 8) |  (diffOffset.dac_0_1) ;


	DACx1416_write_register(dac, DACx1416_REG_OFFSET0, regVal[0]);
	DACx1416_write_register(dac, DACx1416_REG_OFFSET1, regVal[1]);
	DACx1416_write_register(dac, DACx1416_REG_OFFSET2, regVal[2]);
	DACx1416_write_register(dac, DACx1416_REG_OFFSET3, regVal[3]);
}







