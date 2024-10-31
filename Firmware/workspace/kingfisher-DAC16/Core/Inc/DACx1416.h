/*
 * DACx1416.h
 *
 *  Created on: Oct 18, 2024
 *      Author: habiburrahman
 */

#ifndef INC_DACX1416_H_
#define INC_DACX1416_H_

#include "main.h"
#include <stdbool.h>


typedef enum{
	DACx1416_REG_NOP = 0,
	DACx1416_REG_DEVICEID,
	DACx1416_REG_STATUS,
	DACx1416_REG_SPICONFIG,
	DACx1416_REG_GENCONFIG,
	DACx1416_REG_BRDCONFIG,
	DACx1416_REG_SYNCCONFIG,
	DACx1416_REG_TOGGCONFIG0,
	DACx1416_REG_TOGGCONFIG1,
	DACx1416_REG_DACPWDWN,
	DACx1416_REG_DACRANGE0,
	DACx1416_REG_DACRANGE1,
	DACx1416_REG_DACRANGE2,
	DACx1416_REG_DACRANGE3,
	DACx1416_REG_TRIGGER,
	DACx1416_REG_BRDCAST,
	DACx1416_REG_DAC0,
	DACx1416_REG_DAC1,
	DACx1416_REG_DAC2,
	DACx1416_REG_DAC3,
	DACx1416_REG_DAC4,
	DACx1416_REG_DAC5,
	DACx1416_REG_DAC6,
	DACx1416_REG_DAC7,
	DACx1416_REG_DAC8,
	DACx1416_REG_DAC9,
	DACx1416_REG_DAC10,
	DACx1416_REG_DAC11,
	DACx1416_REG_DAC12,
	DACx1416_REG_DAC13,
	DACx1416_REG_DAC14,
	DACx1416_REG_DAC15,
	DACx1416_REG_OFFSET0,
	DACx1416_REG_OFFSET1,
	DACx1416_REG_OFFSET2,
	DACx1416_REG_OFFSET3,
}DACx1416_register_t;



typedef enum{
	DACx1416_DAC0 = 0,
	DACx1416_DAC1,
	DACx1416_DAC2,
	DACx1416_DAC3,
	DACx1416_DAC4,
	DACx1416_DAC5,
	DACx1416_DAC6,
	DACx1416_DAC7,
	DACx1416_DAC8,
	DACx1416_DAC9,
	DACx1416_DAC10,
	DACx1416_DAC11,
	DACx1416_DAC12,
	DACx1416_DAC13,
	DACx1416_DAC14,
	DACx1416_DAC15
}DACx1416_DAC_t;


typedef enum{
	DACx1416_TOGGLE_MODE_DISABLED,
	DACx1416_TOGGLE_MODE_TOGGLE0,
	DACx1416_TOGGLE_MODE_TOGGLE1,
	DACx1416_TOGGLE_MODE_TOGGLE2,
}DACx1416_toggleMode_t;


typedef enum{
	DACx1416_DACRANGE_0_to_p5 = 0b0000,
	DACx1416_DACRANGE_0_to_p10 = 0b0001,
	DACx1416_DACRANGE_0_to_p20 = 0b0010,
	DACx1416_DACRANGE_0_to_p40 = 0b0100,
	DACx1416_DACRANGE_n5_to_p5 = 0b1001,
	DACx1416_DACRANGE_n10_to_p10  = 0b1010,
	DACx1416_DACRANGE_n20_to_p20 = 0b1100,
	DACx1416_DACRANGE_n2_5_to_p2_5 = 0b1110,
}DACx1416_dacRange_t;

typedef struct{
	uint16_t deviceId;
	uint8_t versionId;
}DACx1416_deviceID_t;

typedef struct{
	bool crc_alarm;
	bool dac_busy;
	bool temp_alarm;
}DACx1416_status_t;

typedef struct{
	bool tempalm_en;
	bool dacbusy_en;
	bool crcalm_en;
	bool softtoggle_en;
	bool dev_pwdwn;
	bool crc_en;
	bool str_en;
	bool sdo_en;
	bool fsdo;
}DACx1416_spiconfig_t;


typedef struct{
	bool ref_pwdwn;
	bool DAC_14_15_diff_en;
	bool DAC_12_13_diff_en;
	bool DAC_10_11_diff_en;
	bool DAC_8_9_diff_en;
	bool DAC_6_7_diff_en;
	bool DAC_4_5_diff_en;
	bool DAC_2_3_diff_en;
	bool DAC_0_1_diff_en;
}DACx1416_genconfig_t;





typedef struct{
	bool brdcast_en[16];
}DACx1416_brdConfig;


typedef struct{
	bool sync_en[16];
}DACx1416_syncConfig;



typedef struct{
	DACx1416_toggleMode_t toggle_en[16];
}DACx1416_toggConfig;


typedef struct{
	bool dac_pwdwn[16];
}DACx1416_dacPwdwn;


typedef struct{
	DACx1416_dacRange_t dacRange[16];
}DACx1416_dacRange;



typedef enum{
	DACx1416_TRIG_ALM_RESET,
	DACx1416_TRIG_AB_TOG2,
	DACx1416_TRIG_AB_TOG1,
	DACx1416_TRIG_AB_TOG0,
	DACx1416_TRIG_LDAC,
	DACx1416_TRIG_SOFT_RESEET,
}DACx1416_trigger_t;


typedef struct{
	int8_t dac_14_15;
	int8_t dac_12_13;
	int8_t dac_10_11;
	int8_t dac_8_9;
	int8_t dac_6_7;
	int8_t dac_4_5;
	int8_t dac_2_3;
	int8_t dac_0_1;
}DACx1416_diff_offset;


typedef struct {
    // Function pointers for SPI operations
    void (*SPI_transmit)(uint8_t *txBuffer, uint16_t size);
    void (*SPI_receive)(uint8_t *rxBuffer, uint16_t size);
    void (*SPI_transmitReceive)(uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size);
    void (*nCS)(bool value);
    void (*nLDAC)(bool value);
    void (*nRESET)(bool value);
    void (*nCLR)(bool value);
    void (*TGL)(uint8_t value);
    bool (*nALMOUT)();

} DACx1416_HandleTypeDef;



void DACx1416_nCS(bool value);
void DACx1416_nLDAC(bool value);
void DACx1416_nRESET(bool value);
void DACx1416_nCLR(bool value);
void DACx1416_tgl(uint8_t value);
void DACx1416_SPI_transmit(uint8_t *txBuffer,uint16_t size);
void DACx1416_SPI_receive(uint8_t *rxBuffer,uint16_t size);
void DACx1416_SPI_transmitReceive(uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size);

void DACx1416_write_register_old(uint8_t address, uint16_t data);
uint16_t DACx1416_read_register_old(uint8_t address);



void DACx1416_write_register(DACx1416_HandleTypeDef *dac, DACx1416_register_t reg, uint16_t data);
uint16_t DACx1416_read_register(DACx1416_HandleTypeDef *dac, DACx1416_register_t reg);
DACx1416_deviceID_t DACx1416_get_device_id(DACx1416_HandleTypeDef *dac);
DACx1416_status_t DACx1416_get_status(DACx1416_HandleTypeDef *dac);
DACx1416_spiconfig_t DACx1416_get_spiConfig(DACx1416_HandleTypeDef *dac);
void DACx1416_set_spiConfig(DACx1416_HandleTypeDef *dac, DACx1416_spiconfig_t spiConfig);
DACx1416_genconfig_t DACx1416_get_genConfig(DACx1416_HandleTypeDef *dac);
void DACx1416_set_genConfig(DACx1416_HandleTypeDef *dac, DACx1416_genconfig_t genConfig);
DACx1416_brdConfig DACx1416_get_brdConfig(DACx1416_HandleTypeDef *dac);
void DACx1416_set_brdConfig(DACx1416_HandleTypeDef *dac, DACx1416_brdConfig brdConfig);
DACx1416_syncConfig DACx1416_get_syncConfig(DACx1416_HandleTypeDef *dac);
void DACx1416_set_syncConfig(DACx1416_HandleTypeDef *dac, DACx1416_syncConfig syncConfig);
DACx1416_toggConfig DACx1416_get_toggConfig(DACx1416_HandleTypeDef *dac);
void DACx1416_set_toggConfig(DACx1416_HandleTypeDef *dac, DACx1416_toggConfig toggConfig);
DACx1416_dacPwdwn DACx1416_get_pwdwn(DACx1416_HandleTypeDef *dac);
void DACx1416_set_pwdwn(DACx1416_HandleTypeDef *dac, DACx1416_dacPwdwn dacPwdwn);
DACx1416_dacRange DACx1416_get_dacRange(DACx1416_HandleTypeDef *dac);
void DACx1416_set_dacRange(DACx1416_HandleTypeDef *dac, DACx1416_dacRange dacRange);
void DACx1416_set_trigger(DACx1416_HandleTypeDef *dac, DACx1416_trigger_t triggerType);
void DACx1416_set_broadcast_value(DACx1416_HandleTypeDef *dac, uint16_t value);
void DACx1416_set_dac_value(DACx1416_HandleTypeDef *dac, DACx1416_DAC_t ch, uint16_t value);
void DACx1416_set_diff_offset(DACx1416_HandleTypeDef *dac, DACx1416_diff_offset diffOffset);


#endif /* INC_DACX1416_H_ */
