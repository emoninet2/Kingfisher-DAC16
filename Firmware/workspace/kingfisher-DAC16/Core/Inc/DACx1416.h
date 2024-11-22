/*
 * DACx1416.h
 *
 *  Created on: Oct 31, 2024
 *      Author: habiburrahman
 */

#ifndef INC_DACX1416_H_
#define INC_DACX1416_H_

#include "main.h"


#define DACx1416_DEVICE_ID_MASK		0x3FFF
#define DACx1416_DEVICE_ID_GP		2
#define DACx1416_VERSION_ID_MASK	0x0003

#define DACx1416_STATUS_CRC_ALARM_BP 2
#define DACx1416_STATUS_DAC_BUSY_BP 1
#define DACx1416_STATUS_TEMP_ALARM_BP 0

#define DACx1416_SPICONFIG_TEMPALM_EN_BP 11
#define DACx1416_SPICONFIG_DACBUSY_EN_BP 10
#define DACx1416_SPICONFIG_CRCALM_EN_BP 9
#define DACx1416_SPICONFIG_SOFTTOGGL_EN_BP 6
#define DACx1416_SPICONFIG_DEV_PWDWN_BP 5
#define DACx1416_SPICONFIG_CRC_EN_BP 4
#define DACx1416_SPICONFIG_STRM_EN_BP 3
#define DACx1416_SPICONFIG_SDO_EN_BP 2
#define DACx1416_SPICONFIG_FSDO_EN_BP 1

#define DACx1416_GENCONFIG_REF_PWDN_BP 				14
#define DACx1416_GENCONFIG_DAC_14_15_DIFF_EN_BP		7
#define DACx1416_GENCONFIG_DAC_12_13_DIFF_EN_BP		6
#define DACx1416_GENCONFIG_DAC_10_11_DIFF_EN_BP		5
#define DACx1416_GENCONFIG_DAC_8_9_DIFF_EN_BP		4
#define DACx1416_GENCONFIG_DAC_6_7_DIFF_EN_BP		3
#define DACx1416_GENCONFIG_DAC_4_5_DIFF_EN_BP		2
#define DACx1416_GENCONFIG_DAC_2_3_DIFF_EN_BP		1
#define DACx1416_GENCONFIG_DAC_0_1_DIFF_EN_BP		0

#define DACx1416_TRIG_ALM_RESET_BP 			8
#define DACx1416_TRIG_AB_TOG2_BP 			7
#define DACx1416_TRIG_AB_TOG1_BP 			6
#define DACx1416_TRIG_AB_TOG0_BP 			5
#define DACx1416_TRIG_LDAC_BP 				4
#define DACx1416_TRIG_SOFT_RESEET_GP 		0
#define DACx1416_TRIG_SOFT_RESEET_VALUE 	0b1010


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
	DACx1416_OK = 0,
	DACx1416_ERROR = -1
}DACx1416_error_t;



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
	DACx1416_DIFF_DAC_0_1,
	DACx1416_DIFF_DAC_2_3,
	DACx1416_DIFF_DAC_4_5,
	DACx1416_DIFF_DAC_6_7,
	DACx1416_DIFF_DAC_8_9,
	DACx1416_DIFF_DAC_10_11,
	DACx1416_DIFF_DAC_12_13,
	DACx1416_DIFF_DAC_14_15
}DACx1416_diffDAC_t;



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
	uint8_t crc_alarm;
	uint8_t dac_busy;
	uint8_t temp_alarm;
}DACx1416_status_t;

typedef struct{
	uint8_t tempalm_en;
	uint8_t dacbusy_en;
	uint8_t crcalm_en;
	uint8_t softtoggle_en;
	uint8_t dev_pwdwn;
	uint8_t crc_en;
	uint8_t str_en;
	uint8_t sdo_en;
	uint8_t fsdo;
}DACx1416_spiconfig_t;


typedef struct{
	uint8_t ref_pwdwn;
	uint8_t DAC_14_15_diff_en;
	uint8_t DAC_12_13_diff_en;
	uint8_t DAC_10_11_diff_en;
	uint8_t DAC_8_9_diff_en;
	uint8_t DAC_6_7_diff_en;
	uint8_t DAC_4_5_diff_en;
	uint8_t DAC_2_3_diff_en;
	uint8_t DAC_0_1_diff_en;
}DACx1416_genconfig_t;


typedef struct{
	uint8_t brdcast_en[16];
}DACx1416_brdConfig;


typedef struct{
	uint8_t sync_en[16];
}DACx1416_syncConfig;



typedef struct{
	DACx1416_toggleMode_t toggle_en[16];
}DACx1416_toggConfig;


typedef struct{
	uint8_t dac_pwdwn[16];
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
	DACx1416_TRIG_SOFT_RESET,
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
    void (*nCS)(uint8_t value);
    void (*nLDAC)(uint8_t value);
    void (*nRESET)(uint8_t value);
    void (*nCLR)(uint8_t value);
    void (*TGL)(uint8_t value);
    uint8_t (*nALMOUT)();
    uint8_t (*calculate_crc8)(uint8_t *data, uint32_t length);
} DACx1416_Port_HandleTypeDef;


typedef struct {
    // Function pointers for SPI operations
	DACx1416_Port_HandleTypeDef port;
	DACx1416_deviceID_t deviceID;
	DACx1416_status_t status;
	DACx1416_spiconfig_t spiConfig;
	DACx1416_genconfig_t genConfig;
	DACx1416_brdConfig brdcstConfig;
	DACx1416_syncConfig syncConfig;
	DACx1416_toggConfig toggConfig;
	DACx1416_dacPwdwn chPwrDwn;
	DACx1416_dacRange dacRange;
	DACx1416_diff_offset diffOffset;

} DACx1416;



DACx1416_error_t DACx1416_initialize(DACx1416 *dac);
DACx1416_error_t DACx1416_write_register(DACx1416 *dac, DACx1416_register_t addr, uint16_t *data);
DACx1416_error_t DACx1416_stream_write_registers(DACx1416 *dac, DACx1416_register_t addr, uint16_t *data, uint32_t len);
DACx1416_error_t DACx1416_read_register(DACx1416 *dac, DACx1416_register_t addr, uint16_t *data);

DACx1416_error_t DACx1416_get_device_id(DACx1416 *dac, DACx1416_deviceID_t * deviceID);
DACx1416_error_t DACx1416_get_status(DACx1416 *dac, DACx1416_status_t *status);
DACx1416_error_t DACx1416_get_spiConfig(DACx1416 *dac, DACx1416_spiconfig_t *spiConfig);
DACx1416_error_t DACx1416_get_genConfig(DACx1416 *dac, DACx1416_genconfig_t *genConfig);
DACx1416_error_t DACx1416_get_brdConfig(DACx1416 *dac, DACx1416_brdConfig *brdConfig);
DACx1416_error_t DACx1416_get_syncConfig(DACx1416 *dac, DACx1416_syncConfig *syncConfig);
DACx1416_error_t DACx1416_get_toggConfig(DACx1416 *dac, DACx1416_toggConfig *toggConfig);
DACx1416_error_t DACx1416_get_pwdwn(DACx1416 *dac, DACx1416_dacPwdwn *dacPwdwn);
DACx1416_error_t DACx1416_get_dacRange(DACx1416 *dac, DACx1416_dacRange *dacRange);

DACx1416_error_t DACx1416_set_spiConfig(DACx1416 *dac, DACx1416_spiconfig_t *spiConfig);
DACx1416_error_t DACx1416_set_genConfig(DACx1416 *dac, DACx1416_genconfig_t *genConfig);
DACx1416_error_t DACx1416_set_brdConfig(DACx1416 *dac, DACx1416_brdConfig *brdConfig);
DACx1416_error_t DACx1416_set_syncConfig(DACx1416 *dac, DACx1416_syncConfig *syncConfig);
DACx1416_error_t DACx1416_set_toggConfig(DACx1416 *dac, DACx1416_toggConfig *toggConfig);
DACx1416_error_t DACx1416_set_pwdwn(DACx1416 *dac, DACx1416_dacPwdwn *dacPwdwn);
DACx1416_error_t DACx1416_set_dacRange(DACx1416 *dac, DACx1416_dacRange *dacRange);
DACx1416_error_t DACx1416_set_trigger(DACx1416 *dac, DACx1416_trigger_t triggerType);
DACx1416_error_t DACx1416_set_broadcast_value(DACx1416 *dac, uint16_t *value);
DACx1416_error_t DACx1416_set_dac_value(DACx1416 *dac, DACx1416_DAC_t ch, uint16_t *dacValue);
DACx1416_error_t DACx1416_set_diff_offset(DACx1416 *dac, DACx1416_diff_offset *diffOffset);



DACx1416_error_t DACx1416_tempAlarmEnable(DACx1416 *dac, uint8_t enable);
DACx1416_error_t DACx1416_dacBusyEnable(DACx1416 *dac, uint8_t enable);
DACx1416_error_t DACx1416_crcAlarmEnable(DACx1416 *dac, uint8_t enable);
DACx1416_error_t DACx1416_softToggleEnable(DACx1416 *dac, uint8_t enable);
DACx1416_error_t DACx1416_devicePowerDown(DACx1416 *dac, uint8_t enable);
DACx1416_error_t DACx1416_crcEnable(DACx1416 *dac, uint8_t enable);
DACx1416_error_t DACx1416_streamEnable(DACx1416 *dac, uint8_t enable);
DACx1416_error_t DACx1416_sdoEnable(DACx1416 *dac, uint8_t enable);
DACx1416_error_t DACx1416_fsdoEnable(DACx1416 *dac, uint8_t enable);
DACx1416_error_t DACx1416_internalRefEnable(DACx1416 *dac, uint8_t enable);
DACx1416_error_t DACx1416_diffChannelEnable(DACx1416 *dac, DACx1416_diffDAC_t diffCh, uint8_t enable);
DACx1416_error_t DACx1416_channelBroadcastEnable(DACx1416 *dac, DACx1416_DAC_t Ch, uint8_t enable);
DACx1416_error_t DACx1416_channelsyncModeEnable(DACx1416 *dac, DACx1416_DAC_t Ch, uint8_t enable);
DACx1416_error_t DACx1416_channelToggleMode(DACx1416 *dac, DACx1416_DAC_t Ch, DACx1416_toggleMode_t mode);
DACx1416_error_t DACx1416_channelEnable(DACx1416 *dac, DACx1416_DAC_t Ch, uint8_t enable);
DACx1416_error_t DACx1416_channelRange(DACx1416 *dac,DACx1416_DAC_t Ch, DACx1416_dacRange_t range );
DACx1416_error_t DACx1416_softAlarmReset(DACx1416 *dac);
DACx1416_error_t DACx1416_softToggle(DACx1416 *dac, uint8_t toggleBit);
DACx1416_error_t DACx1416_softReset(DACx1416 *dac);
DACx1416_error_t DACx1416_broadcastValue(DACx1416 *dac, DACx1416_DAC_t Ch, uint16_t value);
DACx1416_error_t DACx1416_channelValue(DACx1416 *dac, DACx1416_DAC_t Ch, uint16_t value);
DACx1416_error_t DACx1416_diffChannelOffset(DACx1416 *dac, DACx1416_diffDAC_t diffCh, uint8_t value);






#endif /* INC_DACX1416_H_ */
