"""
Example bring-up for KINGFISHER-DAC16 over USB (SLIP).

Prerequisites: matching firmware on the MCU, device drivers installed, no other
program holding the serial port.
"""
from kingfisher_dac16 import DAC16

# Match by USB description or manufacturer (see serial.tools.list_ports).
dac = DAC16(description="KINGFISHER-DAC16")
# dac = DAC16(manufacturer="emon.no")

# Pulse DAC nRESET; optional host settle is handled inside hardware_reset().
dac.hardware_reset()

# SPI CRC on the DAC chip (SPICONFIG.CRC-EN) must match DACx1416_use_CRC on
# every transaction. If you enable CRC in hardware but leave this False (or
# the reverse), communication fails. A hardware reset returns CRC-EN to default.
dac.DACx1416_spiConfig_set_crc_en(False)

# Sanity check: read device ID. Product ID is typically 668 (see datasheet).
product_id = dac.DACx1416_get_product_id()
print(product_id)

# Take the DAC out of device power-down so outputs can drive.
dac.DACx1416_spiConfig_set_dev_pwdwn(False)

channel = 0

# Per-channel power-down: 0 = active.
dac.DACx1416_set_dac_pwdwn(channel, 0)

# Output span: DACx1416_set_dacRange(channel, dac.rangeSel.<member>.value).
# +----------------------+---------------------------+
# | rangeSel member      | Nominal output span       |
# +----------------------+---------------------------+
# | V0_VP5               | 0 V ... +5 V              |
# | V0_VP10              | 0 V ... +10 V             |
# | V0_VP20              | 0 V ... +20 V             |
# | V0_VP40              | 0 V ... +40 V             |
# | VN5_VP5              | -5 V ... +5 V             |
# | VN10_VP10            | -10 V ... +10 V           |
# | VN20_VP20            | -20 V ... +20 V           |
# | VN2_5_VP2_5          | -2.5 V ... +2.5 V         |
# +----------------------+---------------------------+
# Unipolar (V0_VP*): tie DAC_VSS to board GND. Bipolar ranges need a negative rail.
dac.DACx1416_set_dacRange(channel, dac.rangeSel.V0_VP5.value)

# Load DAC data (16-bit code). Example: 32768 is mid-scale for a 16-bit code;
# Since dac range is V0_VP5, the dac value of 32768 corresponds to 2.5V.
dac.DACx1416_dac_value(channel, 32768)




dac.close()
exit()
