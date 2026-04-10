import sliplib
import serial
import serial.tools.list_ports
from kingfisher_dac16 import DAC16
import time



dac = DAC16(manufacturer='emon.no')
dac = DAC16(description='KINGFISHER-DAC16')


dac.hardware_reset()


dac.DACx1416_spiConfig_set_crc_en(False)

productID = dac.DACx1416_get_product_id()
# Print both values in hexadecimal format
print(productID)

dac.DACx1416_spiConfig_set_dev_pwdwn(0)
dac.DACx1416_set_dac_pwdwn(0, 0)
dac.DACx1416_set_dacRange(0, dac.rangeSel.V0_VP5.value)
dac.DACx1416_dac_value(0, 32768)



exit()
