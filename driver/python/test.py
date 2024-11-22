import sliplib
import serial
import serial.tools.list_ports
from kingfisher_dac16 import DAC16


dac = DAC16(manufacturer='emon.no')
dac = DAC16(description='KINGFISHER-DAC16')

dac.DACx1416_use_CRC = False

productID = dac.DACx1416_get_product_id()
# Print both values in hexadecimal format
print(productID)

dac.DACx1416_spiConfig_set_dev_pwdwn(0)
dac.DACx1416_set_dac_pwdwn(0, 0)
dac.DACx1416_set_dacRange(0, dac.rangeSel.VN10_VP10.value)
dac.DACx1416_dac_value(0, 0x9656)



exit()


#dac.send_data([0x50, 0x7F, 0xF6])
#data = dac.receive_data()

data = dac.DACx1416_read_register(0x01)
print(hex(data))

data = dac.DACx1416_read_register(0x03)
print(hex(data))

dac.DACx1416_write_register(0x03, 0x0A84)
data = dac.DACx1416_read_register(0x03)
print(hex(data))


dac.DACx1416_write_register(0x09, 0xFFCF)
data = dac.DACx1416_read_register(0x09)
print(hex(data))

data = dac.DACx1416_read_register(0x03)
print(hex(data))




dac.DACx1416_write_register(16, 0x00FF)


data = dac.DACx1416_read_register(0x03)
print(hex(data))

data = dac.DACx1416_read_register(0x03)
print(hex(data))

data = dac.DACx1416_read_register(0x03)
print(hex(data))


print("-------------")

dac.DACx1416_write_register(0x03, 0x0A94)
data = dac.DACx1416_read_register(0x03, True)
print(hex(data))


data = dac.DACx1416_read_register(0x09, True)
print(hex(data))

dac.DACx1416_write_register(0x09, 0xABDD, True)
data = dac.DACx1416_read_register(0x09, True)
print(hex(data))



dac.close()

exit()






# Specify the serial port you are interested in
serial_port = '/dev/cu.usbmodem204F34AB41301'

# Find the port details
ports = serial.tools.list_ports.comports()

for port in ports:
    if port.device == serial_port:
        print("Device String:", port.device)
        print("Manufacturer:", port.manufacturer)
        print("Description:", port.description)
        print("Hardware ID:", port.hwid)
        break
else:
    print(f"Serial port {serial_port} not found.")
# Open the serial port using pyserial
ser = serial.Serial(serial_port, 9600)

# The data to send (as a bytearray)
data_to_send = bytearray([0x50, 0x1F, 0xF6])

# Encode the data using SLIP encoding
encoded_data = sliplib.encode(data_to_send)

# Send the encoded data over the serial connection
ser.write(encoded_data)

# Function to read a complete SLIP packet
def read_slip_packet(serial_connection):
    packet = bytearray()
    while True:
        byte = serial_connection.read(1)  # Read one byte at a time
        if byte == b'\xC0':  # SLIP END byte
            if packet:  # If we already have data, it's the end of the packet
                break
            else:
                # If we receive an END byte with no data, it might be the start
                # of a new packet; continue reading.
                continue
        packet.extend(byte)  # Append byte to the packet
    return packet

# Read a SLIP packet from the serial connection
received_encoded_data = read_slip_packet(ser)

# Decode the received SLIP data
decoded_data = sliplib.decode(received_encoded_data)

# Print the received (decoded) data
print("Decoded received data:", decoded_data)

# Close the serial connection when done
ser.close()