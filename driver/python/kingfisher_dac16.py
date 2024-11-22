import sliplib
import serial
import serial.tools.list_ports
from enum import Enum

class DAC16:
    class register(Enum):
        NOP = 0x00
        DEVICE_ID = 0x01
        STATUS = 0x02
        SPICONFIG = 0x03
        GENCONFIG = 0x04
        BRDCONFIG = 0x05
        SYNCCONFIG = 0x06
        TOGGCONFIG = 0x07
        DACPWDWN = 0x09
        DACRANGE = 0x0A
        TRIGGER = 0x0E
        BRDCAST = 0x0F
        DACn = 0x10
        OFFSET = 0x20

    class toggConfigSel(Enum):
        TOGGLE_MODE_DISABLED = 0b00
        TOGGLE_MODE_ENABLED_TOGGLE0 = 0b01
        TOGGLE_MODE_ENABLED_TOGGLE1 = 0b10
        TOGGLE_MODE_ENABLED_TOGGLE2 = 0b11

    class rangeSel(Enum):
        V0_VP5 = 0b0000
        V0_VP10 = 0b0001
        V0_VP20 = 0b0010
        V0_VP40 = 0b0100
        VN5_VP5 = 0b1001
        VN10_VP10 = 0b1010
        VN20_VP20 = 0b1100
        VN2_5_VP2_5 = 0b1110

    class diffChannel(Enum):
        diff_dac_en_14_15 = 7
        diff_dac_en_12_13 = 6
        diff_dac_en_10_11 = 5
        diff_dac_en_8_9 = 4
        diff_dac_en_6_7 = 3
        diff_dac_en_4_5 = 2
        diff_dac_en_2_3 = 1
        diff_dac_en_0_1 = 0

    class trigger(Enum):
        alm_reset = 8
        ab_tog2 = 7
        ab_tog1 = 6
        ab_tog0 = 5
        ldac = 4
        soft_reset = 0
    
    def __init__(self, target_device_string=None, manufacturer=None, description=None):
        """
        Initialize the DAC16 class and connect to a specific device.
        :param target_device_string: The device string of the desired port (e.g., '/dev/cu.usbmodem204F34AB41301').
        :param manufacturer: Manufacturer name to match (optional).
        :param description: Description string to match (optional).
        """
        self.serial_port = None  # Placeholder for the serial port instance
        self.port_info = None    # Placeholder for port information

        # List all available ports
        ports = serial.tools.list_ports.comports()
        if not ports:
            raise Exception("No serial ports found.")

        # Match the target port based on provided criteria
        for port in ports:
            if (target_device_string and port.device == target_device_string) or \
               (manufacturer and port.manufacturer == manufacturer) or \
               (description and port.description == description):
                self.port_info = port
                self.serial_port = serial.Serial(port.device, baudrate=9600, timeout=1)
                print(f"Connected to port: {port.device}")
                break
        else:
            raise Exception("No matching device found with the given criteria.")

        # Print port details
        self.print_port_details()
        self.DACx1416_use_CRC = False
        self.brdcast_en = [None] * 16  # Create a list with 25 None values
        self.sync_en = [None] * 16
        self.dac_ab_togg_en = [self.toggConfigSel.TOGGLE_MODE_DISABLED] * 16 
        self.dac_pwdwn = [None] * 16 
        self.dac_range = [self.rangeSel.V0_VP5] * 16

    def print_port_details(self):
        """
        Print details of the selected serial port.
        """
        if self.port_info:
            print("Device String:", self.port_info.device)
            print("Manufacturer:", self.port_info.manufacturer)
            print("Description:", self.port_info.description)
            print("Hardware ID:", self.port_info.hwid)
        else:
            print("No port selected.")

    def send_data(self, data):
        """
        Send SLIP-encoded data to the device.

        :param data: Bytearray or bytes to be sent.
        """
        if not self.serial_port:
            raise Exception("Serial port not initialized.")
        
        encoded_data = sliplib.encode(data)
        self.serial_port.write(encoded_data)
        #print(f"Sent: {encoded_data}")

    def receive_data(self):
        """
        Receive and decode a complete SLIP-encoded packet from the device.

        :return: Decoded data from the SLIP packet.
        """
        if not self.serial_port:
            raise Exception("Serial port not initialized.")

        packet = bytearray()
        while True:
            byte = self.serial_port.read(1)  # Read one byte at a time
            if not byte:
                raise Exception("Timeout or no data received.")
            
            if byte == b'\xC0':  # SLIP END byte
                if packet:  # If we already have data, it's the end of the packet
                    break
                else:
                    # If we receive an END byte with no data, it might be the start
                    # of a new packet; continue reading.
                    continue
            packet.extend(byte)  # Append byte to the packet

        # Decode the SLIP packet
        decoded_data = sliplib.decode(packet)
        #print(f"Received packet: {decoded_data}")
        return decoded_data

    def close(self):
        """
        Close the serial port connection.
        """
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Serial port closed.")

    def calculate_crc(self, data):
        # Compute the CRC-8 using the zlib library with polynomial 0x07
        crc = 0
        polynomial = 0x07
        for byte in data:
            crc ^= byte  # XOR byte into the least significant byte of CRC
            for _ in range(8):  # Process 8 bits
                if (crc & 0x80):  # If the MSB is set
                    crc = (crc << 1) ^ polynomial
                else:
                    crc <<= 1
                crc &= 0xFF  # Ensure CRC remains 8-bit
        return crc
        
    

    def DACx1416_read_register(self, register_address, use_crc=False):
        # Prepare the base command with the read bit (7th bit set)
        command = register_address | (1 << 7)

        if not use_crc:
            # Send the basic read command without CRC
            self.send_data(bytearray([command]))
            retVal = self.receive_data()

            # Extract and return the register value
            return (retVal[1] << 8) | retVal[2]
        else:
            # Calculate CRC for the transmitted command
            crc8_calculated = self.calculate_crc(bytearray([command, 0xFF, 0xFF]))
            self.send_data(bytearray([command, crc8_calculated]))
            retVal = self.receive_data()

            # Extract returned values
            byte0, byte1, byte2, crc8_received = retVal[0], retVal[1], retVal[2], retVal[3]

            # Recalculate CRC for the received data
            crc8_calculated = self.calculate_crc(bytearray([byte0, byte1, byte2]))

            # Validate CRC
            if crc8_calculated != crc8_received:
                raise ValueError("CRC mismatch: Data integrity check failed.")

            # Return the 16-bit register value
            return (byte1 << 8) | byte2


    def DACx1416_write_register(self, register_address, value, use_crc=False):
        """
        Write a 16-bit value to a specified register.

        :param register_address: Address of the register to write to (6 bits).
        :param value: 16-bit value to write.
        :param use_crc: Whether to include CRC for error checking.
        """
        # Prepare the base command and data bytes
        byte0 = register_address & 0x3F  # Ensure only 6 bits are used
        byte1 = (value >> 8) & 0xFF      # High byte of the value
        byte2 = value & 0xFF             # Low byte of the value
        # Create the base data packet
        data_packet = bytearray([byte0, byte1, byte2])

        if use_crc:
            # Calculate and append CRC byte
            crc_byte = self.calculate_crc(data_packet)
            data_packet.append(crc_byte)

        # Send the data packet
        self.send_data(data_packet)


    def DACx1416_CRC_Mode(self, enable=False):
        pass

    def DACx1416_get_product_id(self):
        val = self.DACx1416_read_register(self.register.DEVICE_ID.value, self.DACx1416_use_CRC )
        productID = (val >> 2) & 0x3FFF
        versionID = val &0x03
        return productID, versionID


    def DACx1416_get_status_crc_alm(self):
        val = self.DACx1416_read_register(self.register.STATUS.value, self.DACx1416_use_CRC )
        crc_alm = (val >> 2) & 0x01
        return crc_alm

    def DACx1416_get_status_dac_busy(self):
        val = self.DACx1416_read_register(self.register.STATUS.value, self.DACx1416_use_CRC )
        dac_busy = (val >> 1) & 0x01
        return dac_busy

    def DACx1416_get_status_temp_alarm(self):
        val = self.DACx1416_read_register(self.register.STATUS.value, self.DACx1416_use_CRC )
        temp_alm = (val) & 0x01
        return temp_alm

    def DACx1416_spiConfig_get_temp_alarm(self):
        spiConfigVal = self.DACx1416_read_register(self.register.SPICONFIG.value, self.DACx1416_use_CRC)
        print(hex(spiConfigVal))
        return (spiConfigVal >> 11) & 0x01

    def DACx1416_spiConfig_set_temp_alarm(self, value):
        spiConfigVal = self.DACx1416_read_register(self.register.SPICONFIG.value, self.DACx1416_use_CRC)
        spiConfigVal &= ~(1 << 11)
        spiConfigVal |= (value & 0x01) << 11
        self.DACx1416_write_register(self.register.SPICONFIG.value, spiConfigVal, self.DACx1416_use_CRC)

    def DACx1416_spiConfig_get_dacbusy_en(self):
        spiConfigVal = self.DACx1416_read_register(self.register.SPICONFIG.value, self.DACx1416_use_CRC)
        return (spiConfigVal >> 10) & 0x01

    def DACx1416_spiConfig_set_dacbusy_en(self, value):
        spiConfigVal = self.DACx1416_read_register(self.register.SPICONFIG.value, self.DACx1416_use_CRC)
        spiConfigVal &= ~(1 << 10)
        spiConfigVal |= (value & 0x01) << 10
        self.DACx1416_write_register(self.register.SPICONFIG.value, spiConfigVal, self.DACx1416_use_CRC)

    def DACx1416_spiConfig_get_crcalm_en(self):
        spiConfigVal = self.DACx1416_read_register(self.register.SPICONFIG.value, self.DACx1416_use_CRC)
        return (spiConfigVal >> 9) & 0x01

    def DACx1416_spiConfig_set_crcalm_en(self, value):
        spiConfigVal = self.DACx1416_read_register(self.register.SPICONFIG.value, self.DACx1416_use_CRC)
        spiConfigVal &= ~(1 << 9)
        spiConfigVal |= (value & 0x01) << 9
        self.DACx1416_write_register(self.register.SPICONFIG.value, spiConfigVal, self.DACx1416_use_CRC)

    def DACx1416_spiConfig_get_softtoggle_en(self):
        spiConfigVal = self.DACx1416_read_register(self.register.SPICONFIG.value, self.DACx1416_use_CRC)
        return (spiConfigVal >> 6) & 0x01

    def DACx1416_spiConfig_set_softtoggle_en(self, value):
        spiConfigVal = self.DACx1416_read_register(self.register.SPICONFIG.value, self.DACx1416_use_CRC)
        spiConfigVal &= ~(1 << 6)
        spiConfigVal |= (value & 0x01) << 6
        self.DACx1416_write_register(self.register.SPICONFIG.value, spiConfigVal, self.DACx1416_use_CRC)

    def DACx1416_spiConfig_get_dev_pwdwn(self):
        spiConfigVal = self.DACx1416_read_register(self.register.SPICONFIG.value, self.DACx1416_use_CRC)
        return (spiConfigVal >> 5) & 0x01

    def DACx1416_spiConfig_set_dev_pwdwn(self, value):
        spiConfigVal = self.DACx1416_read_register(self.register.SPICONFIG.value, self.DACx1416_use_CRC)
        spiConfigVal &= ~(1 << 5)
        spiConfigVal |= (value & 0x01) << 5
        self.DACx1416_write_register(self.register.SPICONFIG.value, spiConfigVal, self.DACx1416_use_CRC)

    def DACx1416_spiConfig_get_crc_en(self):
        spiConfigVal = self.DACx1416_read_register(self.register.SPICONFIG.value, self.DACx1416_use_CRC)
        return (spiConfigVal >> 4) & 0x01

    def DACx1416_spiConfig_set_crc_en(self, value):
        spiConfigVal = self.DACx1416_read_register(self.register.SPICONFIG.value, self.DACx1416_use_CRC)
        spiConfigVal &= ~(1 << 4)
        spiConfigVal |= (value & 0x01) << 4
        self.DACx1416_write_register(self.register.SPICONFIG.value, spiConfigVal, self.DACx1416_use_CRC)

    def DACx1416_spiConfig_get_str_en(self):
        spiConfigVal = self.DACx1416_read_register(self.register.SPICONFIG.value, self.DACx1416_use_CRC)
        return (spiConfigVal >> 3) & 0x01

    def DACx1416_spiConfig_set_str_en(self, value):
        spiConfigVal = self.DACx1416_read_register(self.register.SPICONFIG.value, self.DACx1416_use_CRC)
        spiConfigVal &= ~(1 << 3)
        spiConfigVal |= (value & 0x01) << 3
        self.DACx1416_write_register(self.register.SPICONFIG.value, spiConfigVal, self.DACx1416_use_CRC)

    def DACx1416_spiConfig_get_sdo_en(self):
        spiConfigVal = self.DACx1416_read_register(self.register.SPICONFIG.value, self.DACx1416_use_CRC)
        return (spiConfigVal >> 2) & 0x01

    def DACx1416_spiConfig_set_sdo_en(self, value):
        spiConfigVal = self.DACx1416_read_register(self.register.SPICONFIG.value, self.DACx1416_use_CRC)
        spiConfigVal &= ~(1 << 2)
        spiConfigVal |= (value & 0x01) << 2
        self.DACx1416_write_register(self.register.SPICONFIG.value, spiConfigVal, self.DACx1416_use_CRC)

    def DACx1416_spiConfig_get_fsdo(self):
        spiConfigVal = self.DACx1416_read_register(self.register.SPICONFIG.value, self.DACx1416_use_CRC)
        return (spiConfigVal >> 1) & 0x01

    def DACx1416_spiConfig_set_fsdo(self, value):
        spiConfigVal = self.DACx1416_read_register(self.register.SPICONFIG.value, self.DACx1416_use_CRC)
        spiConfigVal &= ~(1 << 1)
        spiConfigVal |= (value & 0x01) << 1
        self.DACx1416_write_register(self.register.SPICONFIG.value, spiConfigVal, self.DACx1416_use_CRC)
            

    
    def DACx1416_get_ref_pwdwn(self):
        return ((self.DACx1416_read_register(self.register.GENCONFIG.value, self.DACx1416_use_CRC) >> 14) & 0x01)

    
    def DACx1416_set_ref_pwdwn(self, ref_pwdwn):

        genConfigVal = self.DACx1416_read_register(self.register.GENCONFIG.value, self.DACx1416_use_CRC)
        genConfigVal = (genConfigVal & ~(1 << 14)) | ((ref_pwdwn & 0x01) << 14)
        self.DACx1416_write_register(self.register.GENCONFIG.value, genConfigVal, self.DACx1416_use_CRC)


    def DACx1416_get_diff_ch_enable(self, diffChannel):
        genConfigVal = self.DACx1416_read_register(self.register.GENCONFIG.value, self.DACx1416_use_CRC)
        return (genConfigVal>>diffChannel) &0b1
    
    def DACx1416_set_diff_ch_enable(self, diffChannel, value):
        genConfigVal = self.DACx1416_read_register(self.register.GENCONFIG.value, self.DACx1416_use_CRC)

        genConfigVal &= ~(1<<diffChannel)
        genConfigVal |= (value << diffChannel)
        self.DACx1416_write_register(self.register.GENCONFIG.value, genConfigVal, self.DACx1416_use_CRC)

    
    def DACx1416_get_broadcast_config(self, channel):
        """
        Reads and decodes the broadcast configuration register (BRDCONFIG).

        :return: A dictionary containing the decoded broadcast configuration fields for all channels.
        """
        brdcstConfig = self.DACx1416_read_register(self.register.BRDCONFIG.value, self.DACx1416_use_CRC)
        return (brdcstConfig >> channel) &0b1
       

    def DACx1416_set_broadcast_config(self, channel, value):

        # Read the current BRDCONFIG register value
        brdcstConfig = self.DACx1416_read_register(self.register.BRDCONFIG.value, self.DACx1416_use_CRC)
        brdcstConfig &= ~(1<<channel)
        brdcstConfig |= (value<<channel)
        # Write the updated value back to the BRDCONFIG register
        self.DACx1416_write_register(self.register.BRDCONFIG.value, brdcstConfig, self.DACx1416_use_CRC)

    def DACx1416_get_syncConfig(self, channel):
        """
        Get the sync configuration status for sync channels.

        :return: A dictionary with the sync status for each channel.
        """
        # Read the current SYNCCONFIG register value
        syncConfigVal = self.DACx1416_read_register(self.register.SYNCCONFIG.value, self.DACx1416_use_CRC)

        return (syncConfigVal>>channel) & 0b1
    

    def DACx1416_set_syncConfig(self, channel, value):
        # Read the current SYNCCONFIG register value
        syncConfigVal = self.DACx1416_read_register(self.register.SYNCCONFIG.value, self.DACx1416_use_CRC)
        syncConfigVal &= ~(1<<channel)
        syncConfigVal |= (value << channel)
        self.DACx1416_write_register(self.register.SYNCCONFIG.value, syncConfigVal, self.DACx1416_use_CRC)


    def DACx1416_get_toggleConfig(self, channel):
        """
        Gets the toggle configuration for a specific channel.

        :param channel: The DAC channel (0 to 15).
        :return: The 2-bit toggle configuration value for the specified channel (0, 1, 2, or 3).
        """
        # Ensure the channel is within the valid range
        if channel not in range(16):
            raise ValueError("Invalid channel: must be between 0 and 15.")
        
        # Determine which register to read based on the channel (0-7 or 8-15)
        register_offset = channel // 8  # 0 for channels 0-7, 1 for channels 8-15
        bit_position = (channel % 8) * 2  # The bit position within the register (2 bits per channel)

        # Read the appropriate toggle configuration register
        toggleConfigVal = self.DACx1416_read_register(self.register.TOGGCONFIG.value + register_offset, self.DACx1416_use_CRC)

        # Extract the 2 bits corresponding to the channel
        toggle_value = (toggleConfigVal >> bit_position) & 0b11  # Mask to get the 2 bits

        return toggle_value


    def DACx1416_set_toggleConfig(self, channel, value):
        """
        Sets the toggle configuration for a specific channel.
        
        :param channel: The DAC channel (0 to 15).
        :param value: The value to set for the toggle configuration (0 or 1).
        """
        if value not in [0, 1]:
            raise ValueError("Invalid value: must be 0 or 1.")
        
        # Determine which register to use based on the channel number
        register_offset = channel // 8  # 0 for channels 0-7, 1 for channels 8-15
        bit_position = (channel % 8) * 2  # The bit position in the register (2 bits per channel)

        # Read the current value of the appropriate TOGGCONFIG register
        toggConfigVal = self.DACx1416_read_register(self.register.TOGGCONFIG.value + register_offset, self.DACx1416_use_CRC)

        # Clear the bits for the current channel
        toggConfigVal &= ~(0b11 << bit_position)

        # Set the new value for the current channel
        toggConfigVal |= (value << bit_position)

        # Write the updated value back to the register
        self.DACx1416_write_register(self.register.TOGGCONFIG.value + register_offset, toggConfigVal, self.DACx1416_use_CRC)


    def DACx1416_get_dac_pwdwn(self, channel):
        """
        Reads the DAC power-down register (DACPWDWN) and decodes the power-down status for each DAC channel.
        
        :return: A dictionary where the keys are 'dacpwd_X' (where X is the channel number) and the values are 0 or 1,
                indicating whether the DAC channel is in power-down mode (1) or active (0).
        """
        # Read the DACPWDWN register value
        register_value = self.DACx1416_read_register(self.register.DACPWDWN.value, self.DACx1416_use_CRC)

        return (register_value >> channel) & 0b1
    


    def DACx1416_set_dac_pwdwn(self, channel, powerDown):
        register_value = self.DACx1416_read_register(self.register.DACPWDWN.value , self.DACx1416_use_CRC)

        register_value &= ~(1<<channel)
        register_value |= (powerDown) << channel

        self.DACx1416_write_register(self.register.DACPWDWN.value, register_value, self.DACx1416_use_CRC)


    def DACx1416_set_dacRange(self, channel, value):
        # Map channel to (register offset, bitmask)
        channel_map = {
            15: (0, 0xF000, 12), 14: (0, 0x0F00, 8), 13: (0, 0x00F0, 4), 12: (0, 0x000F, 0),
            11: (1, 0xF000, 12), 10: (1, 0x0F00, 8), 9:  (1, 0x00F0, 4), 8:  (1, 0x000F, 0),
            7:  (2, 0xF000, 12), 6:  (2, 0x0F00, 8), 5:  (2, 0x00F0, 4), 4:  (2, 0x000F, 0),
            3:  (3, 0xF000, 12), 2:  (3, 0x0F00, 8), 1:  (3, 0x00F0, 4), 0:  (3, 0x000F, 0)
        }

        # Ensure the channel is valid
        if channel not in channel_map:
            raise ValueError(f"Invalid channel: {channel}")

        # Get the register offset, bitmask, and bit shift for the given channel
        reg_offset, mask, shift = channel_map[channel]
        
        # Read the register value
        register_value = self.DACx1416_read_register(self.register.DACRANGE.value + reg_offset, self.DACx1416_use_CRC)
        
        # Clear the relevant bits and set the new value
        register_value = (register_value & ~mask) | ((value & 0xF) << shift)
        
        # Write the modified value back to the register
        self.DACx1416_write_register(self.register.DACRANGE.value + reg_offset, register_value, self.DACx1416_use_CRC)

    def DACx1416_clear_trigger(self, trigger):
        # Validate trigger input
        if trigger not in [self.trigger.alm_reset, self.trigger.ab_tog2, self.trigger.ab_tog1, 
                            self.trigger.ab_tog0, self.trigger.ldac, self.trigger.soft_reset]:
            raise ValueError(f"Invalid trigger: {trigger}")

        # Define a dictionary for triggers and their bit shifts (if needed)
        trigger_map = {
            self.trigger.alm_reset: self.trigger.alm_reset.value,
            self.trigger.ab_tog2: self.trigger.ab_tog2.value,
            self.trigger.ab_tog1: self.trigger.ab_tog1.value,
            self.trigger.ab_tog0: self.trigger.ab_tog0.value,
            self.trigger.ldac: self.trigger.ldac.value,
            self.trigger.soft_reset: (0b1010 << self.trigger.soft_reset.value)  # Special case for soft_reset
        }

        # Perform the write operation
        trigger_bit = trigger_map[trigger]
        self.DACx1416_write_register(self.register.TRIGGER.value, (1 << trigger_bit), self.DACx1416_use_CRC)


    def DACx1416_brdcast_value(self, value):
        self.DACx1416_write_register(self.register.BRDCAST.value, value, self.DACx1416_use_CRC)


    def DACx1416_dac_value(self, channel, value):
        self.DACx1416_write_register(self.register.DACn.value + channel, value, self.DACx1416_use_CRC)