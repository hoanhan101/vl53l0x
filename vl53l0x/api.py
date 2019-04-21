#
# api.py contains a set of core functions
#

import smbus
import time
import register

class VL53L0X(object):
    def __init__(self):
        """Initialize a VL53L0X object"""
        # i2c device address
        self.address = register.VL53L0X_DEFAULT_ADDRESS

        # smbus object
        self.bus = smbus.SMBus(1)

        # static sequence config
        self.static_seq_config = 0

        # retry max loop
        self.max_loop = 0

        # retry polling delay in seconds
        self.polling_delay = 0.1

    def data_init(self):
        """TODO"""

        # set i2c standard mode
        write_byte(bus, 0x88, 0x00)

        # read whoami
        read_byte(bus, 0xC0)

        # use internal default settings
        write_byte(bus, 0x80, 0x01)
        write_byte(bus, 0xFF, 0x01)
        write_byte(bus, 0x00, 0x00)

        read_byte(bus, 0x91)

        write_byte(bus, 0x00, 0x01)
        write_byte(bus, 0xFF, 0x00)
        write_byte(bus, 0x80, 0x00)

        write_byte(bus, register.VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF)

    def static_init(self):
        """TODO"""
        return

    def perform_ref_calibration(self):
        """TODO"""

    def perform_ref_spad_management(self):
        """TODO"""
        return

    def perform_ref_signal_measurement(self):
        """TODO"""
        return

    def write_byte(self, reg, data):
        """TODO"""
        self.bus.write_byte_data(self.address, reg, data)

    def read_byte(self, reg):
        """TODO"""
        read = self.bus.read_byte_data(self.address, reg)

        return read

    def read_block(self, reg):
        """TODO"""
        read = self.bus.read_i2c_block_data(self.address, reg)

        return read

    def read_word(self, reg):
        """TODO"""
        raw = self.bus.read_i2c_block_data(self.address, reg)[:2]
        read = (raw[0] << 8) + raw[1]

        return read
