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
        self.write_byte(0x88, 0x00)

        # read whoami
        self.read_byte(0xC0)

        # use internal default settings
        self.write_byte(0x80, 0x01)
        self.write_byte(0xFF, 0x01)
        self.write_byte(0x00, 0x00)

        self.read_byte(0x91)

        self.write_byte(0x00, 0x01)
        self.write_byte(0xFF, 0x00)
        self.write_byte(0x80, 0x00)

        self.write_byte(register.VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF)

    def static_init(self):
        """TODO"""
        self.write_byte(0xFF, 0x01)
        self.read_byte(0x84)
        self.write_byte(0xFF, 0x00)

        # read the sequence config and save it
        self.static_seq_config = self.read_byte(register.VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG)

    def perform_ref_calibration(self):
        """TODO"""
        self.perform_vhv_calibration()
        self.perform_phase_calibration()

        # restore static sequence config
        self.write_byte(register.VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, self.static_seq_config)

    def perform_vhv_calibration(self):
        """TODO"""
        # run vhv
        self.write_byte(register.VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x01)

        self.perform_single_ref_calibration(0x40)

        # read vhv from device
        self.ref_calibration_io(0xCB)

        # restore static sequence config
        self.write_byte(register.VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, self.static_seq_config)

    def perform_phase_calibration(self):
        """TODO"""
        # run phase cal
        self.write_byte(register.VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02)

        self.perform_single_ref_calibration(0x0)

        # read phase cal from device
        self.ref_calibration_io(bus, 0xEE)

        # restore static sequence config
        self.write_byte(register.VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, self.static_seq_config)

    def perform_single_ref_calibration(self, byte):
        """TODO"""
        self.write_byte(register.VL53L0X_REG_SYSRANGE_START, register.VL53L0X_REG_SYSRANGE_MODE_START_STOP | byte)

        self.measurement_poll_for_completion()

        self.write_byte(register.VL53L0X_REG_SYSRANGE_START, 0x00)

    def measurement_poll_for_completion(self):
        """TODO"""
        loop_nb = 0
        while loop_nb <= self.max_loop:
            data = self.get_measurement_data_ready()
            if data == 1:
                # TODO - measurement ready log
                break

            loop_nb += 1

            # polling delay
            time.sleep(self.polling_delay)

        # TODO - timeout error log

    def get_measurement_data_ready(self):
        """TODO"""
        status_reg = self.read_byte(register.VL53L0X_REG_RESULT_RANGE_STATUS)
        if status_reg & 0x01:
            return 1

        return 0

    def ref_calibration_io(self, byte):
        """TODO"""
        # read vhv from device
        self.write_byte(0xFF, 0x01)
        self.write_byte(0x00, 0x00)
        self.write_byte(0xFF, 0x00)

        self.read_byte(byte)

        self.write_byte(0xFF, 0x01)
        self.write_byte(0x00, 0x00)
        self.write_byte(0xFF, 0x00)

    def perform_ref_spad_management(self):
        """TODO"""
        self.write_byte(0xFF, 0x01)
        self.write_byte(register.VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00)
        self.write_byte(register.VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C)
        self.write_byte(0xFF, 0x00)
        self.write_byte(register.VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)
        self.write_byte(register.VL53L0X_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0)
        
        self.perform_ref_calibration()
        
        self.perform_ref_signal_measurement()

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
