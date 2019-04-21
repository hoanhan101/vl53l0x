#
# api.py contains a set of core functions
#

import smbus
import time

from register import *
from utils import *

class VL53L0X(object):
    def __init__(self):
        # i2c device address
        self.address = VL53L0X_DEFAULT_ADDRESS

        # smbus object
        self.bus = smbus.SMBus(1)

        # static sequence config
        self.static_seq_config = 0

        # retry max loop
        self.max_loop = 0

        # retry polling delay in seconds
        self.polling_delay = 0.1

        # measurement data cache
        self.measurement = 0

    def setup(self):
        self.data_init()
        self.static_init()
        self.perform_ref_calibration()
        self.perform_ref_spad_management()

    def measure(self):
        self.perform_ref_signal_measurement()

        return self.measurement

    def data_init(self):
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

        self.write_byte(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF)

    def static_init(self):
        self.write_byte(0xFF, 0x01)
        self.read_byte(0x84)
        self.write_byte(0xFF, 0x00)

        # read the sequence config and save it
        self.static_seq_config = self.read_byte(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG)

    def perform_ref_calibration(self):
        self.perform_vhv_calibration()
        self.perform_phase_calibration()

        # restore static sequence config
        self.write_byte(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, self.static_seq_config)

    def perform_vhv_calibration(self):
        # run vhv
        self.write_byte(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x01)

        self.perform_single_ref_calibration(0x40)

        # read vhv from device
        self.ref_calibration_io(0xCB)

        # restore static sequence config
        self.write_byte(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, self.static_seq_config)

    def perform_phase_calibration(self):
        # run phase cal
        self.write_byte(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02)

        self.perform_single_ref_calibration(0x0)

        # read phase cal from device
        self.ref_calibration_io(0xEE)

        # restore static sequence config
        self.write_byte(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, self.static_seq_config)

    def perform_single_ref_calibration(self, byte):
        self.write_byte(VL53L0X_REG_SYSRANGE_START, VL53L0X_REG_SYSRANGE_MODE_START_STOP | byte)

        self.measurement_poll_for_completion()

        self.write_byte(VL53L0X_REG_SYSRANGE_START, 0x00)

    def measurement_poll_for_completion(self):
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
        status_reg = self.read_byte(VL53L0X_REG_RESULT_RANGE_STATUS)
        if status_reg & 0x01:
            return 1

        return 0

    def ref_calibration_io(self, byte):
        # read vhv from device
        self.write_byte(0xFF, 0x01)
        self.write_byte(0x00, 0x00)
        self.write_byte(0xFF, 0x00)

        self.read_byte(byte)

        self.write_byte(0xFF, 0x01)
        self.write_byte(0x00, 0x00)
        self.write_byte(0xFF, 0x00)

    def perform_ref_spad_management(self):
        self.write_byte(0xFF, 0x01)
        self.write_byte(VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00)
        self.write_byte(VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C)
        self.write_byte(0xFF, 0x00)
        self.write_byte(VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)
        self.write_byte(VL53L0X_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0)

        self.perform_ref_calibration()

        self.perform_ref_signal_measurement()

    def perform_ref_signal_measurement(self):
        self.write_byte(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xC0)

        self.perform_single_ranging_measurement()

        self.write_byte(0xFF, 0x01)

        self.read_word(VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF)

        self.write_byte(0xFF, 0x00)

        # restore static sequence config
        self.write_byte(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, self.static_seq_config)

    def perform_single_ranging_measurement(self):
        self.perform_single_measurement()

        self.get_ranging_measurement_data()

    def perform_single_measurement(self):
        self.start_measurement()

        self.measurement_poll_for_completion()

    def start_measurement(self):
        self.write_byte(0x80, 0x01)
        self.write_byte(0xFF, 0x01)
        self.write_byte(0x00, 0x00)

        self.read_byte(0x91)

        self.write_byte(0x00, 0x01)
        self.write_byte(0xFF, 0x00)
        self.write_byte(0x80, 0x00)

        # device mode single ranging
        self.write_byte(VL53L0X_REG_SYSRANGE_START, 0x01)

        # wait until start bit has been cleared
        start_stop_byte = VL53L0X_REG_SYSRANGE_MODE_START_STOP
        tmp_byte = start_stop_byte
        loop_nb = 0

        while ((tmp_byte & start_stop_byte) == start_stop_byte) and (loop_nb < self.max_loop):
            if loop_nb > 0:
                tmp_byte = self.read_byte(VL53L0X_REG_SYSRANGE_START)

            loop_nb += 1

        # TODO - timeout error log

    def get_ranging_measurement_data(self):
        sysrange_status = self.read_byte(VL53L0X_REG_RESULT_RANGE_STATUS)
        if sysrange_status & 0x01:
            # TODO - measurement data ready log
            print("measurement data ready")
        else:
            return 0

        raw_data = self.read_block(0x14)

        # TODO - log these out
        range_millimeter = make_uint16(raw_data[11], raw_data[10])
        signal_rate = make_uint16(raw_data[7], raw_data[6])
        ambient_rate = make_uint16(raw_data[9], raw_data[8])
        effective_spad_rtn_count = make_uint16(raw_data[3], raw_data[2])
        device_range_status = raw_data[0]

        # update measurement
        self.measurement = range_millimeter

    def write_byte(self, reg, data):
        self.bus.write_byte_data(self.address, reg, data)

    def read_byte(self, reg):
        read = self.bus.read_byte_data(self.address, reg)

        return read

    def read_block(self, reg):
        read = self.bus.read_i2c_block_data(self.address, reg)

        return read

    def read_word(self, reg):
        raw = self.bus.read_i2c_block_data(self.address, reg)[:2]
        read = (raw[0] << 8) + raw[1]

        return read
