#
# api.py contains a set of main functions
#

import smbus
import time
import registers

class VL53L0X(object):
    def __init__(self):
        """
        Initialize a VL53L0X object.
        """
        # smbus object to communicate via I2C
        self.bus = smbus.SMBus(1)

        # static sequence config
        self.static_seq_config = 0


        # retry max loop
        self.max_loop = 0

        # retry polling delay in seconds
        self.polling_delay = 0.1

    def data_init(self):
        """
        TODO
        """
        return

    def static_init(self):
        """
        TODO
        """
        return

    def perform_ref_calibration(self):
        """
        TODO
        """

    def perform_ref_spad_management(self):
        """
        TODO
        """
        return

    def perform_ref_signal_measurement(self):
        """
        TODO
        """
        return
