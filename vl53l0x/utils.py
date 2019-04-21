#
# utils.py contains a set of helper functions
#

def make_uint16(lsb, msb):
    """Make a meaningful uint16 from LSB and MSB"""
    return (msb << 8) + lsb
