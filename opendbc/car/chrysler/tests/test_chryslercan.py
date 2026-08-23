import unittest

from opendbc.can import CANPacker
from opendbc.car.chrysler.chryslercan import create_susw_cruise_buttons


class TestChryslerCan(unittest.TestCase):
  def test_create_susw_cruise_buttons(self):
    packer = CANPacker("chrysler_susw")

    assert create_susw_cruise_buttons(packer, 8, 1) == (0x2FA, b"\x00\x08\x56\x00", 1)
    assert create_susw_cruise_buttons(packer, 8, 1, resume=True) == (0x2FA, b"\x08\x08\x0c\x00", 1)
    assert create_susw_cruise_buttons(packer, 8, 1, cancel=True) == (0x2FA, b"\x80\x08\x9f\x00", 1)
