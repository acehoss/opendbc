import unittest

from opendbc.can import CANPacker
from opendbc.car.chrysler.chryslercan import create_susw_cruise_buttons


class TestChryslerCan(unittest.TestCase):
  def test_create_susw_cruise_buttons(self):
    packer = CANPacker("chrysler_susw")

    assert create_susw_cruise_buttons(packer, 8, 1) == (0x2FA, b"\x00\x08\x56\x00", 1)
    assert create_susw_cruise_buttons(packer, 8, 1, resume=True) == (0x2FA, b"\x08\x08\x0c\x00", 1)
    assert create_susw_cruise_buttons(packer, 8, 1, cancel=True) == (0x2FA, b"\x80\x08\x9f\x00", 1)

  def test_susw_body_signal_placement(self):
    packer = CANPacker("chrysler_susw")

    for gear, raw in enumerate((0x10, 0x20, 0x30, 0x40), start=1):
      assert packer.make_can_msg("GEAR", 0, {"PRNDL": gear})[1] == bytes([0, raw, 0, 0, 0, 0, 0, 0])

    assert packer.make_can_msg("DOORS", 0, {"DOOR_OPEN_FL": 1})[1] == b"\x01\x00\x00\x00\x00\x00\x00\x00"
    assert packer.make_can_msg("DOORS", 0, {"DOOR_OPEN_FR": 1})[1] == b"\x00\x80\x00\x00\x00\x00\x00\x00"
    assert packer.make_can_msg("BSM_1", 0, {"RIGHT_STATUS": 1})[1] == b"\x00\x10\x00\x00\x00\x00\x00\x00"
    assert packer.make_can_msg("BSM_1", 0, {"LEFT_STATUS": 1})[1] == b"\x00\x20\x00\x00\x00\x00\x00\x00"
    assert packer.make_can_msg("PARKING_BRAKE_STATUS", 0, {"PARKING_BRAKE_RELEASED": 1})[1] == b"\x08\x00\x00\x00\x00\x00\x00\x00"

    for state in range(6, 14):
      assert packer.make_can_msg("ACC_HUD", 0, {"ACC_GAP_LEAD_STATE": state})[1] == bytes([0, 0, 0, 0, 0, 0, state, 0])

    for state in range(4):
      assert packer.make_can_msg("STEERING_LEVERS", 0, {"TURN_SIGNALS": state})[1] == bytes([0, 0, state, 0])
