#!/usr/bin/env python3
import unittest
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, make_msg

# private fusion bus, a gateway copies raw CAN C messages onto it
GATEWAY_BUS = 1

# real CRUISE_BUTTONS frames captured on the car: no button, resume, cancel
CRUISE_BUTTONS_FRAMES = (b"\x00\x08\x56\x00", b"\x08\x08\x0c\x00", b"\x80\x08\x9f\x00")


def corrupt_checksum(msg):
  addr, dat, bus = msg
  return addr, dat[:-1] + bytes([dat[-1] ^ 0xFF]), bus


class TestChryslerSuswSafety(common.CarSafetyTest, common.MotorTorqueSteeringSafetyTest):
  TX_MSGS = [[0x1F6, 0]]
  STANDSTILL_THRESHOLD = 0
  RELAY_MALFUNCTION_ADDRS = {0: (0x1F6,)}
  FWD_BLACKLISTED_ADDRS = {2: [0x1F6]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  MAX_RATE_UP = 4
  MAX_RATE_DOWN = 4
  MAX_TORQUE_LOOKUP = [0], [250]
  MAX_RT_DELTA = 150
  MAX_TORQUE_ERROR = 80

  def setUp(self):
    self.packer = CANPackerSafety("chrysler_susw")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.chryslerSusw, 0)
    self.safety.init_tests()

  def _pcm_status_msg(self, enable):
    values = {"ACC_ENGAGED": 1 if enable else 0}
    return self.packer.make_can_msg_safety("ACC_STATUS_1", GATEWAY_BUS, values)

  def _speed_msg(self, speed):
    values = {"VEHICLE_SPEED": speed}
    return self.packer.make_can_msg_safety("ABS_6", 0, values)

  def _user_gas_msg(self, gas):
    values = {"ACCEL_PEDAL": gas}
    return self.packer.make_can_msg_safety("ENGINE_1", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PEDAL_SWITCH": 1 if brake else 0}
    return self.packer.make_can_msg_safety("ABS_3", 0, values)

  def _torque_meas_msg(self, torque):
    values = {"TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("EPS_2", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"STEERING_TORQUE": torque, "LKAS_CONTROL_BIT": steer_req}
    return self.packer.make_can_msg_safety("LKAS_COMMAND", 0, values)

  def _button_msg(self, cancel=False, resume=False):
    values = {"ACC_CANCEL": cancel, "ACC_RESUME": resume}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", GATEWAY_BUS, values)

  def test_rx_hook(self):
    for count in range(20):
      self.assertTrue(self._rx(self._speed_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._user_brake_msg(False)), f"{count=}")
      self.assertTrue(self._rx(self._torque_meas_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._user_gas_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._pcm_status_msg(False)), f"{count=}")
      self.assertTrue(self._rx(self._button_msg()), f"{count=}")

  def test_no_button_spam(self):
    # openpilot never sends cruise buttons on this car, the stock ACC is left alone
    for bus in range(4):
      for values in ({"ACC_CANCEL": 1}, {"ACC_RESUME": 1}, {}):
        for controls_allowed in (True, False):
          self.safety.set_controls_allowed(controls_allowed)
          self.assertFalse(self._tx(self.packer.make_can_msg_safety("CRUISE_BUTTONS", bus, values)))

  def test_lka_hud_forwarded(self):
    # LKA_HUD_2 is the camera's cluster message, it is not blocked in either direction
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x547))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x547))

  def test_lkas_command_blocked_from_camera(self):
    # the stock camera's LKAS_COMMAND must never reach the EPS
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1F6))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x1F6))

  def test_gateway_bus_not_forwarded(self):
    # nothing is forwarded to or from the private fusion bus
    for addr in self.SCANNED_ADDRS:
      self.assertEqual(-1, self.safety.safety_fwd_hook(GATEWAY_BUS, addr), f"{addr=:#x}")
      for bus in range(4):
        self.assertNotEqual(GATEWAY_BUS, self.safety.safety_fwd_hook(bus, addr), f"{addr=:#x} {bus=}")

  def test_cruise_buttons_checksum(self):
    # captured frames, the checksum is in byte 2 and only covers bytes 0-1
    for dat in CRUISE_BUTTONS_FRAMES:
      self._reset_safety_hooks()
      self.assertTrue(self._rx(make_msg(GATEWAY_BUS, 0x2FA, dat=dat)), dat.hex())

      # the trailing padding byte is not covered by the checksum
      self.assertTrue(self._rx(make_msg(GATEWAY_BUS, 0x2FA, dat=dat[:3] + b"\xff")), dat.hex())

      # a corrupted checksum is rejected and drops controls
      bad = bytearray(dat)
      bad[2] ^= 0xFF
      self.safety.set_controls_allowed(True)
      self.assertFalse(self._rx(make_msg(GATEWAY_BUS, 0x2FA, dat=bytes(bad))), bad.hex())
      self.assertFalse(self.safety.get_controls_allowed())

  def test_acc_status_1_checksum(self):
    self.safety.set_controls_allowed(True)
    msg = self.packer.make_can_msg_safety("ACC_STATUS_1", GATEWAY_BUS, {"ACC_ENGAGED": 1}, fix_checksum=corrupt_checksum)
    self.assertFalse(self._rx(msg))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_acc_status_1_counter(self):
    # a stuck counter eventually invalidates the message and drops controls
    for i in range(common.MAX_WRONG_COUNTERS + 1):
      self.safety.set_controls_allowed(True)
      values = {"ACC_ENGAGED": 1, "COUNTER": 0}
      valid = self._rx(self.packer.make_can_msg_safety("ACC_STATUS_1", GATEWAY_BUS, values))
      self.assertEqual(i < (common.MAX_WRONG_COUNTERS - 1), valid, f"{i=}")
    self.assertFalse(self.safety.get_controls_allowed())

  def test_cruise_engagement(self):
    # engage on the rising edge of the stock ACC, disengage as soon as it drops
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._pcm_status_msg(False)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_no_engagement_from_wrong_bus(self):
    # ACC_STATUS_1 only exists on the gateway bus, the same address on the car buses is ignored
    for bus in (0, 2):
      self._rx(self.packer.make_can_msg_safety("ACC_STATUS_1", bus, {"ACC_ENGAGED": 1}))
      self.assertFalse(self.safety.get_controls_allowed(), f"{bus=}")

  def test_gas_pedal_low_values(self):
    # ACCEL_PEDAL spans the low 5 bits of byte 2 and the top 3 bits of byte 3
    for raw in (1, 8):
      self.assertTrue(self._rx(self._user_gas_msg(raw * 0.4)))
      self.assertTrue(self.safety.get_gas_pressed_prev(), f"{raw=}")
      self.assertTrue(self._rx(self._user_gas_msg(0)))
      self.assertFalse(self.safety.get_gas_pressed_prev(), f"{raw=}")

  def test_vehicle_moving_low_speeds(self):
    # VEHICLE_SPEED spans all of byte 1 and the top 3 bits of byte 2
    for raw in (1, 8):
      self.assertTrue(self._rx(self._speed_msg(raw * 0.017)))
      self.assertTrue(self.safety.get_vehicle_moving(), f"{raw=}")
      self.assertTrue(self._rx(self._speed_msg(0)))
      self.assertFalse(self.safety.get_vehicle_moving(), f"{raw=}")


if __name__ == "__main__":
  unittest.main()
