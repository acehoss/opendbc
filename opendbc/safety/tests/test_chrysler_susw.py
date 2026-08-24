#!/usr/bin/env python3
import unittest
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, make_msg

# private fusion bus, a gateway copies raw CAN C messages onto it
GATEWAY_BUS = 1

# ACCEL_PEDAL_DRIVER gain, %/count
ACCEL_PEDAL_DRIVER_SCALE = 0.408

# real CRUISE_BUTTONS frames captured on the car: no button, resume, cancel
CRUISE_BUTTONS_FRAMES = (b"\x00\x08\x56\x00", b"\x08\x08\x0c\x00", b"\x80\x08\x9f\x00")


def corrupt_checksum(msg):
  addr, dat, bus = msg
  return addr, dat[:-1] + bytes([dat[-1] ^ 0xFF]), bus


class TestChryslerSuswSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  TX_MSGS = [[0x1F6, 0], [0x5F0, 1]]
  STANDSTILL_THRESHOLD = 0
  RELAY_MALFUNCTION_ADDRS = {0: (0x1F6,)}
  FWD_BLACKLISTED_ADDRS = {2: [0x1F6]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  MAX_RATE_UP = 6
  MAX_RATE_DOWN = 6
  MAX_TORQUE_LOOKUP = [0], [250]
  MAX_RT_DELTA = 180
  DRIVER_TORQUE_ALLOWANCE = 100
  DRIVER_TORQUE_FACTOR = 2

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
    values = {"ACCEL_PEDAL_DRIVER": gas}
    return self.packer.make_can_msg_safety("ACCEL_PEDAL_DRIVER", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PEDAL_SWITCH": 1 if brake else 0}
    return self.packer.make_can_msg_safety("ABS_3", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"DRIVER_TORQUE": torque}
    return self.packer.make_can_msg_safety("EPS_2", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"STEERING_TORQUE": torque, "LKAS_CONTROL_BIT": steer_req}
    return self.packer.make_can_msg_safety("LKAS_COMMAND", 0, values)

  def _heartbeat_msg(self, bus=GATEWAY_BUS, alive=1, lat_active=0):
    values = {"OPENPILOT_ALIVE": alive, "LAT_ACTIVE": lat_active}
    return self.packer.make_can_msg_safety("COMMA_HEARTBEAT", bus, values)

  def _button_msg(self, cancel=False, resume=False):
    values = {"ACC_CANCEL": cancel, "ACC_RESUME": resume}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", GATEWAY_BUS, values)

  def test_rx_hook(self):
    for count in range(20):
      self.assertTrue(self._rx(self._speed_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._user_brake_msg(False)), f"{count=}")
      self.assertTrue(self._rx(self._torque_driver_msg(0)), f"{count=}")
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

  def _ramp(self, step):
    """Run a continuous ramp from 0 to MAX_TORQUE at `step` counts per 10 ms frame with a moving
    clock, and return the index of the first blocked frame (None if none was blocked)."""
    self._reset_safety_hooks()
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self._reset_torque_driver_measurement(0)
    blocked_at = None
    for i in range(60):
      self.safety.set_timer(i * 10000)  # 100 Hz command rate
      torque = min(step * i, self.MAX_TORQUE)
      if not self._tx(self._torque_cmd_msg(torque)) and blocked_at is None:
        blocked_at = i
    return blocked_at

  def test_continuous_ramp_is_never_blocked(self):
    # A steady +MAX_RATE_UP/frame ramp is what openpilot actually sends. 26 frames fit in
    # MAX_RT_INTERVAL (250 ms), so max_rt_delta must clear 26 * MAX_RATE_UP = 156.
    self.assertGreater(self.MAX_RT_DELTA, 26 * self.MAX_RATE_UP)
    self.assertIsNone(self._ramp(self.MAX_RATE_UP))

  def test_ramp_above_rate_limit_is_blocked(self):
    # one count per frame faster than the stock camera: the per-frame rate limit catches it on the
    # very first step, well before the real time window would
    self.assertEqual(1, self._ramp(self.MAX_RATE_UP + 1))

  def test_heartbeat_tx(self):
    # the gateway's opt-in for INTERCEPT: always sendable on the fusion bus, never anywhere else
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for lat_active in (0, 1):
        self.assertTrue(self._tx(self._heartbeat_msg(lat_active=lat_active)))
      for bus in (0, 2, 3):
        self.assertFalse(self._tx(self._heartbeat_msg(bus=bus)))

  def test_heartbeat_not_forwarded(self):
    self.assertEqual(-1, self.safety.safety_fwd_hook(GATEWAY_BUS, 0x5F0))

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
    # ACCEL_PEDAL_DRIVER spans the low nibble of byte 0 and the top 3 bits of byte 1: raw 1 only
    # sets the byte 1 half, raw 8 only the byte 0 half
    for raw in (1, 8):
      self.assertTrue(self._rx(self._user_gas_msg(raw * ACCEL_PEDAL_DRIVER_SCALE)))
      self.assertTrue(self.safety.get_gas_pressed_prev(), f"{raw=}")
      self.assertTrue(self._rx(self._user_gas_msg(0)))
      self.assertFalse(self.safety.get_gas_pressed_prev(), f"{raw=}")

  def test_gas_pedal_ignores_bits_outside_the_signal(self):
    # bytes 2-7 are always zero on the car, and byte 0 bits 7-4 / byte 1 bits 4-0 are not part of
    # ACCEL_PEDAL_DRIVER. None of them may register as a pedal press.
    for i in range(8):
      dat = bytearray(8)
      if i == 0:
        dat[0] = 0xF0
      elif i == 1:
        dat[1] = 0x1F
      else:
        dat[i] = 0xFF
      self.assertTrue(self._rx(make_msg(0, 0x1F0, dat=bytes(dat))))
      self.assertFalse(self.safety.get_gas_pressed_prev(), bytes(dat).hex())

  def test_throttle_virtual_is_not_gas(self):
    # ENGINE_1.THROTTLE_VIRTUAL is the PCM's resolved demand, nonzero on 97.5 % of ACC-engaged
    # frames. It must never reach gas_pressed, or every stock ACC acceleration would disengage.
    self.safety.set_controls_allowed(True)
    for throttle in (0.4, 18.4, 51.6, 102):
      self._rx(self.packer.make_can_msg_safety("ENGINE_1", 0, {"THROTTLE_VIRTUAL": throttle}))
      self.assertFalse(self.safety.get_gas_pressed_prev(), f"{throttle=}")
      self.assertTrue(self.safety.get_controls_allowed(), f"{throttle=}")

  def test_vehicle_moving_low_speeds(self):
    # VEHICLE_SPEED spans all of byte 1 and the top 3 bits of byte 2
    for raw in (1, 8):
      self.assertTrue(self._rx(self._speed_msg(raw * 0.017)))
      self.assertTrue(self.safety.get_vehicle_moving(), f"{raw=}")
      self.assertTrue(self._rx(self._speed_msg(0)))
      self.assertFalse(self.safety.get_vehicle_moving(), f"{raw=}")


if __name__ == "__main__":
  unittest.main()
