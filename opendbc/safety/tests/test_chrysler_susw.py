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


def flip_byte(index):
  def fix(msg):
    addr, dat, bus = msg
    dat = bytearray(dat)
    dat[index] ^= 0xFF
    return addr, bytes(dat), bus
  return fix


class TestChryslerSuswSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  TX_MSGS = [[0x1F6, 0], [0x547, 0], [0x5F0, 1]]
  STANDSTILL_THRESHOLD = 0
  # the camera is the only stock sender of both, on bus 2, so either one heard on bus 0 means the
  # relay is closed
  RELAY_MALFUNCTION_ADDRS = {0: (0x1F6, 0x547)}
  # nothing is blocked statically: the stock camera's LKAS_COMMAND and LKA_HUD_2 are blocked only
  # while openpilot's accepted stream of the same message is live, which chrysler_susw_fwd_hook()
  # decides, see test_stock_lkas_command_forwarding and test_lka_hud_forwarding
  FWD_BLACKLISTED_ADDRS: dict[int, list[int]] = {}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  MAX_RATE_UP = 5
  MAX_RATE_DOWN = 6
  MAX_TORQUE_LOOKUP = [0], [383]
  MAX_RT_DELTA = 180
  DRIVER_TORQUE_ALLOWANCE = 220
  DRIVER_TORQUE_FACTOR = 3

  # (message, address, bus, checksum byte index or None, carries a counter, values that keep
  # controls up)
  RX_CHECKED_MSGS = (
    ("ABS_3", 0xFA, 0, 7, True, {}),
    ("ABS_6", 0x101, 0, 7, True, {}),
    ("EPS_2", 0x106, 0, 6, True, {}),
    ("ACCEL_PEDAL_DRIVER", 0x1F0, 0, None, False, {}),
    ("ACC_STATUS_1", 0x103, GATEWAY_BUS, 7, True, {"ACC_ENGAGED": 1}),
    ("CRUISE_BUTTONS", 0x2FA, GATEWAY_BUS, 2, True, {}),
    ("LKA_HUD_2", 0x547, 2, None, False, {}),
    ("LANESENSE_BUTTON", 0x384, 0, None, False, {}),
  )

  def setUp(self):
    self.packer = CANPackerSafety("chrysler_susw")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.chryslerSusw, 0)
    self.safety.init_tests()

  def _pcm_status_msg(self, enable):
    values = {"ACC_ENGAGED": 1 if enable else 0}
    return self.packer.make_can_msg_safety("ACC_STATUS_1", GATEWAY_BUS, values)

  def _lanesense_msg(self, disabled, bus=2):
    # LKA_HUD_2 as the camera sends it. Since #1899 its LANESENSE_DISABLED is not G3 any more.
    values = {"LANESENSE_DISABLED": 1 if disabled else 0}
    return self.packer.make_can_msg_safety("LKA_HUD_2", bus, values)

  def _op_lanesense(self, on):
    """openpilot's own LKA_HUD_2 on bus 0, the LaneSense half of G3 since #1899."""
    self.assertTrue(self._tx(self._lka_hud_tx_msg(lanesense_disabled=not on, hud_state=2 if on else 0)))

  def _button_msg_lanesense(self, pressed, bus=0):
    return self.packer.make_can_msg_safety("LANESENSE_BUTTON", bus, {"LANESENSE_BUTTON": int(pressed)})

  def _engage(self):
    """Bring panda up to the full G3 engagement state: LaneSense on and the stock ACC engaged."""
    self._op_lanesense(True)
    self.assertTrue(self._rx(self._pcm_status_msg(True)))

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

  def _lka_hud_tx_msg(self, bus=0, lanesense_disabled=False, hud_state=12):
    # openpilot's own LKA_HUD_2 for the cluster (#1899)
    values = {"LANESENSE_DISABLED": int(lanesense_disabled), "LKA_HUD_STATE": hud_state, "WARN_BYTE5": 64}
    return self.packer.make_can_msg_safety("LKA_HUD_2", bus, values)

  def test_lka_hud_tx(self):
    # display only: accepted on bus 0 whatever the engagement state and content, nowhere else
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for disabled in (False, True):
        for hud_state in (0, 2, 12, 16, 20):
          self.assertTrue(self._tx(self._lka_hud_tx_msg(lanesense_disabled=disabled, hud_state=hud_state)))
      for bus in (1, 2, 3):
        self.assertFalse(self._tx(self._lka_hud_tx_msg(bus=bus)), f"{bus=}")
      # the allow-list entry is 8 bytes
      for length in (4, 7):
        self.assertFalse(self._tx(make_msg(0, 0x547, length)), f"{length=}")
    self.safety.set_relay_malfunction(True)
    self.assertFalse(self._tx(self._lka_hud_tx_msg()))

  def test_lka_hud_forwarding(self):
    # Until openpilot's own LKA_HUD_2 has been accepted, the camera's reaches the cluster. After that
    # the camera's copy stays blocked while openpilot's stream is live, regardless of controls.
    for controls_allowed in (False, True):
      with self.subTest(controls_allowed=controls_allowed):
        self._reset_safety_hooks()
        self.safety.set_controls_allowed(controls_allowed)
        self.safety.set_timer(0)
        self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x547))
        self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x547))   # other direction never blocked
        self.assertTrue(self._tx(self._lka_hud_tx_msg()))
        self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x547))
        self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x547))

  def test_lka_hud_forwarding_timeout(self):
    # openpilot's frame is 4 Hz: its stream is live for 1 s after the last accepted frame, then the
    # camera's frame is forwarded again, so a dead openpilot hands the cluster back to the camera
    self.safety.set_timer(0)
    self.assertTrue(self._tx(self._lka_hud_tx_msg()))
    for t_us, forwarded in ((250000, -1), (999999, -1), (1000000, 0), (1500000, 0)):
      with self.subTest(t_us=t_us):
        self.safety.set_timer(t_us)
        self.assertEqual(forwarded, self.safety.safety_fwd_hook(2, 0x547))

    # every accepted frame refreshes it
    for t_us in range(1500000, 4000000, 250000):
      self.safety.set_timer(t_us)
      self.assertTrue(self._tx(self._lka_hud_tx_msg()))
      self.safety.set_timer(t_us + 249999)
      self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x547))

  def test_lka_hud_rejected_tx_does_not_refresh(self):
    # a frame refused by the allow-list (wrong bus) cannot take or keep the cluster
    self.safety.set_timer(0)
    self.assertFalse(self._tx(self._lka_hud_tx_msg(bus=2)))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x547))
    self.assertTrue(self._tx(self._lka_hud_tx_msg()))
    self.safety.set_timer(900000)
    self.assertFalse(self._tx(self._lka_hud_tx_msg(bus=1)))
    self.safety.set_timer(1000000)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x547))

  def test_lka_hud_and_lkas_command_streams_are_independent(self):
    # each message's stock copy is gated on openpilot's stream of that same message only
    self.safety.set_timer(0)
    self.assertTrue(self._tx(self._lka_hud_tx_msg()))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x1F6))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x547))

    self._reset_safety_hooks()
    self.safety.set_timer(0)
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1F6))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x547))

    # LKAS_COMMAND times out at 50 ms, LKA_HUD_2 at 1 s
    self.assertTrue(self._tx(self._lka_hud_tx_msg()))
    self.safety.set_timer(50000)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x1F6))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x547))

  def test_reinit_clears_lka_hud_stream(self):
    self.safety.set_timer(0)
    self.assertTrue(self._tx(self._lka_hud_tx_msg()))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x547))
    self._reset_safety_hooks()
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x547))

  def test_relay_malfunction_lkas_command_unchanged(self):
    # LKAS_COMMAND keeps its relay check: heard on bus 0 it latches relay_malfunction, which blocks
    # every TX (both of openpilot's vehicle messages) and all forwarding; on bus 2 it is the camera
    # and nothing happens. LKA_HUD_2 joins it on bus 0 only.
    for addr in (0x1F6, 0x547):
      for bus, malfunction in ((0, True), (2, False), (GATEWAY_BUS, False)):
        with self.subTest(addr=hex(addr), bus=bus):
          self.setUp()   # init_tests() is past the relay's 1 s transition window, a bare re-init is not
          self.safety.set_relay_malfunction(False)
          self._rx(make_msg(bus, addr, 8))
          self.assertEqual(malfunction, self.safety.get_relay_malfunction())
          if malfunction:
            self.safety.set_controls_allowed(True)
            self.assertFalse(self._tx(self._torque_cmd_msg(0)))
            self.assertFalse(self._tx(self._lka_hud_tx_msg()))
            self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1F6))
            self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x547))

  def _release_from(self, torque):
    # get an accepted non-zero command on the wire while controls are allowed, then drop controls
    self._reset_safety_hooks()
    self.safety.set_controls_allowed(True)
    self._reset_torque_driver_measurement(0)
    self._set_prev_torque(torque)
    self.assertTrue(self._tx(self._torque_cmd_msg(torque)))
    self.safety.set_controls_allowed(False)

  def test_steer_safety_check(self):
    # Departure from the upstream invariant, on purpose: with controls disallowed a non-zero torque is
    # accepted only as a bounded release of the last accepted command (see test_release_*). From an
    # idle stream (last accepted torque 0) nothing non-zero can ever pass, which is what the upstream
    # test asserts; while enabled the upstream limits apply unchanged.
    for speed in self._torque_speed_range:
      self._reset_speed_measurement(speed)
      max_torque = self._get_max_torque(speed)
      for enabled in [0, 1]:
        for t in range(int(-max_torque * 1.5), int(max_torque * 1.5)):
          self._reset_safety_hooks()
          self._reset_speed_measurement(speed)
          self.safety.set_controls_allowed(enabled)
          self._set_prev_torque(t)
          if abs(t) > max_torque or (not enabled and abs(t) > 0):
            self.assertFalse(self._tx(self._torque_cmd_msg(t)))
          else:
            self.assertTrue(self._tx(self._torque_cmd_msg(t)))

  def test_release_ramps_down_after_disengage(self):
    # the controller's 60 counts/frame release from the cap, both signs, then idle
    for sign in (1, -1):
      with self.subTest(sign=sign):
        self._release_from(383 * sign)
        for torque in (323, 263, 203, 143, 83, 23, 0, 0):
          self.assertTrue(self._tx(self._torque_cmd_msg(torque * sign)), torque)
        # once at zero the stream is idle again: nothing non-zero passes
        self.assertFalse(self._tx(self._torque_cmd_msg(60 * sign)))

  def test_release_must_shrink_by_the_stock_rate(self):
    cases = ((383, False), (378, False), (377, True), (200, True), (1, True), (-377, False), (0, True))
    for torque, accepted in cases:
      with self.subTest(torque=torque):
        self._release_from(383)
        self.assertEqual(accepted, self._tx(self._torque_cmd_msg(torque)))
    # each accepted step has to shrink again from the new value; hovering is refused
    self._release_from(383)
    self.assertTrue(self._tx(self._torque_cmd_msg(300)))
    self.assertFalse(self._tx(self._torque_cmd_msg(300)))
    self.assertFalse(self._tx(self._torque_cmd_msg(295)))
    self.assertTrue(self._tx(self._torque_cmd_msg(294)))

  def test_release_needs_the_control_bit(self):
    self._release_from(383)
    self.assertFalse(self._tx(self._torque_cmd_msg(323, steer_req=0)))
    self.assertTrue(self._tx(self._torque_cmd_msg(323, steer_req=1)))
    self.assertTrue(self._tx(self._torque_cmd_msg(0, steer_req=0)))

  def test_release_is_capped_in_frames(self):
    # ten decaying frames at the minimum step, the eleventh is refused however small
    self._release_from(383)
    for i in range(1, 11):
      self.assertTrue(self._tx(self._torque_cmd_msg(383 - 6 * i)), i)
    self.assertFalse(self._tx(self._torque_cmd_msg(383 - 6 * 11)))
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))

  def test_release_cannot_start_from_idle(self):
    # a refused command while allowed leaves nothing to release from, and neither does idle
    self._reset_safety_hooks()
    self.safety.set_controls_allowed(False)
    for torque in (383, 60, 6, -6):
      self.assertFalse(self._tx(self._torque_cmd_msg(torque)))
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._torque_cmd_msg(6)))

  def test_release_frames_reset_when_controls_return(self):
    self._release_from(383)
    for i in range(1, 11):
      self.assertTrue(self._tx(self._torque_cmd_msg(383 - 6 * i)))
    # re-engage: normal limits, and a fresh release budget afterwards
    self.safety.set_controls_allowed(True)
    self._reset_torque_driver_measurement(0)
    self._set_prev_torque(200)
    self.assertTrue(self._tx(self._torque_cmd_msg(200)))
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._torque_cmd_msg(140)))
    self.assertTrue(self._tx(self._torque_cmd_msg(80)))

  def test_release_keeps_openpilot_the_sender(self):
    # accepted release frames refresh the forwarding stamp, so the stream stays continuous
    self._release_from(383)
    self.safety.set_timer(0)
    self.assertTrue(self._tx(self._torque_cmd_msg(323)))
    self.safety.set_timer(40000)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1F6))
    self.assertTrue(self._tx(self._torque_cmd_msg(263)))
    self.safety.set_timer(80000)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1F6))

  def test_stock_lkas_command_forwarding(self):
    # Until an LKAS_COMMAND passes the tx hook, stock remains the sole sender. Once openpilot has an
    # accepted stream it owns the address regardless of controls_allowed.
    for controls_allowed in (False, True):
      with self.subTest(controls_allowed=controls_allowed):
        self._reset_safety_hooks()
        self.safety.set_controls_allowed(controls_allowed)
        self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x1F6))
        self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x1F6))   # other direction unchanged
        self.assertTrue(self._tx(self._torque_cmd_msg(0)))
        self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1F6))

    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x1F5))

  def test_idle_control_bit_values_refresh_forwarding_timeout(self):
    self.safety.set_controls_allowed(False)
    self.safety.set_timer(0)
    self.assertTrue(self._tx(self._torque_cmd_msg(0, steer_req=1)))

    self.safety.set_timer(40000)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1F6))
    self.assertTrue(self._tx(self._torque_cmd_msg(0, steer_req=0)))

    self.safety.set_timer(80000)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1F6))
    self.safety.set_timer(90000)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x1F6))

  def test_stock_lkas_command_forwarding_timeout(self):
    self.safety.set_timer(0)
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))

    for t_us, forwarded in ((0, -1), (40000, -1), (50000, 0), (60000, 0)):
      with self.subTest(t_us=t_us):
        self.safety.set_timer(t_us)
        self.assertEqual(forwarded, self.safety.safety_fwd_hook(2, 0x1F6))

  def test_refused_lkas_command_does_not_refresh_forwarding_timeout(self):
    cases = (
      (False, 1, "nonzero torque while controls are disallowed"),
      (True, self.MAX_RATE_UP + 1, "rate-limit violation while controls are allowed"),
    )
    for controls_allowed, refused_torque, description in cases:
      with self.subTest(description):
        self._reset_safety_hooks()
        self.safety.set_controls_allowed(controls_allowed)
        self.safety.set_timer(0)
        self.assertTrue(self._tx(self._torque_cmd_msg(0)))

        self.safety.set_timer(10000)
        self.assertFalse(self._tx(self._torque_cmd_msg(refused_torque)))
        self.safety.set_timer(40000)
        self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1F6))
        self.safety.set_timer(50000)
        self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x1F6))

  def test_controls_drop_keeps_stock_blocked_while_idle_stream_continues(self):
    self.safety.set_timer(0)
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1F6))

    self.safety.set_controls_allowed(False)
    self.safety.set_timer(40000)
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1F6))

    self.safety.set_timer(50000)
    self.assertFalse(self._tx(self._torque_cmd_msg(1)))
    self.safety.set_timer(89999)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1F6))
    self.safety.set_timer(90000)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x1F6))

  def test_reinit_clears_lkas_command_stream(self):
    self.safety.set_timer(0)
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1F6))

    self._reset_safety_hooks()
    self.safety.set_controls_allowed(True)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x1F6))

  def test_stock_lkas_command_forwarding_follows_engagement(self):
    # the same thing driven through the real engagement path rather than set_controls_allowed()
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x1F6))
    self._engage()
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.set_timer(0)
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1F6))

    # LaneSense off drops controls, but the accepted torque-zero idle stream retains ownership.
    self._op_lanesense(False)
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertFalse(self.safety.get_controls_allowed())

    self.safety.set_timer(10000)
    self.assertTrue(self._tx(self._torque_cmd_msg(0)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1F6))
    self.safety.set_timer(59999)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1F6))
    self.safety.set_timer(60000)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x1F6))

  def test_gateway_bus_not_forwarded(self):
    # nothing is forwarded to or from the private fusion bus
    for addr in self.SCANNED_ADDRS:
      self.assertEqual(-1, self.safety.safety_fwd_hook(GATEWAY_BUS, addr), f"{addr=:#x}")
      for bus in range(4):
        self.assertNotEqual(GATEWAY_BUS, self.safety.safety_fwd_hook(bus, addr), f"{addr=:#x} {bus=}")

  def _run_ramp(self, torques):
    """Send `torques`, one per 10 ms frame with a moving clock, and return the index of the first
    blocked frame (None if none was blocked)."""
    self._reset_safety_hooks()
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self._reset_torque_driver_measurement(0)
    blocked_at = None
    for i, torque in enumerate(torques):
      self.safety.set_timer(i * 10000)  # 100 Hz command rate
      if not self._tx(self._torque_cmd_msg(torque)) and blocked_at is None:
        blocked_at = i
    return blocked_at

  @staticmethod
  def _ramp_torques(up, down, peak):
    torques, t = [], 0
    while t < peak:
      t = min(t + up, peak)
      torques.append(t)
    while t > 0:
      t = max(t - down, 0)
      torques.append(t)
    return torques

  def test_continuous_ramp_is_never_blocked(self):
    # what openpilot actually sends: up at max_rate_up, back down at max_rate_down, on a moving
    # 100 Hz clock. 26 frames fit MAX_RT_INTERVAL (250 ms), so max_rt_delta has to clear the
    # fastest legal movement inside one window, 26 * MAX_RATE_DOWN.
    self.assertGreater(self.MAX_RT_DELTA, 26 * self.MAX_RATE_DOWN)
    torques = self._ramp_torques(self.MAX_RATE_UP, self.MAX_RATE_DOWN, self.MAX_TORQUE)
    self.assertIsNone(self._run_ramp(torques))
    self.assertIsNone(self._run_ramp([-t for t in torques]))

  def test_ramp_above_rate_limit_is_blocked(self):
    # One count per frame over max_rate_up is refused on the very first step, in both directions.
    # There is no descending counterpart: reducing torque is never rate limited (max_rate_down
    # bounds how fast the upper *limit* winds down, not the command), only the RT window bounds it.
    too_fast = self._ramp_torques(self.MAX_RATE_UP + 1, self.MAX_RATE_DOWN, self.MAX_TORQUE)
    self.assertEqual(0, self._run_ramp(too_fast))
    self.assertEqual(0, self._run_ramp([-t for t in too_fast]))

  def _rx_all_checked(self, skip=None):
    for name, addr, bus, _, _, values in self.RX_CHECKED_MSGS:
      if addr != skip:
        self.assertTrue(self._rx(self.packer.make_can_msg_safety(name, bus, values)), name)

  def test_slow_check_lag_drops_controls(self):
    # LKA_HUD_2 (the camera, liveness only since #1899) and LANESENSE_BUTTON are 4 Hz. safety_tick()
    # refuses any check declared below 10 Hz outright, so both are declared at that floor, which
    # leaves them on the generic 1 s lag threshold: a 250 ms message clears it with 4x margin, but
    # either one going quiet invalidates the config and drops controls even while every other
    # checked message keeps arriving.
    for quiet in (None, 0x547, 0x384):
      with self.subTest(quiet=quiet):
        self._reset_safety_hooks()
        self.safety.set_timer(0)
        self._rx_all_checked()
        self._engage()
        self.assertTrue(self.safety.get_controls_allowed())

        self.safety.set_timer(int(1.1e6))
        self._op_lanesense(True)
        self._rx_all_checked(skip=quiet)
        self.safety.safety_tick_current_safety_config()
        self.assertEqual(quiet is None, self.safety.safety_config_valid())
        self.assertEqual(quiet is None, self.safety.get_controls_allowed())

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

  def test_rx_checksum_rejected(self):
    # every checksummed rx message must reject a corrupted checksum and drop controls with it
    for name, addr, bus, checksum_byte, _, values in self.RX_CHECKED_MSGS:
      with self.subTest(msg=name):
        self._reset_safety_hooks()
        self.assertTrue(self._rx(self.packer.make_can_msg_safety(name, bus, values)))

        self.safety.set_controls_allowed(True)
        fix = flip_byte(0 if checksum_byte is None else checksum_byte)
        bad = self.packer.make_can_msg_safety(name, bus, values, fix_checksum=fix)
        if checksum_byte is None:
          # ACCEL_PEDAL_DRIVER, LKA_HUD_2 and LANESENSE_BUTTON carry neither a checksum nor a counter, they are
          # rate-only checks: a garbage frame is still accepted and engagement survives it
          self.assertTrue(self._rx(bad))
          # all ones with LANESENSE_BUTTON's own bit (byte 3 bit 6) clear: a set bit there is a press,
          # which is a kill by design (test_button_*), not a garbage frame
          garbage = bytearray(b"\xff" * 8)
          if addr == 0x384:
            garbage[3] &= 0xBF
          self.assertTrue(self._rx(make_msg(bus, addr, dat=bytes(garbage))))
          self.assertTrue(self.safety.get_controls_allowed())
        else:
          self.assertFalse(self._rx(bad))
          self.assertFalse(self.safety.get_controls_allowed())

  def test_rx_counter_rejected(self):
    # every counted rx message must fail after MAX_WRONG_COUNTERS stuck counters, and the ones
    # without a counter must keep being accepted
    for name, _addr, bus, _, has_counter, values in self.RX_CHECKED_MSGS:
      with self.subTest(msg=name):
        self._reset_safety_hooks()
        stuck = dict(values, COUNTER=0) if has_counter else values
        for i in range(common.MAX_WRONG_COUNTERS + 1):
          self.safety.set_controls_allowed(True)
          valid = self._rx(self.packer.make_can_msg_safety(name, bus, dict(stuck)))
          expected = (not has_counter) or (i < (common.MAX_WRONG_COUNTERS - 1))
          self.assertEqual(expected, valid, f"{name} {i=}")
        self.assertEqual(not has_counter, self.safety.get_controls_allowed(), name)

  def test_cruise_engagement(self):
    # engage on the rising edge of the stock ACC with LaneSense on, disengage as soon as it drops
    self.assertFalse(self.safety.get_controls_allowed())
    self._engage()
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._pcm_status_msg(False)))
    self.assertFalse(self.safety.get_controls_allowed())

  # the two generic cruise tests, with the LaneSense half of G3 satisfied first
  def test_enable_control_allowed_from_cruise(self):
    self._op_lanesense(True)
    super().test_enable_control_allowed_from_cruise()

  def test_cruise_engaged_prev(self):
    self._op_lanesense(True)
    super().test_cruise_engaged_prev()

  def test_no_engagement_before_openpilot_reports_lanesense(self):
    # fail closed: until openpilot's own LKA_HUD_2 has reported LaneSense on, the stock ACC alone may
    # not engage openpilot, no matter how long it stays engaged
    for _ in range(10):
      self.assertTrue(self._rx(self._pcm_status_msg(True)))
      self.assertFalse(self.safety.get_controls_allowed())

    # and it engages on the first ACC frame after LaneSense turns out to be on
    self._op_lanesense(True)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_no_engagement_with_lanesense_off(self):
    self._op_lanesense(False)
    for _ in range(10):
      self.assertTrue(self._rx(self._pcm_status_msg(True)))
      self.assertFalse(self.safety.get_controls_allowed())

  def test_lanesense_off_disengages(self):
    # openpilot reporting LaneSense off while the stock ACC stays engaged has to drop controls
    self._engage()
    self.assertTrue(self.safety.get_controls_allowed())
    self._op_lanesense(False)
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertFalse(self.safety.get_controls_allowed())

    # it stays down for as long as LaneSense is off
    for _ in range(10):
      self.assertTrue(self._rx(self._pcm_status_msg(True)))
      self.assertFalse(self.safety.get_controls_allowed())

  def test_lanesense_back_on_reengages(self):
    # a fresh 0 -> 1 edge of (ACC engaged and LaneSense on) re-engages, on the ACC frame
    self._engage()
    self._op_lanesense(False)
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertFalse(self.safety.get_controls_allowed())

    self._op_lanesense(True)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_lanesense_needs_a_live_openpilot_stream(self):
    # openpilot's LaneSense on counts only while its LKA_HUD_2 stream is live (1 s, like the
    # forwarding gate): a dead openpilot reads as LaneSense off on the next ACC frame
    self.safety.set_timer(0)
    self._engage()
    self.safety.set_timer(999999)
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.set_timer(1000000)
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertFalse(self.safety.get_controls_allowed())

    # back alive: still no engagement without a fresh rising edge of the whole conjunction
    self._op_lanesense(True)
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())

    # a refused frame (wrong bus) neither refreshes the stream nor sets the state
    self._reset_safety_hooks()
    self.assertFalse(self._tx(self._lka_hud_tx_msg(bus=2)))
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_camera_lanesense_is_ignored(self):
    # #1899: the camera's LANESENSE_DISABLED (bus 2, or the same address anywhere else) can neither
    # satisfy G3 nor clear it. Route e7 2157-2162 s: the camera switches itself off ~5 s into an
    # ignored hands-off warning, which must not disengage openpilot.
    for bus in (0, GATEWAY_BUS, 2, 3):
      self._reset_safety_hooks()
      self._rx(self._lanesense_msg(False, bus=bus))
      self._rx(self._pcm_status_msg(True))
      self.assertFalse(self.safety.get_controls_allowed(), f"{bus=}")

      self._reset_safety_hooks()
      self._engage()
      self._rx(self._lanesense_msg(True, bus=bus))
      self._rx(self._pcm_status_msg(True))
      self.assertTrue(self.safety.get_controls_allowed(), f"{bus=}")

  def test_lanesense_bit_position(self):
    # LANESENSE_DISABLED is DBC 30|1@0+, byte 3 bit 6 of openpilot's LKA_HUD_2: no other bit may stand
    # in for it, and byte 3 bit 6 on its own must be enough
    for i in range(64):
      dat = bytearray(8)
      dat[i // 8] = 1 << (i % 8)
      self._reset_safety_hooks()
      self.assertTrue(self._tx(make_msg(0, 0x547, dat=bytes(dat))))
      self._rx(self._pcm_status_msg(True))
      self.assertEqual(i != 30, self.safety.get_controls_allowed(), f"{i=}")

  def test_button_edge_drops_controls_at_once(self):
    # the LaneSense button means openpilot off. Its rising edge drops controls on the button frame
    # itself, while openpilot's LKA_HUD_2 stream is live and still says LaneSense on, and before any
    # further ACC frame
    self._engage()
    self.assertTrue(self._rx(self._button_msg_lanesense(False)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._button_msg_lanesense(True)))
    self.assertFalse(self.safety.get_controls_allowed())

    # and it sticks: openpilot's frame still says on and ACC stays engaged, but there is no new
    # rising edge of the conjunction until openpilot has reported LaneSense off and then on
    for _ in range(10):
      self._op_lanesense(True)
      self.assertTrue(self._rx(self._pcm_status_msg(True)))
      self.assertFalse(self.safety.get_controls_allowed())
    self._op_lanesense(False)
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self._op_lanesense(True)
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_button_held_does_not_retrigger(self):
    self._engage()
    self.assertTrue(self._rx(self._button_msg_lanesense(True)))
    self.assertFalse(self.safety.get_controls_allowed())
    # re-engaged while the button is still held (a 10 s hold on route 000000d8): no second kill
    self._op_lanesense(False)
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self._op_lanesense(True)
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())
    for _ in range(40):
      self.assertTrue(self._rx(self._button_msg_lanesense(True)))
      self.assertTrue(self.safety.get_controls_allowed())
    # release, then the next press kills again
    self.assertTrue(self._rx(self._button_msg_lanesense(False)))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._button_msg_lanesense(True)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_button_press_while_disallowed_is_a_noop(self):
    # turning LaneSense on is a press too: with controls already off it grants nothing and blocks
    # nothing, engagement follows openpilot's frame and the ACC edge as usual
    self._op_lanesense(False)
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self._rx(self._button_msg_lanesense(True)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._button_msg_lanesense(False)))
    self.assertFalse(self.safety.get_controls_allowed())
    self._op_lanesense(True)
    self.assertTrue(self._rx(self._pcm_status_msg(True)))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_button_only_from_the_car_side(self):
    # the BCM is on bus 0; the same address elsewhere is no press
    for bus in (GATEWAY_BUS, 2, 3):
      self._reset_safety_hooks()
      self._engage()
      self._rx(self._button_msg_lanesense(False, bus=bus))
      self._rx(self._button_msg_lanesense(True, bus=bus))
      self.assertTrue(self.safety.get_controls_allowed(), f"{bus=}")

  def test_button_bit_position(self):
    # LANESENSE_BUTTON is DBC 30|1@0+, byte 3 bit 6; no other bit of 0x384 is a press
    for i in range(64):
      dat = bytearray(8)
      dat[i // 8] = 1 << (i % 8)
      self._reset_safety_hooks()
      self._engage()
      self.assertTrue(self._rx(make_msg(0, 0x384, dat=bytes(dat))))
      self.assertEqual(i != 30, self.safety.get_controls_allowed(), f"{i=}")

  def test_button_kill_reinit(self):
    # a button held through a safety re-init is an edge on its first frame; controls start
    # disallowed there anyway, and engagement still works once it is released and openpilot is on
    self.assertTrue(self._rx(self._button_msg_lanesense(True)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._rx(self._button_msg_lanesense(False)))
    self._engage()
    self.assertTrue(self.safety.get_controls_allowed())

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
    # VEHICLE_SPEED is 12 bits: bit 0 of byte 0, all of byte 1 and the top 3 bits of byte 2. Each
    # of the three slices must count as moving on its own; 2048 is the MSB alone, which is what the
    # frame looks like 0.017 m/s above the old 11-bit full scale (AH-299).
    for raw in (1, 8, 2048):
      self.assertTrue(self._rx(self._speed_msg(raw * 0.017)))
      self.assertTrue(self.safety.get_vehicle_moving(), f"{raw=}")
      self.assertTrue(self._rx(self._speed_msg(0)))
      self.assertFalse(self.safety.get_vehicle_moving(), f"{raw=}")


if __name__ == "__main__":
  unittest.main()
