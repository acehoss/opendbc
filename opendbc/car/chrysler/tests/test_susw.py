import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.chrysler.chryslercan import create_comma_heartbeat, create_lkas_command
from opendbc.car.chrysler.interface import CarInterface
from opendbc.car.chrysler.values import CAR, CarControllerParams

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter

# Addresses of every message the SUSW port reads, see opendbc/dbc/chrysler_susw.dbc.
# Bus 0 is the camera-side "CAN CH" bus, bus 1 is the private fusion bus fed by the gateway.
PT_ADDRS = {"EPS_1": 0xde, "ABS_1": 0xee, "ABS_3": 0xfa, "ENGINE_1": 0xfc,
            "ABS_6": 0x101, "EPS_2": 0x106, "ACCEL_PEDAL_DRIVER": 0x1f0,
            "SEATBELT_STATUS": 0x257, "DOORS": 0x4b1, "GEAR_2": 0x5a9,
            "STEERING_LEVERS": 0x73e}
ADAS_ADDRS = {"ACC_STATUS_1": 0x103, "CRUISE_BUTTONS": 0x2fa, "ACC_HUD": 0x73c}
CAM_ADDRS = {"LKA_HUD_2": 0x547}

# All raw frames below were captured from the 2023 Jeep Renegade on route
# 873a474e9ad72abb|000000e7--c05680fea1 and its paired raw CAN C capture.

# t=1200 s, cruising at ~16 m/s with the driver holding the wheel. ACC engagement #1 (1160.9-1210.6)
# is active here, so ENGINE_1.THROTTLE_VIRTUAL reads 5.6 % while the driver's foot is off the pedal.
DRIVING = {
  "EPS_1": "1c2697cf0b02",
  "ABS_1": "1d68e987543a606f",
  "ABS_3": "80200000480800b5",
  "ENGINE_1": "1b9cc1c208420fa8",
  "ABS_6": "00754000000000ba",
  "EPS_2": "7c737060400b15",
  "ACCEL_PEDAL_DRIVER": "0000000000000000",
  "SEATBELT_STATUS": "01fe104000000000",   # driver belt latched
  "GEAR_2": "000c800000000000",            # D, parking brake released
  "DOORS": "0000000000000000",
  "STEERING_LEVERS": "00000000",
}

# Isolated parked body-state sweep
BRAKE_PRESSED = "7c200000080800c7"      # ABS_3 with BRAKE_PEDAL_SWITCH set
REVERSE = "0e10c0060841ee74"            # ENGINE_1 in reverse, accelerator released

# 0x257 driver seatbelt, from the narrated latch/unlatch/latch block at 202.7 / 212.4 / 218.3 s
BELT_LATCHED = "01fe104000000000"       # t=205.1, byte 2 = 0x10
BELT_UNLATCHED = "01fe304000000000"     # t=214.0, byte 2 = 0x30

# 0x5a9 gear and parking brake, from the parked PRNDL sweep (marks 124-127) and the brake sweep
GEAR2_P = "008c200000000000"            # t=462.9
GEAR2_R = "008c400000000000"            # t=467.9
GEAR2_N = "008c600000000000"            # t=472.9
GEAR2_D = "008c800000000000"            # t=477.9
GEAR2_MANUAL = "008cc00000000000"       # route 000000d8 t=557.0, the AutoStick gate
GEAR2_PB_ENGAGED = "058c200000000000"   # t=494.8, byte 1 = 0x8c
GEAR2_PB_RELEASED = "000c200000000000"  # t=498.9, byte 1 = 0x0c
LEVERS_LEFT = "00000200"
LEVERS_RIGHT = "00000100"
LEVERS_HAZARDS = "00000300"
DOOR_FL_OPEN = "0100000000000000"
DOOR_FR_OPEN = "0080000000000000"

# All-ones sentinel / saturated EPS frames. EPS_1 is the second frame of route
# 873a474e9ad72abb|00000003--a598b092c5, EPS_2 is from the parked steering sweep in route e7.
EPS_1_SENTINEL = "ffff9fff0046"          # STEERING_ANGLE raw 0x3fff, STEERING_RATE raw 0xfff
EPS_2_SATURATED = "41d300080008e8"       # DRIVER_TORQUE raw 0, the signed code minimum
EPS_2_QUIET = "7d638168000794"           # DRIVER_TORQUE 11, TORQUE_MOTOR 6

# Frames backing the DBC scale/sign corrections, all from route e7
EPS_1_AT_REST = "1c6697d00153"           # t=100.0, stationary, STEERING_RATE raw 2000
EPS_1_SWEEP = "20ba98fe0d68"             # t=509.2, parked sweep toward the left lock
ABS_2_LEFT_TURN = "83689790207f8076"     # t=1752.5, steering angle +152.5 deg
ABS_2_RIGHT_TURN = "8147946c60320747"    # t=924.5, steering angle -203.6 deg
ABS_5_STOPPED = "0000000000000d8b"       # t=0.6, VEHICLE_SPEED 0
ABS_5_FORWARD = "304afd21f500080f"       # t=915.0, 5.0 m/s forward
ABS_5_REVERSE = "15130f0ffa000822"       # t=890.2, 0.5 m/s in reverse

# 0x1f0 driver accelerator, from the isolated parked press at 564.15-571.59 s (marks 134/135)
PEDAL_PRESSED = "0440000000000000"       # t=564.6, 13.872 %, the peak of the press
PEDAL_BARELY_PRESSED = "0020000000000000"  # t=566.8, 0.408 %, one count

# LKA_HUD_2 from the camera on bus 2, LaneSense state in byte 3 bit 6
LANESENSE_ON_GREY = "0000000000400200"     # LaneSense on, no lanes acquired
LANESENSE_ON_GREEN = "0000000000400c00"    # LaneSense on, lanes acquired and armed
LANESENSE_OFF = "0000004000400000"         # LaneSense switched off (route 00000003 t=196.5)

# ACC messages, raw CAN C
ACC_OFF = "000003e80000082b"            # ACC_STATUS_1, ACC_ENGAGED = 0
ACC_ENGAGED = "0000200200000c70"        # ACC_STATUS_1, ACC_ENGAGED = 1
HUD_OFF = "009fc00000000104"            # ACC_HUD, ACC_STATE 0, no set speed
HUD_READY = "009fc000000dc314"          # ACC_HUD, ACC_STATE 1, no set speed
HUD_ENGAGED_60 = "009fc03c250fcb24"     # ACC_HUD, ACC_STATE 2, set speed 60 km/h
HUD_STANDBY_64 = "009fc04028119154"     # ACC_HUD, ACC_STATE 5, set speed 64 km/h retained
HUD_CANCELLING_64 = "009fc04028119134"  # ACC_HUD, ACC_STATE 3, the one-frame transition at cancel
BTN_NONE = "00085600"
BTN_MAIN = "010a2000"                   # ACC_ON_OFF
BTN_RESUME = "080c7800"                 # ACC_RESUME
BTN_SET_DECEL = "10023000"              # ACC_SET_DECEL
BTN_ACCEL = "200a1900"                  # ACC_ACCEL
BTN_GAP_DEC = "400ef200"                # ACC_DISTANCE_DEC
BTN_GAP_INC = "0085f100"                # ACC_DISTANCE_INC, byte 1 bit 7
BTN_CANCEL = "80089f00"                 # ACC_CANCEL, test-drive-2 t=947.9 ("cancel with button")


class SuswTestBase(unittest.TestCase):
  def setUp(self):
    self.CP = CarInterface.get_non_essential_params(CAR.JEEP_RENEGADE)
    self.CI = CarInterface(self.CP)
    self.nanos = 0

  def update(self, pt: dict | None = None, adas: dict | None = None, cam: dict | None = None) -> structs.CarState:
    frames = [CanData(PT_ADDRS[n], bytes.fromhex(h), 0) for n, h in (pt or {}).items()]
    frames += [CanData(ADAS_ADDRS[n], bytes.fromhex(h), 1) for n, h in (adas or {}).items()]
    frames += [CanData(CAM_ADDRS[n], bytes.fromhex(h), 2) for n, h in (cam or {}).items()]
    self.nanos += int(DT_CTRL * 1e9)
    return self.CI.update([(self.nanos, frames)])


class TestSuswCarState(SuswTestBase):
  def test_driving_snapshot(self):
    CS = self.update(DRIVING)

    # ABS_6.VEHICLE_SPEED is already m/s
    self.assertAlmostEqual(CS.vEgoRaw, 15.946, places=3)
    self.assertFalse(CS.standstill)
    self.assertAlmostEqual(CS.wheelSpeeds.fl, 15.997, places=3)
    self.assertAlmostEqual(CS.wheelSpeeds.fr, 15.878, places=3)
    self.assertAlmostEqual(CS.wheelSpeeds.rl, 15.946, places=3)
    self.assertAlmostEqual(CS.wheelSpeeds.rr, 15.878, places=3)

    self.assertAlmostEqual(CS.steeringAngleDeg, 3.8, places=3)
    self.assertAlmostEqual(CS.steeringRateDeg, -1.0, places=3)
    self.assertEqual(CS.steeringTorque, -125)      # EPS_2.DRIVER_TORQUE
    self.assertEqual(CS.steeringTorqueEps, -9)     # EPS_2.TORQUE_MOTOR
    self.assertTrue(CS.steeringPressed)            # |125| > STEER_THRESHOLD
    self.assertFalse(CS.steerFaultPermanent)

    self.assertFalse(CS.brakePressed)
    self.assertFalse(CS.gasPressed)                # driver pedal is 0 despite 5.6 % virtual throttle
    self.assertFalse(CS.doorOpen)
    self.assertFalse(CS.leftBlinker)
    self.assertFalse(CS.rightBlinker)
    self.assertEqual(CS.gearShifter, GearShifter.drive)

    # documented gaps: no seatbelt signal is decoded and BSM is not on a bus openpilot sees
    self.assertFalse(CS.seatbeltUnlatched)
    self.assertFalse(self.CP.enableBsm)

  def test_brake_and_reverse(self):
    CS = self.update(DRIVING | {"ABS_3": BRAKE_PRESSED, "ENGINE_1": REVERSE, "GEAR_2": GEAR2_R})
    self.assertTrue(CS.brakePressed)
    self.assertFalse(CS.gasPressed)
    self.assertEqual(CS.gearShifter, GearShifter.reverse)
    # ENGINE_1.REVERSE is only a cross-check now, but it must still agree
    self.assertTrue(self.CI.can_parsers[Bus.pt].vl["ENGINE_1"]["REVERSE"])

  def test_gas_pressed_is_driver_only(self):
    # the ACC-engaged frame pair: the PCM commands 5.6 % throttle with the driver's foot off the pedal
    engaged = self.update(DRIVING)
    self.assertFalse(engaged.gasPressed)
    throttle_virtual = self.CI.can_parsers[Bus.pt].vl["ENGINE_1"]["THROTTLE_VIRTUAL"]
    self.assertAlmostEqual(throttle_virtual, 5.6, places=3)

    # the isolated parked press, same 0x1f0 message, no deadband needed down to a single count
    for frame in (PEDAL_PRESSED, PEDAL_BARELY_PRESSED):
      with self.subTest(frame=frame):
        self.setUp()
        CS = self.update(DRIVING | {"ACCEL_PEDAL_DRIVER": frame})
        self.assertTrue(CS.gasPressed)

  def test_blinkers(self):
    for levers, left, right in ((LEVERS_LEFT, True, False), (LEVERS_RIGHT, False, True),
                                (LEVERS_HAZARDS, True, True), ("00000000", False, False)):
      with self.subTest(levers=levers):
        self.setUp()
        CS = self.update(DRIVING | {"STEERING_LEVERS": levers})
        self.assertEqual(CS.leftBlinker, left)
        self.assertEqual(CS.rightBlinker, right)

  def test_seatbelt(self):
    for frame, unlatched in ((BELT_LATCHED, False), (BELT_UNLATCHED, True)):
      with self.subTest(frame=frame):
        self.setUp()
        CS = self.update(DRIVING | {"SEATBELT_STATUS": frame})
        self.assertEqual(CS.seatbeltUnlatched, unlatched)

  def test_gear_shifter(self):
    # GEAR_2 (0x5a9) is on CAN CH, so P and N are reachable without forwarding GEAR 0x190
    cases = (
      (GEAR2_P, GearShifter.park),
      (GEAR2_R, GearShifter.reverse),
      (GEAR2_N, GearShifter.neutral),
      (GEAR2_D, GearShifter.drive),
      (GEAR2_MANUAL, GearShifter.manumatic),
    )
    for frame, gear in cases:
      with self.subTest(frame=frame):
        self.setUp()
        self.assertEqual(self.update(DRIVING | {"GEAR_2": frame}).gearShifter, gear)

  def test_parking_brake(self):
    for frame, engaged in ((GEAR2_PB_ENGAGED, True), (GEAR2_PB_RELEASED, False)):
      with self.subTest(frame=frame):
        self.setUp()
        self.assertEqual(self.update(DRIVING | {"GEAR_2": frame}).parkingBrake, engaged)

  def test_doors(self):
    for doors, expected in ((DOOR_FL_OPEN, True), (DOOR_FR_OPEN, True), ("0000000000000000", False)):
      with self.subTest(doors=doors):
        self.setUp()
        CS = self.update(DRIVING | {"DOORS": doors})
        self.assertEqual(CS.doorOpen, expected)

  def test_eps_wake_sentinel_holds_last_angle_and_rate(self):
    quiet = DRIVING | {"EPS_2": EPS_2_QUIET}
    CS = self.update(quiet)
    self.assertAlmostEqual(CS.steeringAngleDeg, 3.8, places=3)
    self.assertAlmostEqual(CS.steeringRateDeg, -1.0, places=3)
    self.assertEqual(CS.steeringTorque, 11)
    self.assertFalse(CS.steeringPressed)

    # only the EPS_1 all-ones wake artifact is discarded
    CS = self.update(quiet | {"EPS_1": EPS_1_SENTINEL})
    self.assertAlmostEqual(CS.steeringAngleDeg, 3.8, places=3)
    self.assertAlmostEqual(CS.steeringRateDeg, -1.0, places=3)

  def test_eps_wake_sentinel_on_first_frame(self):
    # nothing valid has been seen yet, so the held angle and rate are zero
    CS = self.update(DRIVING | {"EPS_1": EPS_1_SENTINEL})
    self.assertEqual(CS.steeringAngleDeg, 0.)
    self.assertEqual(CS.steeringRateDeg, 0.)

  def test_saturated_driver_torque_is_a_real_measurement(self):
    # -1024 is full right lock, not a sentinel: the parked right-to-lock sweep bottoms out there
    # while the left sweep only reaches +1022. Filtering it would hide the driver's hardest pull.
    CS = self.update(DRIVING | {"EPS_2": EPS_2_QUIET})
    self.assertEqual(CS.steeringTorque, 11)
    self.assertFalse(CS.steeringPressed)

    CS = self.update(DRIVING | {"EPS_2": EPS_2_SATURATED})
    self.assertEqual(CS.steeringTorque, -1024)
    self.assertTrue(CS.steeringPressed)
    self.assertEqual(CS.steeringTorqueEps, -947)

    # and it is what the controller limits against, so full-left torque must not survive it
    CC = structs.CarControl()
    CC.enabled = True
    CC.latActive = True
    CC.actuators.torque = 1.0
    self.CI.CS.out = CS
    _, can_sends = self.CI.apply(CC.as_reader(), self.nanos)
    torque = [(dat[0] << 3 | dat[1] >> 5) - 1024 for addr, dat, _ in can_sends if addr == 0x1f6]
    self.assertEqual(torque, [0])

  def test_cruise_state(self):
    cases = (
      (HUD_OFF, ACC_OFF, False, False, 0.),
      (HUD_READY, ACC_OFF, True, False, 0.),
      (HUD_ENGAGED_60, ACC_ENGAGED, True, True, 60 / 3.6),
      (HUD_STANDBY_64, ACC_OFF, True, False, 64 / 3.6),
      # ACC_STATE 3 and 4 are one-frame transitions at cancel and engage. They must not drop
      # availability: ACC_HUD is 1 Hz plus on change, so the parser latches the transition value
      # for a whole second while ACC_ENGAGED (100 Hz) already reads engaged -> wrongCarMode.
      (HUD_CANCELLING_64, ACC_ENGAGED, True, True, 64 / 3.6),
    )
    for hud, status, available, enabled, speed in cases:
      with self.subTest(hud=hud):
        self.setUp()
        CS = self.update(DRIVING, {"ACC_HUD": hud, "ACC_STATUS_1": status, "CRUISE_BUTTONS": BTN_NONE},
                         {"LKA_HUD_2": LANESENSE_ON_GREEN})
        self.assertEqual(CS.cruiseState.available, available)
        self.assertEqual(CS.cruiseState.enabled, enabled)
        self.assertAlmostEqual(CS.cruiseState.speed, speed, places=4)
        self.assertFalse(CS.cruiseState.nonAdaptive)

  def test_engagement_needs_lanesense(self):
    # G3: openpilot is active only while ACC is engaged AND LaneSense is on
    engaged = {"ACC_HUD": HUD_ENGAGED_60, "ACC_STATUS_1": ACC_ENGAGED, "CRUISE_BUTTONS": BTN_NONE}
    cases = (
      (LANESENSE_ON_GREY, True),
      (LANESENSE_ON_GREEN, True),
      (LANESENSE_OFF, False),
    )
    for hud, enabled in cases:
      with self.subTest(hud=hud):
        self.setUp()
        CS = self.update(DRIVING, engaged, {"LKA_HUD_2": hud})
        self.assertEqual(CS.cruiseState.enabled, enabled)
        # availability is the stock ACC state and is unaffected by LaneSense
        self.assertTrue(CS.cruiseState.available)

    # LaneSense on but ACC not engaged is also inactive
    self.setUp()
    CS = self.update(DRIVING, engaged | {"ACC_STATUS_1": ACC_OFF}, {"LKA_HUD_2": LANESENSE_ON_GREEN})
    self.assertFalse(CS.cruiseState.enabled)

    # and before the camera has said anything at all, LaneSense counts as off, not on
    self.setUp()
    self.assertFalse(self.update(DRIVING, engaged).cruiseState.enabled)

  def test_fusion_bus_freshness(self):
    engaged = {"ACC_HUD": HUD_ENGAGED_60, "ACC_STATUS_1": ACC_ENGAGED, "CRUISE_BUTTONS": BTN_NONE}
    lanesense = {"LKA_HUD_2": LANESENSE_ON_GREEN}

    # dashcam mode never transmits the heartbeat, so the gateway stays in BYPASS and bus 1 is empty.
    # There must be no cruise state, and the CarState must still be valid so the port works as a
    # dashcam: the bus 1 messages are registered with a nan frequency for exactly this reason.
    for _ in range(200):
      CS = self.update(DRIVING, cam=lanesense)
    self.assertFalse(CS.cruiseState.available)
    self.assertFalse(CS.cruiseState.enabled)
    self.assertTrue(self.CI.can_parsers[Bus.adas].can_valid)

    # once the gateway starts forwarding, cruise state appears
    CS = self.update(DRIVING, engaged, lanesense)
    self.assertTrue(CS.cruiseState.available)
    self.assertTrue(CS.cruiseState.enabled)

    # a fusion bus that goes quiet mid-drive drops back out after 0.5 s, and not before
    for _ in range(49):
      CS = self.update(DRIVING, cam=lanesense)
    self.assertTrue(CS.cruiseState.enabled)
    CS = self.update(DRIVING, cam=lanesense)
    self.assertFalse(CS.cruiseState.available)
    self.assertFalse(CS.cruiseState.enabled)

  def test_button_events(self):
    cases = (
      (BTN_MAIN, ButtonType.mainCruise),
      (BTN_ACCEL, ButtonType.accelCruise),
      (BTN_SET_DECEL, ButtonType.decelCruise),
      (BTN_RESUME, ButtonType.resumeCruise),
      (BTN_GAP_DEC, ButtonType.gapAdjustCruise),
      (BTN_GAP_INC, ButtonType.gapAdjustCruise),
    )
    for frame, button in cases:
      with self.subTest(frame=frame):
        self.setUp()
        adas = {"ACC_HUD": HUD_READY, "ACC_STATUS_1": ACC_OFF, "CRUISE_BUTTONS": BTN_NONE}
        self.assertEqual(len(self.update(DRIVING, adas).buttonEvents), 0)

        CS = self.update(DRIVING, adas | {"CRUISE_BUTTONS": frame})
        self.assertEqual([(e.type, e.pressed) for e in CS.buttonEvents], [(button, True)])

        CS = self.update(DRIVING, adas)
        self.assertEqual([(e.type, e.pressed) for e in CS.buttonEvents], [(button, False)])

  def test_cancel_button_is_never_masked(self):
    packer = CANPacker("chrysler_susw")
    adas = {"ACC_HUD": HUD_READY, "ACC_STATUS_1": ACC_OFF, "CRUISE_BUTTONS": BTN_NONE}
    lanesense = {"LKA_HUD_2": LANESENSE_ON_GREY}

    # a real captured cancel press
    self.update(DRIVING, adas, lanesense)
    CS = self.update(DRIVING, adas | {"CRUISE_BUTTONS": BTN_CANCEL}, lanesense)
    self.assertEqual([(e.type, e.pressed) for e in CS.buttonEvents], [(ButtonType.cancel, True)])

    # no two-button press was ever captured, so this frame is packed rather than recorded.
    # Cancel is checked first, so a co-pressed button cannot swallow it.
    both = packer.make_can_msg("CRUISE_BUTTONS", 1, {"ACC_CANCEL": 1, "ACC_ACCEL": 1, "COUNTER": 3})[1]
    self.setUp()
    self.update(DRIVING, adas, lanesense)
    CS = self.update(DRIVING, adas | {"CRUISE_BUTTONS": both.hex()}, lanesense)
    self.assertEqual([(e.type, e.pressed) for e in CS.buttonEvents], [(ButtonType.cancel, True)])

  def test_button_counter(self):
    # the counter is used to continue the stock button sequence, it is not read from the camera bus
    self.update(DRIVING, {"ACC_HUD": HUD_READY, "ACC_STATUS_1": ACC_OFF, "CRUISE_BUTTONS": BTN_RESUME})
    self.assertEqual(self.CI.CS.button_counter, 0xc)
    self.assertEqual(self.CI.CS.lkas_car_model, -1)  # SUSW never sends a HUD message


class TestSuswLkasCommand(unittest.TestCase):
  def setUp(self):
    self.CP = CarInterface.get_non_essential_params(CAR.JEEP_RENEGADE)
    self.packer = CANPacker("chrysler_susw")

  def test_matches_stock_camera(self):
    # real stock LKAS_COMMAND frames from the camera, packed byte for byte by create_lkas_command
    cases = (
      ("8cb00ae1", 101, True, 10),    # positive torque, steers left
      ("72d00e8b", -106, True, 14),   # negative torque, steers right
      ("80100a57", 0, True, 10),      # armed, no torque
      ("80000e97", 0, False, 14),     # not armed
    )
    for expected, torque, control_bit, counter in cases:
      with self.subTest(expected=expected):
        self.packer.counters[0x1f6] = counter
        addr, dat, bus = create_lkas_command(self.packer, self.CP, torque, control_bit)
        self.assertEqual(addr, 0x1f6)
        self.assertEqual(bus, 0)
        self.assertEqual(dat.hex(), expected)

  def test_comma_heartbeat(self):
    packer = CANPacker("chrysler_susw")
    # bus 1, the private fusion bus. CANPacker fills the FCA CRC-8 over bytes 0-6.
    self.assertEqual(create_comma_heartbeat(packer, 5, False), (0x5f0, b"\x01\x00\x00\x00\x00\x00\x05\x3e", 1))
    self.assertEqual(create_comma_heartbeat(packer, 5, True), (0x5f0, b"\x03\x00\x00\x00\x00\x00\x05\x84", 1))
    # the counter wraps at 16
    self.assertEqual(create_comma_heartbeat(packer, 21, False)[1], create_comma_heartbeat(packer, 5, False)[1])

    # the packed frame round-trips through the parser, checksum and counter included
    cp = CANParser("chrysler_susw", [("COMMA_HEARTBEAT", 10)], 1)
    cp.update([(0, [create_comma_heartbeat(packer, 5, True)])])
    self.assertEqual(cp.vl["COMMA_HEARTBEAT"]["OPENPILOT_ALIVE"], 1)
    self.assertEqual(cp.vl["COMMA_HEARTBEAT"]["LAT_ACTIVE"], 1)
    self.assertEqual(cp.vl["COMMA_HEARTBEAT"]["COUNTER"], 5)

  def test_control_bit_is_one(self):
    # RAM uses 2 in LKAS_CONTROL_BIT, SUSW (like CUSW) uses 1
    self.packer.counters[0x1f6] = 0
    dat = create_lkas_command(self.packer, self.CP, 0, True)[1]
    self.assertEqual(dat[1] & 0x30, 0x10)


class TestSuswCarController(SuswTestBase):
  def _run(self, frames: int, v_ego: float, lat_active: bool = True, cancel: bool = False, resume: bool = False, torque: float = 0.5):
    CC = structs.CarControl()
    CC.enabled = True
    CC.latActive = lat_active
    CC.cruiseControl.cancel = cancel
    CC.cruiseControl.resume = resume
    CC.actuators.torque = torque
    CC = CC.as_reader()

    self.update(DRIVING, {"ACC_HUD": HUD_ENGAGED_60, "ACC_STATUS_1": ACC_ENGAGED, "CRUISE_BUTTONS": BTN_NONE})

    sent = []
    for _ in range(frames):
      self.CI.CS.out.vEgo = v_ego
      _, can_sends = self.CI.apply(CC, self.nanos)
      self.nanos += int(DT_CTRL * 1e9)
      sent.append(can_sends)
    return sent

  def test_only_lkas_command_and_heartbeat_are_sent(self):
    # no cruise button TX and no HUD TX for SUSW, even when the controller asks to cancel and resume.
    # LKAS_COMMAND on bus 0 every frame, COMMA_HEARTBEAT on bus 1 every 10th frame, nothing else.
    sent = self._run(300, 20., cancel=True, resume=True)
    for frame, can_sends in enumerate(sent):
      expected = [(0x1f6, 0)] + ([(0x5f0, 1)] if frame % 10 == 0 else [])
      self.assertEqual(sorted((addr, bus) for addr, _, bus in can_sends), sorted(expected))

    heartbeats = [dat for can_sends in sent for addr, dat, _ in can_sends if addr == 0x5f0]
    self.assertEqual(len(heartbeats), 30)                            # 300 frames at 100 Hz -> 10 Hz
    self.assertTrue(all(dat[0] & 0x01 for dat in heartbeats))        # OPENPILOT_ALIVE always set
    self.assertEqual([dat[6] & 0xf for dat in heartbeats], [c % 16 for c in range(30)])

  def test_lat_inactive_commands_zero_torque(self):
    # the control bit is already latched on and the command is full scale, so only latActive
    # separates these two runs
    self._run(300, 20.)
    active = self._lkas(self._run(50, 20., torque=1.0))
    self.assertTrue(all(dat[1] & 0x10 for dat in active))          # still armed
    self.assertNotEqual(max((dat[0] << 3 | dat[1] >> 5) - 1024 for dat in active), 0)

    inactive = self._lkas(self._run(50, 20., lat_active=False, torque=1.0))
    self.assertTrue(all(dat[1] & 0x10 for dat in inactive))        # still armed
    self.assertEqual([(dat[0] << 3 | dat[1] >> 5) - 1024 for dat in inactive], [0] * 50)

  def test_heartbeat_is_sent_when_inactive(self):
    # the gateway opt-in does not depend on openpilot being engaged
    sent = self._run(30, 20., lat_active=False)
    heartbeats = [dat for can_sends in sent for addr, dat, _ in can_sends if addr == 0x5f0]
    self.assertEqual(len(heartbeats), 3)
    self.assertTrue(all(dat[0] & 0x01 for dat in heartbeats))
    self.assertFalse(any(dat[0] & 0x02 for dat in heartbeats))       # LAT_ACTIVE clear

  def test_steer_limits(self):
    params = CarControllerParams(self.CP)
    self.assertEqual(params.STEER_STEP, 1)               # 100 Hz, like the stock camera
    self.assertEqual(params.STEER_DELTA_UP, 6)           # measured stock per-frame max delta
    self.assertEqual(params.STEER_DELTA_DOWN, 6)
    self.assertEqual(params.STEER_MAX, 250)              # stock reaches 383, we stay conservative

  def test_driver_torque_limiting(self):
    # SUSW limits against DRIVER_TORQUE, so driver torque opposing the command clamps it
    params = CarControllerParams(self.CP)
    self.assertEqual(params.STEER_DRIVER_ALLOWANCE, 100)
    self.assertEqual(params.STEER_DRIVER_MULTIPLIER, 2)
    self.assertEqual(params.STEER_DRIVER_FACTOR, 1)

    # DRIVING carries DRIVER_TORQUE = -125, so max allowed = 250 + (100 - 125) * 2 = 200
    torques = [dat[0] << 3 | dat[1] >> 5 for dat in self._lkas(self._run(400, 20., torque=1.0))]
    self.assertEqual(max(t - 1024 for t in torques), 200)

  def test_control_bit_below_min_steer_speed(self):
    # the control bit never comes on below the stock LaneSense drop-out speed, however long we drive
    self.assertFalse(any(self._control_bits(self._run(400, self.CP.minSteerSpeed - 1.5))))

    # ...and it does come on above minSteerSpeed, once the re-enable guard has expired
    self.assertTrue(self._control_bits(self._run(400, self.CP.minSteerSpeed + 1.0))[-1])

  def test_min_steer_speed_hysteresis(self):
    # stock arms at 16.0 m/s and drops out at 14.9 m/s. Probe either side of 14.9, not either side
    # of some wider band, so that a wrong hysteresis value cannot pass.
    self.assertEqual(self.CP.minSteerSpeed, 16.0)
    self._run(300, self.CP.minSteerSpeed + 1.0)
    self.assertTrue(all(self._control_bits(self._run(50, 15.0))))
    self.assertFalse(any(self._control_bits(self._run(50, 14.8))))

  def test_reenable_guard(self):
    # EPS faults if LKAS re-enables too quickly, so the control bit is held off for 200 frames
    bits = self._control_bits(self._run(300, self.CP.minSteerSpeed + 1.0))
    self.assertFalse(any(bits[:201]))
    self.assertTrue(all(bits[201:]))

    # a single frame below the minimum steering speed restarts the 200 frame guard
    self.assertFalse(self._control_bits(self._run(1, self.CP.minSteerSpeed - 3.0))[0])
    bits = self._control_bits(self._run(250, self.CP.minSteerSpeed + 1.0))
    self.assertFalse(any(bits[:200]))
    self.assertTrue(all(bits[200:]))

  @staticmethod
  def _lkas(sent):
    """The LKAS_COMMAND payloads only, dropping the interleaved 10 Hz heartbeat."""
    return [dat for can_sends in sent for addr, dat, _ in can_sends if addr == 0x1f6]

  @classmethod
  def _control_bits(cls, sent):
    return [bool(dat[1] & 0x10) for dat in cls._lkas(sent)]


class TestSuswDbc(unittest.TestCase):
  """The three DBC corrections from analysis/carstate-evidence.md, checked on captured frames."""

  @staticmethod
  def _decode(name: str, addr: int, frame: str) -> dict:
    cp = CANParser("chrysler_susw", [(name, 0)], 0)
    cp.update([(0, [(addr, bytes.fromhex(frame), 0)])])
    return cp.vl[name]

  def test_steering_rate_scale(self):
    # raw sits at exactly 2000 at rest, and one count is one deg/s (not the 0.5 originally declared)
    at_rest = self._decode("EPS_1", 0xde, EPS_1_AT_REST)
    self.assertAlmostEqual(at_rest["STEERING_RATE"], 0., places=4)
    self.assertAlmostEqual(at_rest["STEERING_ANGLE"], 10.2, places=3)

    # mid parked sweep toward the left lock: angle and rate are both positive and 1:1 with d(angle)/dt
    sweep = self._decode("EPS_1", 0xde, EPS_1_SWEEP)
    self.assertAlmostEqual(sweep["STEERING_RATE"], 302., places=3)
    self.assertAlmostEqual(sweep["STEERING_ANGLE"], 121., places=3)

  def test_yaw_rate_sign(self):
    # left turn: angle, LATERAL_ACCEL and YAW_RATE must all be positive (openpilot is left-positive)
    left = self._decode("ABS_2", 0xfe, ABS_2_LEFT_TURN)
    self.assertGreater(left["LATERAL_ACCEL"], 0)
    self.assertGreater(left["YAW_RATE"], 0)
    self.assertAlmostEqual(left["YAW_RATE"], 0.3684, places=4)

    right = self._decode("ABS_2", 0xfe, ABS_2_RIGHT_TURN)
    self.assertLess(right["LATERAL_ACCEL"], 0)
    self.assertLess(right["YAW_RATE"], 0)
    self.assertAlmostEqual(right["YAW_RATE"], -0.4324, places=4)

  def test_parking_brake_polarity(self):
    # raw-C PARKING_BRAKE_STATUS 0x256 bit 3 is 1 = ENGAGED (the earlier reading was inverted)
    engaged = self._decode("PARKING_BRAKE_STATUS", 0x256, "a882010000400000")
    self.assertEqual(engaged["PARKING_BRAKE_ENGAGED"], 1)
    released = self._decode("PARKING_BRAKE_STATUS", 0x256, "0000010000400000")
    self.assertEqual(released["PARKING_BRAKE_ENGAGED"], 0)

    # the two CAN CH copies must agree with it frame for frame
    for pb, gear2, lanesense in ((1, GEAR2_PB_ENGAGED, "081dbc0800000000"),
                                 (0, GEAR2_PB_RELEASED, "081dbc0000000000")):
      with self.subTest(engaged=pb):
        self.assertEqual(self._decode("GEAR_2", 0x5a9, gear2)["PARKING_BRAKE_ENGAGED"], pb)
        self.assertEqual(self._decode("LANESENSE_BUTTON", 0x384, lanesense)["PARKING_BRAKE_ENGAGED_2"], pb)

  def test_low_beam(self):
    # byte 3 bit 7, reported alongside the parking brake in the same byte
    lit = self._decode("LANESENSE_BUTTON", 0x384, "081dbc8800000000")   # t=395.3, lamps lit
    self.assertEqual((lit["LOW_BEAM"], lit["PARKING_BRAKE_ENGAGED_2"]), (1, 1))
    out = self._decode("LANESENSE_BUTTON", 0x384, "081dbc0800000000")   # t=405.2, auto, daytime
    self.assertEqual((out["LOW_BEAM"], out["PARKING_BRAKE_ENGAGED_2"]), (0, 1))

  def test_abs_5_moving_and_direction(self):
    # bits 36-39 are a vehicle-moving flag, not a direction pair
    for frame, moving in ((ABS_5_STOPPED, 0), (ABS_5_FORWARD, 1), (ABS_5_REVERSE, 1)):
      with self.subTest(frame=frame):
        vl = self._decode("ABS_5", 0x116, frame)
        self.assertEqual([vl[f"VEHICLE_MOVING_{i}"] for i in range(1, 5)], [moving] * 4)

    # direction lives in the ACTIVE_* bits: FL/RL forward, FR/RR backward
    fwd = self._decode("ABS_5", 0x116, ABS_5_FORWARD)
    self.assertEqual((fwd["ACTIVE_FL"], fwd["ACTIVE_FR"]), (1, 0))
    rev = self._decode("ABS_5", 0x116, ABS_5_REVERSE)
    self.assertEqual((rev["ACTIVE_FL"], rev["ACTIVE_FR"]), (0, 1))


if __name__ == "__main__":
  unittest.main()
