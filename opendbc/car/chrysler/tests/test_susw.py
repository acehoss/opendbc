import unittest

from opendbc.can import CANDefine, CANPacker, CANParser
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import can_fingerprint
from opendbc.car.structs import CarParams
from opendbc.car.fingerprints import _FINGERPRINTS as ALL_FINGERPRINTS
from opendbc.car.chrysler.chryslercan import create_comma_heartbeat, create_lkas_command
from opendbc.car.chrysler.fingerprints import FINGERPRINTS
from opendbc.car.chrysler.interface import CarInterface
from opendbc.car.chrysler.values import CAR, CarControllerParams
from opendbc.safety.tests.common import CANPackerSafety, make_msg
from opendbc.safety.tests.libsafety import libsafety_py

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter

# Addresses of every message the SUSW port reads, see opendbc/dbc/chrysler_susw.dbc.
# Bus 0 is the camera-side "CAN CH" bus, bus 1 is the private fusion bus fed by the gateway.
PT_ADDRS = {"EPS_1": 0xde, "ABS_1": 0xee, "ABS_3": 0xfa, "ENGINE_1": 0xfc,
            "ABS_6": 0x101, "EPS_2": 0x106, "ACCEL_PEDAL_DRIVER": 0x1f0,
            "SEATBELT_STATUS": 0x257, "DOORS": 0x4b1, "GEAR_2": 0x5a9,
            "STEERING_LEVERS": 0x73e}
ADAS_ADDRS = {"ACC_STATUS_1": 0x103, "CRUISE_BUTTONS": 0x2fa, "ACC_HUD": 0x73c}
CAM_ADDRS = {"LKA_HUD_2": 0x547, "LKAS_COMMAND": 0x1f6}

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
GEAR2_PB_APPLYING = "028c200000000000"   # t=500.2, byte 0 = 0x02, the apply transition
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
# LKA_FAULT was 0 on 100 % of 674k captured frames, so this frame is packed, not recorded:
# EPS_2_QUIET with LKA_FAULT forced to 1 and the checksum recomputed.
EPS_2_LKA_FAULT = "7d63816880075d"

# EPS_2.LKA_LOW_SPEED_INHIBIT transitions, route e7. Each row is
# (label, EPS_2 with the bit set, EPS_2 with it clear, ABS_6 at the first, ABS_6 at the second),
# taken as the two consecutive 0x106 frames either side of one transition.
EPS_2_INHIBIT_EDGES = (
  ("release t=1153.69", "7b738868000386", "7c2384e0000475", "006440000000091d", "0064400000000a3a"),
  ("release t=1239.48", "7e63986800077a", "7ec39a40000838", "0064600000000cb5", "0064800000000dd5"),
  ("release t=1398.60", "7ca37ec8000176", "7ca37e000002fd", "0064e0000000032d", "0065000000000445"),
  ("assert  t=1216.02", "7de38ee8000eea", "7de38c20000d46", "0063e07400000357", "00634074000002a8"),
  ("assert  t=1382.38", "7d639808000c2f", "7de399e0000b5e", "0063e02400000e67", "0063a02400000ddf"),
  ("assert  t=1697.49", "7d937ce800065b", "7d337c20000532", "0062c0d4000004fc", "0062a0d4000003f1"),
)

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

# a real stock camera LKAS_COMMAND from bus 2: disarmed, zero torque, COUNTER 14
STOCK_LKAS_C14 = "80000e97"

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

# 0x103 / 0x15c engage-bit corroboration, raw CAN C capture 20260823T211320Z-collection-run-1 (route e7).
# One row per side of every ACC_STATUS_1.ACC_ENGAGED transition, sampled 300 ms out so the ~16 ms skew
# between the two messages cannot alias: (t, ACC_STATUS_1 frame, the ACC_COMMAND frame in force at that
# instant, the engaged state both must report). All 14 engagements are covered twice.
ACC_ENGAGE_PAIRS = (
  (1160.395, "000003e800000e65", "320001000000075f", 0),
  (1160.995, "0000647600000aa1", "3200014790000a66", 1),
  (1210.116, "001024a600000aae", "3200014a900003dc", 1),
  (1210.716, "002003e8000006a5", "3200010000000478", 0),
  (1242.326, "000003e800000f78", "320001000000000c", 0),
  (1242.926, "000024e800000bc6", "3200014e90000392", 1),
  (1283.687, "0030e47e000007a3", "32000148100009e9", 1),
  (1284.287, "002083e8000003ef", "3200010000000bc3", 0),
  (1286.847, "002003e8000003cc", "3200010000000478", 0),
  (1287.447, "0020246a00000f88", "32000146d000071a", 1),
  (1350.098, "000023f40000081d", "3200013f70000437", 1),
  (1350.698, "000003e8000004b7", "3200010000000642", 0),
  (1420.269, "002003e8000001f6", "3200010000000d8d", 0),
  (1420.869, "007024a200000dcb", "3200014a5000005b", 1),
  (1681.391, "00102456000001af", "32000145900003b4", 1),
  (1681.991, "001003e800000d56", "3200010000000565", 0),
  (1800.422, "002003e8000000eb", "3200010000000565", 0),
  (1801.022, "0000252c00000c10", "32000152f00008fa", 1),
  (1870.832, "000024ea00000133", "3200014ed0000b1a", 1),
  (1871.432, "002003e800000d6a", "3200010000000d8d", 0),
  (1884.312, "00b003e800000536", "3200010000000d8d", 0),
  (1884.912, "00d02416000001cb", "32000141900000dd", 1),
  (2242.762, "0000245200000a3a", "320001455000067d", 1),
  (2243.362, "000003e80000068d", "32000100000008e4", 0),
  (2353.231, "000003e8000001de", "3200010000000236", 0),
  (2353.832, "0000250000000d95", "32000150300005fc", 1),
  (2604.281, "00002446000002b9", "3200014490000113", 1),
  (2604.881, "000043e800000efa", "320001000000032b", 0),
  (2614.900, "002043e80000089c", "320001000000075f", 0),
  (2615.501, "000024900000041a", "3200014930000a63", 1),
  (2805.349, "0020249600000dae", "3200014990000ffa", 1),
  (2805.949, "000043e8000009a9", "3200010000000111", 0),
  (2883.079, "000003e800000a11", "3200010000000478", 0),
  (2883.679, "0000241200000632", "320001413000077e", 1),
  (2898.938, "0000a49200000cf6", "32000149500002db", 1),
  (2899.538, "000043e8000008b4", "3200010000000478", 0),
  (2940.758, "0000c3e800000245", "320001000000000c", 0),
  (2941.358, "0000a4f800000ea9", "3200014fb000033f", 1),
  (2995.068, "00002432000009c3", "3200014350000147", 1),
  (2995.668, "001003e8000005be", "320001000000032b", 0),
  (3016.408, "000003e800000f78", "3200010000000236", 0),
  (3017.008, "0010251000000bea", "3200015130000561", 1),
  (3141.497, "0030647800000c26", "32000147b00006ca", 1),
  (3142.097, "003043e800000888", "32000100000008e4", 0),
  (3197.237, "001003e8000002ed", "3200010000000fb7", 0),
  (3197.836, "0000242a00000e29", "32000142d000023d", 1),
  (3266.626, "0000200200000d6d", "ad91010010000298", 1),
  (3267.226, "000003e800000936", "3200010000000478", 0),
  (3309.886, "001003e8000003f0", "3200010000000eaa", 0),
  (3310.486, "000024aa00000f01", "3200014ad0000186", 1),
  (3356.576, "0030e48e0000003e", "32000149100002bb", 1),
  (3357.175, "0030c3e800000cdf", "3200010000000478", 0),
  (3383.076, "001043e800000a9a", "320001000000075f", 0),
  (3383.676, "000024300000065f", "32000143300009ff", 1),
  (3414.975, "0000243600000078", "32000143900006b4", 1),
  (3415.575, "000003e800000c5f", "32000100000008e4", 0),
)


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
    self.assertFalse(CS.steeringPressed)           # |125| < SUSW_STEER_THRESHOLD 160: resting hands
    self.assertFalse(CS.steerFaultTemporary)
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

  def test_lka_fault_is_temporary(self):
    """LKA_FAULT is wired to steerFaultTemporary, deliberately and on no positive evidence.

    The bit never asserts in any capture, so temporary vs permanent cannot be settled from data
    (raised by quiet-lynx and velvet-moth in round 1). carstate.py explains why temporary is the
    conservative choice for a dashcam-only first drive; this test exists so that flipping it to
    permanent later has to be a deliberate edit backed by a bench fault sample, not a silent drift.
    The asserting frame is packed, not recorded - see EPS_2_LKA_FAULT.
    """
    quiet = self.update(DRIVING | {"EPS_2": EPS_2_QUIET})
    self.assertFalse(quiet.steerFaultTemporary)
    self.assertFalse(quiet.steerFaultPermanent)

    CS = self.update(DRIVING | {"EPS_2": EPS_2_LKA_FAULT})
    self.assertTrue(CS.steerFaultTemporary)
    self.assertFalse(CS.steerFaultPermanent)

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

    # P and N are real now, so openpilot can tell it is not in a drivable gear (wrongGear) instead
    # of believing it is in drive. Chrysler adds only `low` to the drivable set.
    drivable = (GearShifter.drive,) + CarInterface.DRIVABLE_GEARS
    self.assertFalse(GearShifter.park in drivable)
    self.assertFalse(GearShifter.neutral in drivable)
    self.assertTrue(GearShifter.drive in drivable)

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

  def test_no_reengage_without_a_rising_edge(self):
    # The panda grants controls only on a 0 -> 1 transition of ACC_ENGAGED, and it sees no
    # transition across a fusion outage because it stops receiving 0x103 entirely. CarState must
    # not re-engage on its own or every LKAS_COMMAND would be blocked until controlsMismatch.
    # a live 0x103 stream: the counter has to advance or the parser rejects the frames
    packer = CANPacker("chrysler_susw")
    counter = 0

    def step(engaged: bool | None, n: int = 1):
      nonlocal counter
      for _ in range(n):
        adas = {"ACC_HUD": HUD_ENGAGED_60, "CRUISE_BUTTONS": BTN_NONE}
        if engaged is not None:
          dat = packer.make_can_msg("ACC_STATUS_1", 1, {"ACC_ENGAGED": int(engaged), "COUNTER": counter})[1]
          counter = (counter + 1) % 16
          adas["ACC_STATUS_1"] = dat.hex()
        CS = self.update(DRIVING, adas, {"LKA_HUD_2": LANESENSE_ON_GREEN})
      return CS

    self.assertTrue(step(True, 3).cruiseState.enabled)

    # the gateway drops out mid-engagement
    self.assertFalse(step(None, 60).cruiseState.enabled)

    # it comes back with ACC still engaged: no rising edge happened, so we stay disengaged
    CS = step(True, 20)
    self.assertTrue(CS.cruiseState.available)
    self.assertFalse(CS.cruiseState.enabled)

    # seeing ACC_ENGAGED low is what clears the panda's latch, and ours
    self.assertFalse(step(False).cruiseState.enabled)

    # now a real 0 -> 1 transition re-engages both
    self.assertTrue(step(True).cruiseState.enabled)

  def test_first_engagement_needs_no_edge(self):
    # at a clean start the panda's cruise_engaged_prev is false, so the first frame with
    # ACC_ENGAGED set IS a rising edge for it - we must not require an extra low observation
    engaged = {"ACC_HUD": HUD_ENGAGED_60, "ACC_STATUS_1": ACC_ENGAGED, "CRUISE_BUTTONS": BTN_NONE}
    CS = self.update(DRIVING, engaged, {"LKA_HUD_2": LANESENSE_ON_GREEN})
    self.assertTrue(CS.cruiseState.enabled)

  def test_speed_saturation_guard(self):
    # ABS_6.VEHICLE_SPEED is 11 bits and saturates at 34.799 m/s; nothing in the captures reaches
    # it, so a wrap above 125 km/h is unobserved. A wrap to zero would read as standstill and drop
    # the LKAS control bit at highway speed. This frame is packed, not captured - no wrap exists in
    # the data to record.
    packer = CANPacker("chrysler_susw")
    wrapped = packer.make_can_msg("ABS_6", 0, {"VEHICLE_SPEED": 0.})[1].hex()

    CS = self.update(DRIVING | {"ABS_6": wrapped})
    self.assertAlmostEqual(CS.wheelSpeeds.fl, 15.997, places=3)
    self.assertAlmostEqual(CS.vEgoRaw, 15.92475, places=4)   # the 13-bit wheel-speed mean
    self.assertFalse(CS.standstill)

    # a genuine standstill, where the wheels agree, is left alone
    self.setUp()
    stopped = packer.make_can_msg("ABS_6", 0, {"VEHICLE_SPEED": 0.})[1].hex()
    zero_wheels = packer.make_can_msg("ABS_1", 0, {})[1].hex()
    CS = self.update(DRIVING | {"ABS_6": stopped, "ABS_1": zero_wheels})
    self.assertEqual(CS.vEgoRaw, 0.)
    self.assertTrue(CS.standstill)

    # and below the ceiling ABS_6 is used unchanged, not blended
    self.setUp()
    CS = self.update(DRIVING)
    self.assertAlmostEqual(CS.vEgoRaw, 15.946, places=3)

    # the guard must stay a saturation guard rather than becoming a general divergence override:
    # a plausible mid-range ABS_6 reading wins even when the wheel speeds disagree wildly, because
    # in that direction it is the wheel speeds that are more likely to have gone stale
    self.setUp()
    mid = packer.make_can_msg("ABS_6", 0, {"VEHICLE_SPEED": 20.})[1].hex()
    CS = self.update(DRIVING | {"ABS_6": mid, "ABS_1": packer.make_can_msg("ABS_1", 0, {})[1].hex()})
    self.assertAlmostEqual(CS.vEgoRaw, 20., delta=0.02)   # 0.017 m/s LSB
    self.assertFalse(CS.standstill)

  def test_cruise_speed_is_gated(self):
    # holding the last set speed through an outage is inconsistent with available/enabled going False
    engaged = {"ACC_HUD": HUD_ENGAGED_60, "ACC_STATUS_1": ACC_ENGAGED, "CRUISE_BUTTONS": BTN_NONE}
    self.assertAlmostEqual(self.update(DRIVING, engaged).cruiseState.speed, 60 / 3.6, places=4)
    for _ in range(60):
      CS = self.update(DRIVING)
    self.assertEqual(CS.cruiseState.speed, 0.)

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
  def _run(self, frames: int, v_ego: float, lat_active: bool = True, cancel: bool = False, resume: bool = False,
           torque: float = 0.5, cam: dict | None = None):
    CC = structs.CarControl()
    CC.enabled = True
    CC.latActive = lat_active
    CC.cruiseControl.cancel = cancel
    CC.cruiseControl.resume = resume
    CC.actuators.torque = torque
    CC = CC.as_reader()

    self.update(DRIVING, {"ACC_HUD": HUD_ENGAGED_60, "ACC_STATUS_1": ACC_ENGAGED, "CRUISE_BUTTONS": BTN_NONE}, cam)

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

    # with lateral inactive nothing is transmitted at all any more - the stock camera's frame is
    # forwarded instead, see test_lkas_tx_is_gated_on_lat_active
    self.assertEqual(self._lkas(self._run(50, 20., lat_active=False, torque=1.0)), [])

  def test_lkas_tx_is_gated_on_lat_active(self):
    # G3 hand-over: the panda forwards the stock camera's 0x1F6 to the EPS while openpilot is inactive
    # (chrysler_susw_fwd_hook) and blocks it only while controls are allowed. openpilot must therefore
    # be silent on 0x1F6 unless it is the intended controller, or the EPS sees two 100 Hz senders.
    inactive = self._run(50, 20., lat_active=False, torque=1.0)
    self.assertEqual([(addr, bus) for can_sends in inactive for addr, _, bus in can_sends],
                     [(0x5f0, 1)] * 5)                            # heartbeat only, 10 Hz

    self.setUp()
    active = self._run(50, 20., lat_active=True, torque=1.0)
    self.assertEqual(len(self._lkas(active)), 50)                 # one 0x1f6 per 100 Hz frame

  def test_counter_continues_the_stock_sequence(self):
    # the camera keeps sending 0x1F6 while the panda blocks it, so the first openpilot frame after the
    # hand-over resumes the counter the EPS last saw rather than restarting an independent sequence
    sent = self._run(20, 20., cam={"LKAS_COMMAND": STOCK_LKAS_C14})
    counters = [dat[2] & 0xf for dat in self._lkas(sent)]         # LKAS_COMMAND.COUNTER, 19|4
    self.assertEqual(counters, [(15 + i) % 16 for i in range(20)])

    # going inactive and coming back resynchronises again instead of continuing from 15+20
    self._run(20, 20., lat_active=False, cam={"LKAS_COMMAND": STOCK_LKAS_C14})
    resumed = self._lkas(self._run(4, 20., cam={"LKAS_COMMAND": STOCK_LKAS_C14}))
    self.assertEqual([dat[2] & 0xf for dat in resumed], [15, 0, 1, 2])

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
    self.assertEqual(params.STEER_DELTA_UP, 5)           # one count under the measured stock max
    self.assertEqual(params.STEER_DELTA_DOWN, 6)         # release at the stock rate
    self.assertEqual(params.STEER_MAX, 383)              # the stock camera's measured peak (route 000000d8)

  def test_driver_torque_limiting(self):
    # SUSW limits against DRIVER_TORQUE, so driver torque opposing the command clamps it
    params = CarControllerParams(self.CP)
    self.assertEqual(params.STEER_DRIVER_ALLOWANCE, 160)
    self.assertEqual(params.STEER_DRIVER_MULTIPLIER, 3)
    self.assertEqual(params.STEER_DRIVER_FACTOR, 1)

    # DRIVING carries DRIVER_TORQUE = -125. That is resting-hands territory on this car (route
    # 00000123 p90 was 137), and under the 160 allowance it must NOT trim the command at all.
    torques = [dat[0] << 3 | dat[1] >> 5 for dat in self._lkas(self._run(400, 20., torque=1.0))]
    self.assertEqual(max(t - 1024 for t in torques), params.STEER_MAX)

    # A genuine override still takes it away: the ceiling is 383 + (160 - d) * 3, reaching zero at
    # d ~= 288 counts. The full driver-limit envelope is exercised by the generic
    # TorqueDriverSafetyTest in opendbc.safety.tests.test_chrysler_susw.

  def test_control_bit_below_min_steer_speed(self):
    # the control bit never comes on below the stock LaneSense drop-out speed, however long we drive
    self.assertFalse(any(self._control_bits(self._run(400, self.CP.minSteerSpeed - 1.5))))

    # ...and it does come on above minSteerSpeed, once the re-enable guard has expired
    self.assertTrue(self._control_bits(self._run(400, self.CP.minSteerSpeed + 1.0))[-1])

  def test_min_steer_speed_hysteresis(self):
    # minSteerSpeed is the stock drop-out (14.9 m/s); the control bit keeps the stock 1.1 m/s band
    # below it and falls at 13.8 m/s. Probe either side of 13.8, not either side of some wider band,
    # so that a wrong hysteresis value cannot pass.
    self.assertAlmostEqual(self.CP.minSteerSpeed, 14.9, places=5)
    self._run(300, self.CP.minSteerSpeed + 1.0)
    self.assertTrue(all(self._control_bits(self._run(50, 13.9))))
    self.assertFalse(any(self._control_bits(self._run(50, 13.7))))

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


class TestSuswHandover(SuswTestBase):
  def setUp(self):
    super().setUp()
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.chryslerSusw, 0)
    self.safety.init_tests()
    self.safety_packer = CANPackerSafety("chrysler_susw")
    self.t_us = 0

  def _step(self, *, acc_engaged: bool, lanesense_on: bool, lat_active: bool, v_ego: float) -> int:
    self.safety.set_timer(self.t_us)

    # Pack one physical copy of every safety-checked message, then feed those same bytes to both
    # layers. LaneSense goes first so its state is current when the 100 Hz ACC frame evaluates G3.
    rx_values = (
      ("LKA_HUD_2", 2, {"LANESENSE_DISABLED": int(not lanesense_on)}),
      ("ABS_3", 0, {"BRAKE_PEDAL_SWITCH": 0}),
      ("ABS_6", 0, {"VEHICLE_SPEED": v_ego}),
      ("EPS_2", 0, {"DRIVER_TORQUE": 0}),
      ("ACCEL_PEDAL_DRIVER", 0, {"ACCEL_PEDAL_DRIVER": 0}),
      ("CRUISE_BUTTONS", 1, {}),
      ("ACC_STATUS_1", 1, {"ACC_ENGAGED": int(acc_engaged)}),
    )
    packed = {}
    for name, bus, values in rx_values:
      addr, dat, packed_bus = self.safety_packer.make_can_msg(name, bus, values)
      self.assertEqual(bus, packed_bus)
      self.assertTrue(self.safety.safety_rx_hook(make_msg(bus, addr, len(dat), dat)), name)
      packed[name] = dat.hex()

    pt = DRIVING | {name: packed[name] for name in ("ABS_3", "ABS_6", "EPS_2", "ACCEL_PEDAL_DRIVER")}
    adas = {
      "ACC_HUD": HUD_ENGAGED_60 if acc_engaged else HUD_READY,
      "ACC_STATUS_1": packed["ACC_STATUS_1"],
      "CRUISE_BUTTONS": packed["CRUISE_BUTTONS"],
    }
    CS = self.update(pt, adas, {"LKA_HUD_2": packed["LKA_HUD_2"]})
    self.assertEqual(acc_engaged and lanesense_on, self.safety.get_controls_allowed())
    self.assertEqual(acc_engaged and lanesense_on, CS.cruiseState.enabled)

    CC = structs.CarControl()
    CC.enabled = CS.cruiseState.enabled
    CC.latActive = lat_active
    CC.actuators.torque = 0.

    _, can_sends = self.CI.apply(CC.as_reader(), self.nanos)
    accepted_op = 0
    for addr, dat, bus in can_sends:
      if bus == 0:
        accepted = self.safety.safety_tx_hook(make_msg(bus, addr, len(dat), dat))
        if (addr == 0x1f6) and accepted:
          accepted_op += 1

    self.assertLessEqual(accepted_op, 1)
    stock_forwarded = self.safety.safety_fwd_hook(2, 0x1f6) == 0
    self.t_us += 10000
    return accepted_op + int(stock_forwarded)

  def _phase(self, frames: int = 30, **kwargs) -> list[int]:
    return [self._step(**kwargs) for _ in range(frames)]

  def test_acc_handover_has_one_sender(self):
    off_before = self._phase(acc_engaged=False, lanesense_on=True, lat_active=False, v_ego=20.)
    on = self._phase(acc_engaged=True, lanesense_on=True, lat_active=True, v_ego=20.)
    off_after = self._phase(acc_engaged=False, lanesense_on=True, lat_active=False, v_ego=20.)
    self.assertEqual(off_before + on + off_after, [1] * 90)

  def test_lanesense_handover_has_one_sender(self):
    off_before = self._phase(acc_engaged=True, lanesense_on=False, lat_active=False, v_ego=20.)
    on = self._phase(acc_engaged=True, lanesense_on=True, lat_active=True, v_ego=20.)
    off_after = self._phase(acc_engaged=True, lanesense_on=False, lat_active=False, v_ego=20.)
    self.assertEqual(off_before + on + off_after, [1] * 90)

  def test_lat_active_handover_gap_is_bounded_by_the_timeout(self):
    inactive_before = self._phase(acc_engaged=True, lanesense_on=True, lat_active=False, v_ego=20.)
    active = self._phase(acc_engaged=True, lanesense_on=True, lat_active=True, v_ego=20.)
    inactive_after = self._phase(acc_engaged=True, lanesense_on=True, lat_active=False, v_ego=20.)

    # exactly one sender while inactive (the defect this closes: stock used to be blocked here) and
    # while active, including the hand-over frame itself
    self.assertEqual(inactive_before + active, [1] * 60)
    # Hand-back is the accepted residual of the accepted-stream gate: openpilot goes silent at once
    # and stock stays blocked until its last accepted command is CHRYSLER_SUSW_LKAS_TX_TIMEOUT old,
    # so the EPS sees a gap of at most timeout - 1 frame (4 x 10 ms at the provisional 50 ms) and
    # never two senders. The final bound comes from the parked-EPS gap measurement (AH-148).
    self.assertEqual(inactive_after[:4], [0] * 4)
    self.assertEqual(inactive_after[4:], [1] * 26)

  def test_min_steer_speed_handover_gap_is_bounded_by_the_timeout(self):
    phases = []
    for v_ego in (14., 17., 14.):
      phases.append(self._phase(acc_engaged=True, lanesense_on=True,
                                lat_active=v_ego >= self.CP.minSteerSpeed, v_ego=v_ego))

    # below minSteerSpeed stock steers (one sender, no gap); crossing up hands over cleanly; crossing
    # back down leaves the same bounded hand-back gap as test_lat_active_handover_gap_is_bounded_by_the_timeout
    self.assertEqual(phases[0] + phases[1], [1] * 60)
    self.assertEqual(phases[2][:4], [0] * 4)
    self.assertEqual(phases[2][4:], [1] * 26)


class TestSuswDbc(unittest.TestCase):
  """The three DBC corrections from analysis/carstate-evidence.md, checked on captured frames."""

  @staticmethod
  def _decode(name: str, addr: int, frame: str, bus: int = 0) -> dict:
    cp = CANParser("chrysler_susw", [(name, 0)], bus)
    cp.update([(0, [(addr, bytes.fromhex(frame), bus)])])
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

  def test_inertial_scale(self):
    # 0.01916 m/s2 per count, not the 0.01 originally declared, which understated acceleration
    # 1.9x and capped an hour of driving with eleven stops at -2.34 m/s2. Both turns now read a
    # physical cornering acceleration instead of an implausibly gentle one.
    left = self._decode("ABS_2", 0xfe, ABS_2_LEFT_TURN)
    right = self._decode("ABS_2", 0xfe, ABS_2_RIGHT_TURN)
    self.assertAlmostEqual(left["LATERAL_ACCEL"], 2.7428, places=4)
    self.assertAlmostEqual(right["LATERAL_ACCEL"], -2.2196, places=4)
    self.assertAlmostEqual(left["LONG_ACCEL"], 1.0343, places=4)

    # the lateral zero is NOT mid-scale: a joint scale-and-zero fit puts it at 2055-2057 counts on
    # four routes on four different days, so the offset is -39.39 rather than LONG_ACCEL's -39.24.
    # Reading it at mid-scale would carry a constant +0.15 m/s2 bias.
    self.assertAlmostEqual(left["LATERAL_ACCEL"] - (2199 * 0.01916 - 39.24), -0.15, places=2)

    # a_lat = v * yaw_rate is the independent check the scale was calibrated against
    for frame, speed in ((ABS_2_LEFT_TURN, 8.5), (ABS_2_RIGHT_TURN, 4.9)):
      with self.subTest(frame=frame):
        vl = self._decode("ABS_2", 0xfe, frame)
        self.assertAlmostEqual(vl["LATERAL_ACCEL"], speed * vl["YAW_RATE"], delta=0.5)

  def test_odometer(self):
    # 20-bit count across byte2, byte3 and byte4's high nibble; 1 km per count (inferred, see the
    # DBC comment) and monotone within and across routes
    boot = self._decode("ODOMETER", 0x259, "00000a3f50800000")     # t=0.6, still invalid
    self.assertEqual((boot["ODOMETER"], boot["ODOMETER_INVALID"]), (41973, 1))
    late = self._decode("ODOMETER", 0x259, "00000a4210000000")     # t=3000
    self.assertEqual((late["ODOMETER"], late["ODOMETER_INVALID"]), (42017, 0))
    self.assertGreater(late["ODOMETER"], boot["ODOMETER"])

  def test_vehicle_status(self):
    for frame, moving, ignition in (("00e00000000004a6", 1, 1),      # byte1 0xe0, driving
                                    ("0060000000000df3", 0, 1),      # byte1 0x60, stopped
                                    ("00200000000008ca", 0, 0)):     # byte1 0x20, last 0.24 s
      with self.subTest(frame=frame):
        vl = self._decode("VEHICLE_STATUS", 0xf1, frame)
        self.assertEqual((vl["VEHICLE_MOVING"], vl["IGNITION_RUN"]), (moving, ignition))

  def test_vin(self):
    # byte 0 is a 3-frame multiplex index, bytes 1-7 are ASCII; not a DBC multiplex because the
    # 56-bit concatenation would not survive float64 signal values
    vin = ""
    for frame, mux in (("005a41434e4a4444", 0), ("0131305050503230", 1), ("0232313800000000", 2)):
      vl = self._decode("VIN", 0x416, frame)
      self.assertEqual(vl["VIN_MUX"], mux)
      vin += "".join(chr(int(vl[f"VIN_BYTE_{i}"])) for i in range(1, 8) if vl[f"VIN_BYTE_{i}"])
    self.assertEqual(len(vin), 17)
    self.assertTrue(vin.startswith("ZAC"))          # WMI: Jeep Italy, the Renegade line
    self.assertEqual(vin[9], "P")                   # model year 2023

  def test_battery_is_raw_counts(self):
    # deliberately undeclared scale: 0.0703 V/count is a back-fit from assuming 14.2 V at the
    # charging plateau, never a measurement, so the signal stays in counts
    vl = self._decode("BATTERY", 0x41a, "c6c200a4a04920")
    self.assertEqual(vl["BATTERY_VOLTAGE_RAW"], 198)

  def test_parking_brake_transition(self):
    for frame, state in ((GEAR2_PB_ENGAGED, 5), (GEAR2_PB_APPLYING, 2), (GEAR2_D, 0)):
      with self.subTest(frame=frame):
        self.assertEqual(self._decode("GEAR_2", 0x5a9, frame)["PARKING_BRAKE_TRANSITION"], state)

  def test_engine_torque(self):
    # the former UNKNOWN_4 (declared little-endian by mistake) is the top of a 10-bit offset-binary
    # torque field; only the 10-bit reading is continuous
    for frame, rpm, torque in (("0df0c002084640fb", 892, 50),      # idle
                               ("272cc2c2084529d7", 2507, 41),
                               ("1b9cc1c208420fa8", 1767, 16)):
      with self.subTest(frame=frame):
        vl = self._decode("ENGINE_1", 0xfc, frame)
        self.assertEqual((vl["ENGINE_RPM"], vl["ENGINE_TORQUE"], vl["ENGINE_STOPPED"]), (rpm, torque, 0))

  def test_eps_2_status_bits(self):
    # NOT_REVERSE and LKA_LOW_SPEED_INHIBIT were UNKNOWN_STATUS and UNKNOWN_2
    quiet = self._decode("EPS_2", 0x106, EPS_2_QUIET)              # parked, so under the floor
    self.assertEqual((quiet["NOT_REVERSE"], quiet["LKA_LOW_SPEED_INHIBIT"]), (1, 1))
    driving = self._decode("EPS_2", 0x106, DRIVING["EPS_2"])       # t=1200, 15.9 m/s = 57 km/h
    self.assertEqual((driving["NOT_REVERSE"], driving["LKA_LOW_SPEED_INHIBIT"]), (1, 0))

  def test_lka_low_speed_inhibit_threshold(self):
    # Name adopted from brisk-otter's round-1 review; re-verified here against ABS_6.VEHICLE_SPEED.
    # Each entry is the two consecutive 0x106 frames straddling one transition on route 000000e7 plus
    # the 0x101 frame in force at each instant. Every transition in both drives happens inside the
    # 13.3-13.8 m/s band; the ABS_6 quantum is 0.017 m/s and the EPS clearly filters its own speed, so
    # the claim under test is the band and the polarity, not a sample-exact threshold.
    for label, eps_set, eps_clear, abs_set, abs_clear in EPS_2_INHIBIT_EDGES:
      with self.subTest(edge=label):
        self.assertEqual(self._decode("EPS_2", 0x106, eps_set)["LKA_LOW_SPEED_INHIBIT"], 1)
        self.assertEqual(self._decode("EPS_2", 0x106, eps_clear)["LKA_LOW_SPEED_INHIBIT"], 0)
        for frame in (abs_set, abs_clear):
          v = self._decode("ABS_6", 0x101, frame)["VEHICLE_SPEED"]
          self.assertGreater(v, 13.3)
          self.assertLess(v, 13.8)

  def test_abs_6_counter(self):
    # the field previously declared as BRAKE_PRESSURE_2 (43|12) is byte5's zero nibble plus the
    # message counter, and consecutive captured frames step it by one
    counters = [self._decode("ABS_6", 0x101, f)["COUNTER"] for f in
                ("0075000000000138", "0075400000000280", "007560000000035c")]
    self.assertEqual(counters, [1, 2, 3])

  def test_clock_carries_the_date(self):
    vl = self._decode("CLOCK", 0x73a, "171323082026")
    bcd = {k: (int(v) >> 4) * 10 + (int(v) & 0xf) for k, v in vl.items()}
    self.assertEqual((bcd["CENTURY_BCD"], bcd["YEAR_BCD"], bcd["MONTH_BCD"], bcd["DAY_BCD"]), (20, 26, 8, 23))
    self.assertEqual((bcd["HOUR_BCD"], bcd["MINUTE_BCD"]), (17, 13))   # local time, route is 21:13 UTC

  def test_acc_command(self):
    idle = self._decode("ACC_COMMAND", 0x15c, "320001000000032b")
    self.assertEqual((idle["ACC_DECEL_REQUEST"], idle["ACC_DECEL_ACTIVE"]), (1600, 0))
    self.assertEqual((idle["ACC_ACCEL_REQUEST"], idle["ACC_ENGAGED"]), (0, 0))

    # the accel request is 11 bits: byte3 plus byte4 bits 7-5, which an earlier reading had as a
    # separate rolling counter. byte3 alone would read 70 here.
    accel = self._decode("ACC_COMMAND", 0x15c, "3200014610000d68")
    self.assertEqual((accel["ACC_ACCEL_REQUEST"], accel["ACC_ENGAGED"]), (560, 1))
    self.assertEqual(accel["ACC_DECEL_ACTIVE"], 0)

    # accel and decel are mutually exclusive
    decel = self._decode("ACC_COMMAND", 0x15c, "b01901001000026d")
    self.assertEqual((decel["ACC_DECEL_REQUEST"], decel["ACC_DECEL_ACTIVE"]), (5635, 1))
    self.assertEqual(decel["ACC_ACCEL_REQUEST"], 0)
    self.assertEqual(decel["ACC_ENGAGED"], 1)

  def test_engage_bit_corroborated_by_0x15c(self):
    """ACC_STATUS_1 (0x103) and ACC_COMMAND (0x15c) must agree about ACC_ENGAGED.

    Cross-message corroboration idea from brisk-otter's round-1 review, which validated the same
    claim on its own data (14/14 intervals, 17 ms median delta). This is a DBC/evidence regression
    test only: 0x15c is not in the RPGW whitelist, so openpilot never sees it at runtime and no
    CarState behavior depends on it. What it locks down is that the engage bit the port DOES use -
    ACC_STATUS_1 bit 21, the one pcm_cruise_check() keys off - is corroborated by an independent
    message that was decoded separately.
    """
    engaged = 0
    for t, frame_103, frame_15c, expected in ACC_ENGAGE_PAIRS:
      with self.subTest(t=t):
        status_1 = self._decode("ACC_STATUS_1", 0x103, frame_103)["ACC_ENGAGED"]
        command = self._decode("ACC_COMMAND", 0x15c, frame_15c)["ACC_ENGAGED"]
        self.assertEqual(status_1, command)
        self.assertEqual(status_1, expected)
      engaged += expected

    # the fixture straddles all 14 engagements, so it must be balanced and cover both states
    self.assertEqual(len(ACC_ENGAGE_PAIRS), 56)
    self.assertEqual(engaged, 28)

  def test_radar_track(self):
    # the private fusion bus. 0x2c0 is the ACC lead slot.
    lead = self._decode("RADAR_TRACK_1", 0x2c0, "23f80427049050ea", bus=1)   # t=2106.6 "lead present"
    self.assertEqual(lead["TRACK_STATUS"], 1)
    self.assertEqual((lead["AZIMUTH_LO"], lead["AZIMUTH_HI"]), (-8, 39))     # straddles boresight
    self.assertAlmostEqual(lead["RANGE"], 73.0, places=2)
    self.assertLess(lead["AZIMUTH_LO"], 0)
    self.assertGreater(lead["AZIMUTH_HI"], 0)

    empty = self._decode("RADAR_TRACK_1", 0x2c0, "0400040000006042", bus=1)
    self.assertEqual(empty["TRACK_STATUS"], 0)
    self.assertEqual((empty["AZIMUTH_LO"], empty["AZIMUTH_HI"], empty["RANGE"]), (0, 0, 0))

    # the counter is in byte 6's HIGH nibble on fusion, the opposite of raw CAN C
    self.assertEqual(lead["COUNTER"], 5)

  def test_track_status_is_one_hot(self):
    # a TRACK_VALID declared on bit 5 alone missed 2-8 % of real objects: this captured frame is a
    # genuine track at 50.4 m in state 2, which bit 5 reads as empty
    state2 = self._decode("RADAR_TRACK_1", 0x2c0, "427202cc03260065", bus=1)
    self.assertEqual(state2["TRACK_STATUS"], 2)
    self.assertAlmostEqual(state2["RANGE"], 50.375, places=3)
    self.assertEqual((state2["AZIMUTH_LO"], state2["AZIMUTH_HI"]), (-398, -308))
    self.assertEqual(int(state2["TRACK_STATUS"]) & 1, 0)          # the old bit-5 flag would say empty

  def test_lead_status_enum(self):
    # byte0 high nibble, not a bit-7 flag: bit 7 misses the whole value-5 state
    for frame, status in (("00000104d8005044", 0), ("500001271d00707d", 5)):
      with self.subTest(frame=frame):
        self.assertEqual(self._decode("RADAR_STATUS", 0x200, frame, bus=1)["LEAD_STATUS"], status)
    define = CANDefine("chrysler_susw")
    lead = define.dv["RADAR_STATUS"]["LEAD_STATUS"]
    self.assertEqual((lead[0], lead[9]), ("NONE", "LEAD"))

  def test_radar_track_ext(self):
    # 0x2a0 is the other half of the same object record as 0x2c0, joined on COUNTER
    occ = self._decode("RADAR_TRACK_1_EXT", 0x2a0, "ff05ffdc00000005", bus=1)
    self.assertEqual(occ["TRACK_PRESENT"], 255)
    self.assertEqual(occ["TRACK_ID"], 5)
    self.assertEqual(occ["INV_TTC"], -1)                     # signed, positive = closing
    self.assertEqual(occ["EXT_RESERVED"], 0)

    idle = self._decode("RADAR_TRACK_1_EXT", 0x2a0, "000000740000602b", bus=1)
    self.assertEqual((idle["TRACK_PRESENT"], idle["TRACK_ID"], idle["INV_TTC"]), (0, 0, 0))

    # INV_TTC is signed: the field must reach negative values, not wrap to 255
    self.assertLess(occ["INV_TTC"], 0)

  def test_steering_levers_layouts_are_disjoint(self):
    # the CH and raw CAN C versions of 0x73e are different messages that share one definition
    ch = self._decode("STEERING_LEVERS", 0x73e, "00000300")               # comma bus 0, hazards
    self.assertEqual(ch["TURN_SIGNALS"], 3)
    self.assertEqual([ch[k] for k in ("PARK_LAMPS", "PARK_LAMPS_2", "BRAKE_LAMPS",
                                      "TURN_LAMP_LEFT", "TURN_LAMP_RIGHT")], [0] * 5)

    # raw CAN C frames: every lamp bit lives outside TURN_SIGNALS, which reads 0 there
    for frame, lamp in (("00000400", "BRAKE_LAMPS"), ("00800000", "TURN_LAMP_LEFT"),
                        ("00002000", "TURN_LAMP_RIGHT"), ("02008000", "PARK_LAMPS")):
      with self.subTest(frame=frame):
        vl = self._decode("STEERING_LEVERS", 0x73e, frame)
        self.assertEqual(vl[lamp], 1)
        self.assertEqual(vl["TURN_SIGNALS"], 0)
    # PARK_LAMPS_2 is an exact duplicate of PARK_LAMPS in every captured frame
    park = self._decode("STEERING_LEVERS", 0x73e, "02008000")
    self.assertEqual(park["PARK_LAMPS"], park["PARK_LAMPS_2"])

  def test_radar_status_time_base(self):
    # bytes 3-4 ramp at 36.06 counts/s independently of speed, so it is a time base
    a = self._decode("RADAR_STATUS", 0x200, "00000104d8005044", bus=1)   # t=41.94
    b = self._decode("RADAR_STATUS", 0x200, "000001100f00f0bc", bus=1)   # t=121.70
    self.assertEqual((a["TIME_BASE"], b["TIME_BASE"]), (1240, 4111))
    self.assertAlmostEqual((b["TIME_BASE"] - a["TIME_BASE"]) / (121.70 - 41.94), 36.0, delta=0.5)

  def test_acc_gap_lead_enum(self):
    define = CANDefine("chrysler_susw")
    gap = define.dv["ACC_HUD"]["ACC_GAP_LEAD_STATE"]
    self.assertEqual(gap[1], "OFF")
    self.assertEqual((gap[2], gap[6], gap[10], gap[14]),
                     ("READY_GAP_1", "LEAD_GAP_1", "NO_LEAD_GAP_1", "STANDBY_GAP_1"))
    # gap index is consistent across the four banks
    for base in (2, 6, 10, 14):
      for i in range(4):
        self.assertTrue(gap[base + i].endswith(f"_{i + 1}"))
    self.assertEqual(define.dv["LKA_HUD_2"]["LKA_HUD_STATE"][0], "LANESENSE_OFF")
    self.assertEqual(define.dv["ACC_HUD"]["ACC_STATE"][5], "STANDBY")

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

    # direction lives in the ROLLING_* bits. They were named per-wheel (ACTIVE_FL/FR/RL/RR) but
    # bits 32/34 are one signal sent twice and 33/35 are its reverse counterpart, so no axle is
    # claimed: they are numbered 1 and 2.
    fwd = self._decode("ABS_5", 0x116, ABS_5_FORWARD)
    self.assertEqual([fwd[k] for k in ("ROLLING_FORWARD_1", "ROLLING_REVERSE_1",
                                       "ROLLING_FORWARD_2", "ROLLING_REVERSE_2")], [1, 0, 1, 0])
    rev = self._decode("ABS_5", 0x116, ABS_5_REVERSE)
    self.assertEqual([rev[k] for k in ("ROLLING_FORWARD_1", "ROLLING_REVERSE_1",
                                       "ROLLING_FORWARD_2", "ROLLING_REVERSE_2")], [0, 1, 0, 1])


class TestSuswFingerprint(unittest.TestCase):
  """The two CAN fingerprint dicts, and what each one survives.

  Robustness against trim variation was raised by brisk-otter in round 1 as a fuzzy/subset fallback.
  opendbc has no fuzzy CAN fingerprint in this version - see the comment in chrysler/fingerprints.py -
  so the answer here is a second dict on the private fusion bus, which can_fingerprint() eliminates
  independently of bus 0. These tests pin down that both dicts identify the platform on their own,
  that a body-ID-only census loss still matches, and that neither dict makes any other platform
  ambiguous.
  """

  BUS_0, BUS_1 = FINGERPRINTS[CAR.JEEP_RENEGADE]

  # every ID the port's CarState needs. A trim that dropped any of these would not be portable at all,
  # so these are the IDs the fingerprint is really keyed on.
  LOAD_BEARING_BUS_0 = dict(sorted({0xde: 6, 0xee: 8, 0xfa: 8, 0xfc: 8, 0xfe: 8, 0x101: 8, 0x106: 7,
                                    0x116: 8, 0x1f0: 8, 0x1f6: 4, 0x257: 8, 0x4b1: 8, 0x547: 8,
                                    0x5a9: 8, 0x73e: 4}.items()))

  @staticmethod
  def _fingerprint(finger: dict[int, int], bus: int) -> str | None:
    can = [CanData(address=addr, dat=b"\x00" * dlc, src=bus) for addr, dlc in finger.items()]
    packets = iter([can])
    return can_fingerprint(lambda **kwargs: [next(packets, [])])[0]

  def test_both_dicts_identify_the_platform(self):
    self.assertEqual(self._fingerprint(self.BUS_0, 0), CAR.JEEP_RENEGADE)
    self.assertEqual(self._fingerprint(self.BUS_1, 1), CAR.JEEP_RENEGADE)

  def test_load_bearing_ids_alone_still_match(self):
    # matching is elimination-based, so a trim that DROPS body IDs still matches as long as no other
    # platform survives the same census. Down to the 15 IDs CarState actually reads, it is still unique.
    self.assertEqual(self._fingerprint(self.LOAD_BEARING_BUS_0, 0), CAR.JEEP_RENEGADE)

  def test_fusion_bus_matches_without_the_gateway(self):
    # the fusion fallback has to work in both gateway states: in BYPASS the comma sees only the radar
    # burst, in INTERCEPT it also sees the three copied ACC IDs and GATEWAY_HEARTBEAT.
    gateway_ids = (0x103, 0x2fa, 0x5f1, 0x73c)
    for addr in gateway_ids:
      self.assertIsNotNone(self.BUS_1.get(addr), f"{hex(addr)} missing from the bus 1 fingerprint")
    stock = {a: d for a, d in self.BUS_1.items() if a not in gateway_ids}
    self.assertEqual(self._fingerprint(stock, 1), CAR.JEEP_RENEGADE)

  def test_load_bearing_ids_are_in_the_census(self):
    for addr, dlc in self.LOAD_BEARING_BUS_0.items():
      with self.subTest(addr=hex(addr)):
        self.assertEqual(self.BUS_0.get(addr), dlc)
    for addr in list(PT_ADDRS.values()) + list(CAM_ADDRS.values()):
      self.assertIsNotNone(self.BUS_0.get(addr), f"{hex(addr)} is parsed but missing from the bus 0 fingerprint")
    for addr in ADAS_ADDRS.values():
      self.assertIsNotNone(self.BUS_1.get(addr), f"{hex(addr)} is parsed but missing from the bus 1 fingerprint")

  def test_no_other_platform_becomes_ambiguous(self):
    # widening a platform's accept-set can only hurt other platforms, so check the whole catalog
    for car_model, fingerprints in ALL_FINGERPRINTS.items():
      for i, fingerprint in enumerate(fingerprints):
        with self.subTest(car_model=car_model, fingerprint=i):
          self.assertEqual(self._fingerprint(fingerprint, 0), car_model)


if __name__ == "__main__":
  unittest.main()
