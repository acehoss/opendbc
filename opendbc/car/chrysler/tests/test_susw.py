import unittest

from opendbc.can import CANPacker
from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.chrysler.chryslercan import create_lkas_command
from opendbc.car.chrysler.interface import CarInterface
from opendbc.car.chrysler.values import CAR, CarControllerParams

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter

# Addresses of every message the SUSW port reads, see opendbc/dbc/chrysler_susw.dbc.
# Bus 0 is the camera-side "CAN CH" bus, bus 1 is the private fusion bus fed by the gateway.
PT_ADDRS = {"EPS_1": 0xde, "ABS_1": 0xee, "ABS_3": 0xfa, "ENGINE_1": 0xfc,
            "ABS_6": 0x101, "EPS_2": 0x106, "DOORS": 0x4b1, "STEERING_LEVERS": 0x73e}
ADAS_ADDRS = {"ACC_STATUS_1": 0x103, "CRUISE_BUTTONS": 0x2fa, "ACC_HUD": 0x73c}

# All raw frames below were captured from the 2023 Jeep Renegade on route
# 873a474e9ad72abb|000000e7--c05680fea1 and its paired raw CAN C capture.

# t=1200 s, cruising at ~16 m/s with the driver holding the wheel
DRIVING = {
  "EPS_1": "1c2697cf0b02",
  "ABS_1": "1d68e987543a606f",
  "ABS_3": "80200000480800b5",
  "ENGINE_1": "1b9cc1c208420fa8",
  "ABS_6": "00754000000000ba",
  "EPS_2": "7c737060400b15",
  "DOORS": "0000000000000000",
  "STEERING_LEVERS": "00000000",
}

# Isolated parked body-state sweep
BRAKE_PRESSED = "7c200000080800c7"      # ABS_3 with BRAKE_PEDAL_SWITCH set
REVERSE = "0e10c0060841ee74"            # ENGINE_1 in reverse, accelerator released
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

# ACC messages, raw CAN C
ACC_OFF = "000003e80000082b"            # ACC_STATUS_1, ACC_ENGAGED = 0
ACC_ENGAGED = "0000200200000c70"        # ACC_STATUS_1, ACC_ENGAGED = 1
HUD_OFF = "009fc00000000104"            # ACC_HUD, ACC_STATE 0, no set speed
HUD_READY = "009fc000000dc314"          # ACC_HUD, ACC_STATE 1, no set speed
HUD_ENGAGED_60 = "009fc03c250fcb24"     # ACC_HUD, ACC_STATE 2, set speed 60 km/h
HUD_STANDBY_64 = "009fc04028119154"     # ACC_HUD, ACC_STATE 5, set speed 64 km/h retained
BTN_NONE = "00085600"
BTN_MAIN = "010a2000"                   # ACC_ON_OFF
BTN_RESUME = "080c7800"                 # ACC_RESUME
BTN_SET_DECEL = "10023000"              # ACC_SET_DECEL
BTN_ACCEL = "200a1900"                  # ACC_ACCEL
BTN_GAP_DEC = "400ef200"                # ACC_DISTANCE_DEC
BTN_GAP_INC = "0085f100"                # ACC_DISTANCE_INC, byte 1 bit 7


class SuswTestBase(unittest.TestCase):
  def setUp(self):
    self.CP = CarInterface.get_non_essential_params(CAR.JEEP_RENEGADE)
    self.CI = CarInterface(self.CP)
    self.nanos = 0

  def update(self, pt: dict | None = None, adas: dict | None = None) -> structs.CarState:
    frames = [CanData(PT_ADDRS[n], bytes.fromhex(h), 0) for n, h in (pt or {}).items()]
    frames += [CanData(ADAS_ADDRS[n], bytes.fromhex(h), 1) for n, h in (adas or {}).items()]
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
    self.assertAlmostEqual(CS.steeringRateDeg, -0.5, places=3)
    self.assertEqual(CS.steeringTorque, -125)      # EPS_2.DRIVER_TORQUE
    self.assertEqual(CS.steeringTorqueEps, -9)     # EPS_2.TORQUE_MOTOR
    self.assertTrue(CS.steeringPressed)            # |125| > STEER_THRESHOLD
    self.assertFalse(CS.steerFaultPermanent)

    self.assertFalse(CS.brakePressed)
    self.assertTrue(CS.gasPressed)                 # ENGINE_1.ACCEL_PEDAL = 5.6 %
    self.assertFalse(CS.doorOpen)
    self.assertFalse(CS.leftBlinker)
    self.assertFalse(CS.rightBlinker)
    self.assertEqual(CS.gearShifter, GearShifter.drive)

    # documented gaps: no seatbelt signal is decoded and BSM is not on a bus openpilot sees
    self.assertFalse(CS.seatbeltUnlatched)
    self.assertFalse(self.CP.enableBsm)

  def test_brake_and_reverse(self):
    CS = self.update(DRIVING | {"ABS_3": BRAKE_PRESSED, "ENGINE_1": REVERSE})
    self.assertTrue(CS.brakePressed)
    self.assertFalse(CS.gasPressed)
    self.assertEqual(CS.gearShifter, GearShifter.reverse)

  def test_blinkers(self):
    for levers, left, right in ((LEVERS_LEFT, True, False), (LEVERS_RIGHT, False, True),
                                (LEVERS_HAZARDS, True, True), ("00000000", False, False)):
      with self.subTest(levers=levers):
        self.setUp()
        CS = self.update(DRIVING | {"STEERING_LEVERS": levers})
        self.assertEqual(CS.leftBlinker, left)
        self.assertEqual(CS.rightBlinker, right)

  def test_doors(self):
    for doors, expected in ((DOOR_FL_OPEN, True), (DOOR_FR_OPEN, True), ("0000000000000000", False)):
      with self.subTest(doors=doors):
        self.setUp()
        CS = self.update(DRIVING | {"DOORS": doors})
        self.assertEqual(CS.doorOpen, expected)

  def test_eps_sentinel_holds_last_value(self):
    quiet = DRIVING | {"EPS_2": EPS_2_QUIET}
    CS = self.update(quiet)
    self.assertAlmostEqual(CS.steeringAngleDeg, 3.8, places=3)
    self.assertAlmostEqual(CS.steeringRateDeg, -0.5, places=3)
    self.assertEqual(CS.steeringTorque, 11)
    self.assertFalse(CS.steeringPressed)

    # the EPS all-ones sentinel and the saturated driver torque are both discarded
    CS = self.update(quiet | {"EPS_1": EPS_1_SENTINEL, "EPS_2": EPS_2_SATURATED})
    self.assertAlmostEqual(CS.steeringAngleDeg, 3.8, places=3)
    self.assertAlmostEqual(CS.steeringRateDeg, -0.5, places=3)
    self.assertEqual(CS.steeringTorque, 11)
    self.assertFalse(CS.steeringPressed)

    # TORQUE_MOTOR is not sentinel protected, it still tracks the saturated frame
    self.assertEqual(CS.steeringTorqueEps, -947)

  def test_eps_sentinel_on_first_frame(self):
    # nothing valid has been seen yet, so the held values are zero and the driver is not pressing
    CS = self.update(DRIVING | {"EPS_1": EPS_1_SENTINEL, "EPS_2": EPS_2_SATURATED})
    self.assertEqual(CS.steeringAngleDeg, 0.)
    self.assertEqual(CS.steeringRateDeg, 0.)
    self.assertEqual(CS.steeringTorque, 0.)
    self.assertFalse(CS.steeringPressed)

  def test_cruise_state(self):
    cases = (
      (HUD_OFF, ACC_OFF, False, False, 0.),
      (HUD_READY, ACC_OFF, True, False, 0.),
      (HUD_ENGAGED_60, ACC_ENGAGED, True, True, 60 / 3.6),
      (HUD_STANDBY_64, ACC_OFF, True, False, 64 / 3.6),
    )
    for hud, status, available, enabled, speed in cases:
      with self.subTest(hud=hud):
        self.setUp()
        CS = self.update(DRIVING, {"ACC_HUD": hud, "ACC_STATUS_1": status, "CRUISE_BUTTONS": BTN_NONE})
        self.assertEqual(CS.cruiseState.available, available)
        self.assertEqual(CS.cruiseState.enabled, enabled)
        self.assertAlmostEqual(CS.cruiseState.speed, speed, places=4)
        self.assertFalse(CS.cruiseState.nonAdaptive)

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

  def test_control_bit_is_one(self):
    # RAM uses 2 in LKAS_CONTROL_BIT, SUSW (like CUSW) uses 1
    self.packer.counters[0x1f6] = 0
    dat = create_lkas_command(self.packer, self.CP, 0, True)[1]
    self.assertEqual(dat[1] & 0x30, 0x10)


class TestSuswCarController(SuswTestBase):
  def _run(self, frames: int, v_ego: float, lat_active: bool = True, cancel: bool = False, resume: bool = False):
    CC = structs.CarControl()
    CC.enabled = True
    CC.latActive = lat_active
    CC.cruiseControl.cancel = cancel
    CC.cruiseControl.resume = resume
    CC.actuators.torque = 0.5
    CC = CC.as_reader()

    self.update(DRIVING, {"ACC_HUD": HUD_ENGAGED_60, "ACC_STATUS_1": ACC_ENGAGED, "CRUISE_BUTTONS": BTN_NONE})

    sent = []
    for _ in range(frames):
      self.CI.CS.out.vEgo = v_ego
      _, can_sends = self.CI.apply(CC, self.nanos)
      self.nanos += int(DT_CTRL * 1e9)
      sent.append(can_sends)
    return sent

  def test_only_lkas_command_is_sent(self):
    # no cruise button TX and no HUD TX for SUSW, even when the controller asks to cancel and resume
    sent = self._run(300, 20., cancel=True, resume=True)
    for can_sends in sent:
      self.assertEqual([(addr, bus) for addr, _, bus in can_sends], [(0x1f6, 0)])

  def test_steer_step_is_100hz(self):
    self.assertEqual(CarControllerParams(self.CP).STEER_STEP, 1)
    self.assertEqual(CarControllerParams(self.CP).STEER_MAX, 250)

  def test_control_bit_below_min_steer_speed(self):
    # the control bit never comes on below minSteerSpeed, however long we drive
    sent = self._run(400, self.CP.minSteerSpeed - 3.0)
    self.assertTrue(all(dat[1] & 0x30 == 0 for can_sends in sent for _, dat, _ in can_sends))

    # ...and it does come on above it, once the re-enable guard has expired
    sent = self._run(400, self.CP.minSteerSpeed + 1.0)
    self.assertEqual(sent[-1][0][1][1] & 0x30, 0x10)

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
  def _control_bits(sent):
    return [bool(dat[1] & 0x10) for can_sends in sent for _, dat, _ in can_sends]


if __name__ == "__main__":
  unittest.main()
