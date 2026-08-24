from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.chrysler.values import CUSW_CARS, DBC, STEER_THRESHOLD, RAM_CARS, SUSW_CARS
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase

ButtonType = structs.CarState.ButtonEvent.Type

# SUSW CRUISE_BUTTONS (0x2fa) is a set of independent bits. Only one at a time was ever observed,
# but that is not guaranteed, so they collapse into a single index in priority order for
# create_button_events(). ACC_CANCEL is checked first: a simultaneous press must never swallow the
# driver's cancel. No ACC_CANCEL press was captured on either long route, so it is also the least
# proven bit in the map.
SUSW_BUTTON_SIGNALS = ("ACC_CANCEL", "ACC_ON_OFF", "ACC_ACCEL", "ACC_SET_DECEL", "ACC_RESUME")
SUSW_BUTTONS = {
  1: ButtonType.cancel,           # ACC_CANCEL
  2: ButtonType.mainCruise,       # ACC_ON_OFF
  3: ButtonType.accelCruise,      # ACC_ACCEL, RES+ / speed +
  4: ButtonType.decelCruise,      # ACC_SET_DECEL, SET / speed -
  5: ButtonType.resumeCruise,     # ACC_RESUME
  6: ButtonType.gapAdjustCruise,  # ACC_DISTANCE_DEC and ACC_DISTANCE_INC, two separate buttons on this car
}

# On bus wake the SUSW EPS publishes the all-ones code extreme instead of a measurement in the
# first EPS_1 frames of a route (STEERING_ANGLE raw 0x3fff, STEERING_RATE raw 0xfff), so those are
# held at the last valid value. Compared with < rather than == because they are the code extremes:
# nothing valid can reach them.
# EPS_2.DRIVER_TORQUE is deliberately NOT filtered. Its -1024 rail is a real saturated measurement
# at full right lock (the parked right-to-lock sweep reads mean -394.8, min -1024, and the left
# sweep peaks at +1022, so only the negative rail is reachable). Discarding it would hide the one
# rail a driver can actually hit, and SUSW rate-limits the command against this signal, so a stale
# value there means no override detection at exactly the moment the driver is fighting the wheel.
SUSW_STEER_ANGLE_INVALID = 921.5     # deg, EPS_1 STEERING_ANGLE raw 0x3fff
SUSW_STEER_RATE_INVALID = 2095.      # deg/s, EPS_1 STEERING_RATE raw 0xfff

# How many CarState cycles ACC_STATUS_1 may go without a fresh frame before the fusion bus counts
# as dead. CarState runs at 100 Hz and the gateway copies 0x103 at 100 Hz, so 50 cycles is 0.5 s.
SUSW_ACC_TIMEOUT_FRAMES = 50

# ABS_6.VEHICLE_SPEED is 11 bits and tops out here. No capture goes near it, so whether it clips or
# wraps above 125 km/h is unobserved; the ABS_1 wheel speeds are 13 bits and cannot wrap in this
# range, so they are the fallback. The threshold is far above any observed disagreement between the
# two (mean 0.022 m/s, p99 0.119, max 0.565 over a whole route).
SUSW_SPEED_MAX = 34.799              # m/s, ABS_6.VEHICLE_SPEED full scale
SUSW_SPEED_MISMATCH = 2.0            # m/s


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CP = CP
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])

    self.auto_high_beam = 0
    self.button_counter = 0
    self.lkas_car_model = -1
    # the stock camera's own LKAS_COMMAND counter, mirrored so openpilot can continue its sequence
    self.lkas_counter = 0
    self.susw_button = 0
    self.susw_steering_angle = 0.
    self.susw_steering_rate = 0.
    self.susw_acc_nanos = 0
    self.susw_acc_stale_frames = 0
    self.susw_acc_valid_prev = False
    self.susw_needs_reengage = False

    if CP.carFingerprint in RAM_CARS:
      self.shifter_values = can_define.dv["Transmission_Status"]["Gear_State"]
    elif CP.carFingerprint in SUSW_CARS:
      # GEAR (0x190) is raw CAN C only. GEAR_2 (0x5a9) is the CH-side gear report, same enum,
      # 99.775 % frame agreement with 0x190, and it is visible to the comma without forwarding.
      self.shifter_values = can_define.dv["GEAR_2"]["PRNDL"]
    else:
      self.shifter_values = can_define.dv["GEAR"]["PRNDL"]

    self.distance_button = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    if self.CP.carFingerprint in CUSW_CARS:
      return self.update_cusw(cp, cp_cam)

    if self.CP.carFingerprint in SUSW_CARS:
      return self.update_susw(cp, cp_cam, can_parsers[Bus.adas])

    ret = structs.CarState()

    prev_distance_button = self.distance_button
    self.distance_button = cp.vl["CRUISE_BUTTONS"]["ACC_Distance_Dec"]

    # lock info
    ret.doorOpen = any([cp.vl["BCM_1"]["DOOR_OPEN_FL"],
                        cp.vl["BCM_1"]["DOOR_OPEN_FR"],
                        cp.vl["BCM_1"]["DOOR_OPEN_RL"],
                        cp.vl["BCM_1"]["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = cp.vl["ORC_1"]["SEATBELT_DRIVER_UNLATCHED"] == 1

    # brake pedal
    ret.brakePressed = cp.vl["ESP_1"]['Brake_Pedal_State'] == 1  # Physical brake pedal switch

    # gas pedal
    ret.gasPressed = cp.vl["ECM_5"]["Accelerator_Position"] > 1e-5

    # car speed
    if self.CP.carFingerprint in RAM_CARS:
      ret.vEgoRaw = cp.vl["ESP_8"]["Vehicle_Speed"] * CV.KPH_TO_MS
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl["Transmission_Status"]["Gear_State"], None))
    else:
      ret.vEgoRaw = (cp.vl["SPEED_1"]["SPEED_LEFT"] + cp.vl["SPEED_1"]["SPEED_RIGHT"]) / 2.
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl["GEAR"]["PRNDL"], None))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = not ret.vEgoRaw > 0.001

    # button presses
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(200, cp.vl["STEERING_LEVERS"]["TURN_SIGNALS"] == 1,
                                                                       cp.vl["STEERING_LEVERS"]["TURN_SIGNALS"] == 2)
    ret.genericToggle = cp.vl["STEERING_LEVERS"]["HIGH_BEAM_PRESSED"] == 1

    # steering wheel
    ret.steeringAngleDeg = cp.vl["STEERING"]["STEERING_ANGLE"] + cp.vl["STEERING"]["STEERING_ANGLE_HP"]
    ret.steeringRateDeg = cp.vl["STEERING"]["STEERING_RATE"]
    ret.steeringTorque = cp.vl["EPS_2"]["COLUMN_TORQUE"]
    ret.steeringTorqueEps = cp.vl["EPS_2"]["EPS_TORQUE_MOTOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # cruise state
    cp_cruise = cp_cam if self.CP.carFingerprint in RAM_CARS else cp

    ret.cruiseState.available = cp_cruise.vl["DAS_3"]["ACC_AVAILABLE"] == 1
    ret.cruiseState.enabled = cp_cruise.vl["DAS_3"]["ACC_ACTIVE"] == 1
    ret.cruiseState.speed = cp_cruise.vl["DAS_4"]["ACC_SET_SPEED_KPH"] * CV.KPH_TO_MS
    ret.cruiseState.nonAdaptive = cp_cruise.vl["DAS_4"]["ACC_STATE"] in (1, 2)  # 1 NormalCCOn and 2 NormalCCSet
    ret.cruiseState.standstill = cp_cruise.vl["DAS_3"]["ACC_STANDSTILL"] == 1
    ret.accFaulted = cp_cruise.vl["DAS_3"]["ACC_FAULTED"] != 0

    if self.CP.carFingerprint in RAM_CARS:
      # Auto High Beam isn't Located in this message on chrysler or jeep currently located in 729 message
      self.auto_high_beam = cp_cam.vl["DAS_6"]['AUTO_HIGH_BEAM_ON']
      ret.steerFaultTemporary = cp.vl["EPS_3"]["DASM_FAULT"] == 1
    else:
      ret.steerFaultTemporary = cp.vl["EPS_2"]["LKAS_TEMPORARY_FAULT"] == 1
      ret.steerFaultPermanent = cp.vl["EPS_2"]["LKAS_STATE"] == 4

    # blindspot sensors
    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["BSM_1"]["LEFT_STATUS"] == 1
      ret.rightBlindspot = cp.vl["BSM_1"]["RIGHT_STATUS"] == 1

    self.lkas_car_model = cp_cam.vl["DAS_6"]["CAR_MODEL"]
    self.button_counter = cp.vl["CRUISE_BUTTONS"]["COUNTER"]

    ret.buttonEvents = create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})

    return ret

  def update_cusw(self, cp, cp_cam):
    ret = structs.CarState()

    ret.doorOpen = any([cp.vl["DOORS"]["DOOR_OPEN_FL"],
                        cp.vl["DOORS"]["DOOR_OPEN_FR"],
                        cp.vl["DOORS"]["DOOR_OPEN_RL"],
                        cp.vl["DOORS"]["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = bool(cp.vl["SEATBELT_STATUS"]["SEATBELT_DRIVER_UNLATCHED"])

    ret.brakePressed = bool(cp.vl["BRAKE_3"]["DRIVER_BRAKE_SWITCH"])
    ret.gasPressed = cp.vl["ACCEL_GAS"]["GAS_HUMAN"] > 0

    ret.espDisabled = bool(cp.vl["TRACTION_BUTTON"]["TRACTION_OFF"])

    ret.vEgoRaw = cp.vl["BRAKE_1"]["VEHICLE_SPEED"]
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = not ret.vEgoRaw > 0.001
    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS_FRONT"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS_REAR"]["WHEEL_SPEED_RR"],
      cp.vl["WHEEL_SPEEDS_REAR"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS_FRONT"]["WHEEL_SPEED_FR"],
      unit=1,
    )

    ret.leftBlinker = cp.vl["STEERING_LEVERS"]["TURN_SIGNALS"] == 1
    ret.rightBlinker = cp.vl["STEERING_LEVERS"]["TURN_SIGNALS"] == 2
    ret.steeringAngleDeg = cp.vl["STEERING"]["STEER_ANGLE"]
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl["GEAR"]["PRNDL"], None))

    ret.cruiseState.speed = cp.vl["ACC_HUD"]["ACC_SET_SPEED_KMH"] * CV.KPH_TO_MS
    ret.cruiseState.available = bool(cp.vl["ACC_CONTROL"]["ACC_MAIN_ON"])
    ret.cruiseState.enabled = bool(cp.vl["ACC_CONTROL"]["ACC_ACTIVE"])

    ret.steeringTorque = cp.vl["EPS_STATUS"]["TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp.vl["EPS_STATUS"]["TORQUE_MOTOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.steerFaultPermanent = bool(cp.vl["EPS_STATUS"]["LKAS_FAULT"])

    if self.CP.enableBsm:
      ret.leftBlindspot = bool(cp.vl["BSM_LEFT"]["LEFT_DETECTED"])
      ret.rightBlindspot = bool(cp.vl["BSM_RIGHT"]["RIGHT_DETECTED"])

    self.lkas_car_model = cp_cam.vl["DAS_6"]["CAR_MODEL"]

    return ret

  def update_susw(self, cp, cp_cam, cp_fusion):
    # cp: bus 0, the camera-side "CAN CH" bus. cp_cam: bus 2, the stock camera, which is where the
    # camera's own LKAS_COMMAND and LKA_HUD_2 arrive before the panda forwards them to bus 0.
    # cp_fusion: bus 1, where a gateway republishes the three raw CAN C ACC messages.
    ret = structs.CarState()

    # Only the two front doors are decoded on this platform, the rear-door bits in 0x4b1 are unknown
    ret.doorOpen = any([cp.vl["DOORS"]["DOOR_OPEN_FL"],
                        cp.vl["DOORS"]["DOOR_OPEN_FR"]])
    # Driver belt only: the passenger belt moves no bit on either bus in the captured sweep.
    # Byte 2 reads 0xff for a few frames after ignition-off, which decodes as unlatched - the safe
    # direction, and the car is off by then.
    ret.seatbeltUnlatched = bool(cp.vl["SEATBELT_STATUS"]["SEATBELT_DRIVER_UNLATCHED"])

    ret.brakePressed = bool(cp.vl["ABS_3"]["BRAKE_PEDAL_SWITCH"])
    # ENGINE_1.THROTTLE_VIRTUAL is the PCM's resolved demand (driver OR ACC), so it reads > 0 on 97.5 %
    # of ACC-engaged frames. ACCEL_PEDAL_DRIVER (0x1f0) is driver-only: exactly 0 across 2,596 s of
    # settled ACC-engaged driving, and > 0 for 98.6 % of the parked pedal press. No deadband needed.
    ret.gasPressed = cp.vl["ACCEL_PEDAL_DRIVER"]["ACCEL_PEDAL_DRIVER"] > 0

    # ABS_1 wheel speeds are already m/s. They are not run through parse_wheel_speeds() because that
    # helper also overwrites vEgoRaw with the wheel speed mean, and ABS_6.VEHICLE_SPEED is the speed
    # the ABS itself publishes (also m/s). The two agree within 0.05 m/s on 91 % of captured frames.
    ret.wheelSpeeds.fl = cp.vl["ABS_1"]["WHEEL_SPEED_FL"] * self.CP.wheelSpeedFactor
    ret.wheelSpeeds.fr = cp.vl["ABS_1"]["WHEEL_SPEED_FR"] * self.CP.wheelSpeedFactor
    ret.wheelSpeeds.rl = cp.vl["ABS_1"]["WHEEL_SPEED_RL"] * self.CP.wheelSpeedFactor
    ret.wheelSpeeds.rr = cp.vl["ABS_1"]["WHEEL_SPEED_RR"] * self.CP.wheelSpeedFactor
    # Saturation guard, not a filter: ABS_6.VEHICLE_SPEED is the speed source and is used unchanged
    # everywhere it is plausible. It only gets overridden by the wheel-speed mean at the two ends of
    # its 11-bit range - pinned at full scale, or reading zero - and only when the 13-bit wheel
    # speeds disagree by more than 2 m/s. A wrap to zero at highway speed would otherwise read as
    # standstill and drop the LKAS control bit through the minSteerSpeed branch.
    speed = cp.vl["ABS_6"]["VEHICLE_SPEED"]
    wheel_speed_mean = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    if (speed >= SUSW_SPEED_MAX - 0.05 or speed <= 0.001) and abs(speed - wheel_speed_mean) > SUSW_SPEED_MISMATCH:
      speed = wheel_speed_mean

    ret.vEgoRaw = speed
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = not ret.vEgoRaw > 0.001

    # GEAR_2.PRNDL (0x5a9) gives real P/R/N/D/MANUAL on CH. ENGINE_1.REVERSE is kept only as a
    # cross-check: it is high over exactly the reverse window. A single frame in 6,740 s of capture
    # reads 0 mid-shift and maps to unknown, same as upstream does for other platforms.
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl["GEAR_2"]["PRNDL"], None))
    ret.parkingBrake = bool(cp.vl["GEAR_2"]["PARKING_BRAKE_ENGAGED"])

    # 1 right, 2 left, 3 hazards
    turn_signals = cp.vl["STEERING_LEVERS"]["TURN_SIGNALS"]
    ret.leftBlinker = turn_signals in (2, 3)
    ret.rightBlinker = turn_signals in (1, 3)

    # sentinel/saturated frames are dropped, see the SUSW_*_INVALID comment above
    if cp.vl["EPS_1"]["STEERING_ANGLE"] < SUSW_STEER_ANGLE_INVALID:
      self.susw_steering_angle = cp.vl["EPS_1"]["STEERING_ANGLE"]
    if cp.vl["EPS_1"]["STEERING_RATE"] < SUSW_STEER_RATE_INVALID:
      self.susw_steering_rate = cp.vl["EPS_1"]["STEERING_RATE"]

    ret.steeringAngleDeg = self.susw_steering_angle
    ret.steeringRateDeg = self.susw_steering_rate
    ret.steeringTorque = cp.vl["EPS_2"]["DRIVER_TORQUE"]
    ret.steeringTorqueEps = cp.vl["EPS_2"]["TORQUE_MOTOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    # EPS_2.LKA_FAULT -> steerFaultTemporary is an EXPLICIT DECISION MADE WITHOUT A POSITIVE SAMPLE,
    # not an accident. The bit is 0 on 100 % of 674k captured frames across every route, so nothing in
    # the data distinguishes a recoverable EPS objection from a latching one, and the temporary vs
    # permanent split for this signal is UNPROVEN - quiet-lynx and velvet-moth both flagged this in
    # their round-1 reviews, and they are right that it cannot be finalized from what we have.
    # It stays temporary for the first drive because that is the conservative direction for a port
    # that actuates nothing yet: steerFaultPermanent latches steerUnavailable for the whole ignition
    # cycle, which would hide the fault behind a single alert instead of letting it re-raise every
    # time the condition recurs, and the fault this port actually expects to meet first (the EPS
    # objecting to LKAS re-enabling too quickly) is recoverable by construction. The cost of being
    # wrong is bounded by dashcamOnly and by the driver-torque limits, not by this bit.
    # Promote to steerFaultPermanent only once a bench fault sample shows the bit latching.
    # test_lka_fault_is_temporary locks the current behavior so a change here has to be deliberate.
    ret.steerFaultTemporary = bool(cp.vl["EPS_2"]["LKA_FAULT"])

    # G3: openpilot engages through the stock ACC controls, and is active only while ACC is engaged
    # AND LaneSense is on. LaneSense off (or ACC off) means openpilot is inactive and ACC/LaneSense
    # behave exactly as stock. LANESENSE_DISABLED comes from the camera's own HUD message on bus 2;
    # until the first one arrives (4 Hz, so up to 250 ms) treat LaneSense as off rather than on.
    lanesense_disabled = not cp_cam.ts_nanos["LKA_HUD_2"]["LANESENSE_DISABLED"] or \
                         bool(cp_cam.vl["LKA_HUD_2"]["LANESENSE_DISABLED"])

    # The stock camera keeps sending 0x1F6 even while the panda is blocking it, so this stays live
    # through an engagement and the CarController can resume the counter from it at hand-over.
    self.lkas_counter = int(cp_cam.vl["LKAS_COMMAND"]["COUNTER"])

    # The three ACC messages only reach bus 1 once the RPGW gateway is in INTERCEPT, which needs
    # openpilot's heartbeat, which dashcam mode never transmits. They are registered with a nan
    # frequency so an empty fusion bus cannot make the whole CarState canInvalid, and freshness is
    # checked here instead: no fusion bus simply means no cruise state. That is a behavior
    # degradation, not the safety path - the panda's own rx checks on 0x103/0x2fa drop
    # controls_allowed within ~100 ms of the fusion bus going quiet.
    # Counted in CarState cycles rather than against another message's timestamp, so that an
    # unhealthy bus 0 cannot freeze the reference clock and hide a dead fusion bus.
    acc_nanos = cp_fusion.ts_nanos["ACC_STATUS_1"]["ACC_ENGAGED"]
    acc_fresh = bool(acc_nanos) and acc_nanos != self.susw_acc_nanos
    if acc_fresh:
      self.susw_acc_nanos = acc_nanos
      self.susw_acc_stale_frames = 0
    else:
      self.susw_acc_stale_frames += 1
    acc_valid = bool(self.susw_acc_nanos) and self.susw_acc_stale_frames < SUSW_ACC_TIMEOUT_FRAMES

    # Mirror the panda's rising-edge engagement semantics. pcm_cruise_check() only grants
    # controls_allowed on a 0 -> 1 transition of ACC_ENGAGED, and during a fusion outage the panda
    # stops receiving 0x103 entirely, so its cruise_engaged_prev stays latched high while its own
    # timeout clears controls_allowed. If the bus comes back with ACC still engaged there is no
    # rising edge for the panda and every LKAS_COMMAND would be blocked - but CarState would happily
    # report enabled again, openpilot would re-engage on pcmCruise, and the mismatch would only
    # surface as controlsMismatch. So: latch on the falling edge of acc_valid and stay disengaged
    # until a fresh frame shows ACC_ENGAGED low, which is exactly when the panda's prev clears too.
    acc_engaged = cp_fusion.vl["ACC_STATUS_1"]["ACC_ENGAGED"] == 1
    if self.susw_acc_valid_prev and not acc_valid:
      self.susw_needs_reengage = True
    elif acc_fresh and not acc_engaged:
      self.susw_needs_reengage = False
    self.susw_acc_valid_prev = acc_valid

    # ACC_STATE: 0 off, 1 on/ready, 2 engaged, 5 standby after cancel. 3 and 4 are one-frame
    # transitions at cancel and engage, so anything but 0 means the stock system is available;
    # testing for (1, 2, 5) dropped availability for a frame at every engagement.
    ret.cruiseState.available = acc_valid and cp_fusion.vl["ACC_HUD"]["ACC_STATE"] != 0
    ret.cruiseState.enabled = acc_valid and acc_engaged and not self.susw_needs_reengage and not lanesense_disabled
    ret.cruiseState.speed = cp_fusion.vl["ACC_HUD"]["ACC_SET_SPEED_KPH"] * CV.KPH_TO_MS if acc_valid else 0.
    ret.cruiseState.nonAdaptive = False  # this car has no non-adaptive cruise mode

    prev_button = self.susw_button
    self.susw_button = next((b for b, sig in enumerate(SUSW_BUTTON_SIGNALS, start=1)
                             if cp_fusion.vl["CRUISE_BUTTONS"][sig]), 0)
    if not self.susw_button and (cp_fusion.vl["CRUISE_BUTTONS"]["ACC_DISTANCE_DEC"] or cp_fusion.vl["CRUISE_BUTTONS"]["ACC_DISTANCE_INC"]):
      self.susw_button = 6
    ret.buttonEvents = create_button_events(self.susw_button, prev_button, SUSW_BUTTONS)

    self.button_counter = cp_fusion.vl["CRUISE_BUTTONS"]["COUNTER"]

    return ret

  @staticmethod
  def get_can_parsers(CP):
    if CP.carFingerprint in SUSW_CARS:
      pt_messages = [
        ("EPS_1", 100),
        ("ABS_1", 100),
        ("ABS_3", 100),
        ("ENGINE_1", 100),
        ("ACCEL_PEDAL_DRIVER", 50),
        ("SEATBELT_STATUS", 10),
        ("GEAR_2", 1),            # 1 Hz plus on change
        ("ABS_6", 100),
        ("EPS_2", 100),
        ("DOORS", 2),             # 2 Hz plus on change
        ("STEERING_LEVERS", 4),   # 4 Hz plus on change
      ]
      # The gateway copies exactly these three raw CAN C messages onto the private fusion bus, at
      # 100 / 50 / 1 Hz respectively. They are registered with a nan frequency, which is opendbc's
      # "never time out" marker (CANParser.ignore_alive): the fusion bus is empty until the gateway
      # reaches INTERCEPT, and a timeout there would pin canValid False forever. NOTE: a frequency
      # of 0 does NOT do this - it means "learn the rate" and still times out after 10 s.
      adas_messages = [
        ("ACC_STATUS_1", float('nan')),
        ("CRUISE_BUTTONS", float('nan')),
        ("ACC_HUD", float('nan')),
      ]
      # LKAS_COMMAND is read on bus 2 for its COUNTER only. The panda forwards the stock camera's
      # 0x1F6 to the EPS while openpilot is inactive and blocks it while controls are allowed, so the
      # CarController has to pick the sequence up where the camera left it - see carcontroller.py.
      cam_messages = [
        ("LKA_HUD_2", 4),
        ("LKAS_COMMAND", 100),
      ]
      return {
        Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
        # the stock camera's HUD message, the only LaneSense state openpilot can see
        Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
        Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.adas], adas_messages, 1),
      }

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
