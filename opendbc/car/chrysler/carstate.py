from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.chrysler.values import CUSW_CARS, DBC, STEER_THRESHOLD, RAM_CARS, SUSW_CARS
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase

ButtonType = structs.CarState.ButtonEvent.Type

# SUSW CRUISE_BUTTONS (0x2fa) is a set of independent bits, but only one is ever pressed at a time.
# Collapse them into a single button index so create_button_events() can diff them.
SUSW_BUTTONS = {
  1: ButtonType.mainCruise,       # ACC_ON_OFF
  2: ButtonType.accelCruise,      # ACC_ACCEL, RES+ / speed +
  3: ButtonType.decelCruise,      # ACC_SET_DECEL, SET / speed -
  4: ButtonType.resumeCruise,     # ACC_RESUME
  5: ButtonType.cancel,           # ACC_CANCEL
  6: ButtonType.gapAdjustCruise,  # ACC_DISTANCE_DEC and ACC_DISTANCE_INC, two separate buttons on this car
}


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CP = CP
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])

    self.auto_high_beam = 0
    self.button_counter = 0
    self.lkas_car_model = -1
    self.susw_button = 0

    if CP.carFingerprint in RAM_CARS:
      self.shifter_values = can_define.dv["Transmission_Status"]["Gear_State"]
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
    # cp: bus 0, the camera-side "CAN CH" bus. cp_cam: bus 2, the stock camera.
    # cp_fusion: bus 1, where a gateway republishes the three raw CAN C ACC messages.
    # The camera bus carries only LKAS_COMMAND and LKA_HUD_2, neither of which maps to CarState yet.
    ret = structs.CarState()

    # Only the two front doors are decoded on this platform, the rear-door bits in 0x4b1 are unknown
    ret.doorOpen = any([cp.vl["DOORS"]["DOOR_OPEN_FL"],
                        cp.vl["DOORS"]["DOOR_OPEN_FR"]])
    # TODO: seatbelt state is not decoded yet, several raw CAN C messages react but no isolated driver boolean was found
    ret.seatbeltUnlatched = False

    ret.brakePressed = bool(cp.vl["ABS_3"]["BRAKE_PEDAL_SWITCH"])
    ret.gasPressed = cp.vl["ENGINE_1"]["ACCEL_PEDAL"] > 0

    # ABS_1 wheel speeds are already m/s. They are not run through parse_wheel_speeds() because that
    # helper also overwrites vEgoRaw with the wheel speed mean, and ABS_6.VEHICLE_SPEED is the speed
    # the ABS itself publishes (also m/s). The two agree within 0.05 m/s on 91 % of captured frames.
    ret.wheelSpeeds.fl = cp.vl["ABS_1"]["WHEEL_SPEED_FL"] * self.CP.wheelSpeedFactor
    ret.wheelSpeeds.fr = cp.vl["ABS_1"]["WHEEL_SPEED_FR"] * self.CP.wheelSpeedFactor
    ret.wheelSpeeds.rl = cp.vl["ABS_1"]["WHEEL_SPEED_RL"] * self.CP.wheelSpeedFactor
    ret.wheelSpeeds.rr = cp.vl["ABS_1"]["WHEEL_SPEED_RR"] * self.CP.wheelSpeedFactor
    ret.vEgoRaw = cp.vl["ABS_6"]["VEHICLE_SPEED"]
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = not ret.vEgoRaw > 0.001

    # PRNDL lives in GEAR (0x190), which is raw CAN C only and not visible to the comma.
    # ENGINE_1.REVERSE is the only gear information on the camera bus.
    ret.gearShifter = structs.CarState.GearShifter.reverse if cp.vl["ENGINE_1"]["REVERSE"] else structs.CarState.GearShifter.drive

    # 1 right, 2 left, 3 hazards
    turn_signals = cp.vl["STEERING_LEVERS"]["TURN_SIGNALS"]
    ret.leftBlinker = turn_signals in (2, 3)
    ret.rightBlinker = turn_signals in (1, 3)

    ret.steeringAngleDeg = cp.vl["EPS_1"]["STEERING_ANGLE"]
    ret.steeringRateDeg = cp.vl["EPS_1"]["STEERING_RATE"]
    ret.steeringTorque = cp.vl["EPS_2"]["DRIVER_TORQUE"]
    ret.steeringTorqueEps = cp.vl["EPS_2"]["TORQUE_MOTOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    ret.steerFaultPermanent = bool(cp.vl["EPS_2"]["LKA_FAULT"])

    # ACC state comes from the fusion bus. ACC_STATE: 0 off, 1 on/ready, 2 engaged, 5 standby after cancel
    ret.cruiseState.available = cp_fusion.vl["ACC_HUD"]["ACC_STATE"] in (1, 2, 5)
    ret.cruiseState.enabled = cp_fusion.vl["ACC_STATUS_1"]["ACC_ENGAGED"] == 1
    ret.cruiseState.speed = cp_fusion.vl["ACC_HUD"]["ACC_SET_SPEED_KPH"] * CV.KPH_TO_MS
    ret.cruiseState.nonAdaptive = False  # this car has no non-adaptive cruise mode

    prev_button = self.susw_button
    self.susw_button = next((b for b, sig in enumerate(("ACC_ON_OFF", "ACC_ACCEL", "ACC_SET_DECEL", "ACC_RESUME", "ACC_CANCEL"), start=1)
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
        ("ABS_6", 100),
        ("EPS_2", 100),
        ("DOORS", 2),             # 2 Hz plus on change
        ("STEERING_LEVERS", 4),   # 4 Hz plus on change
      ]
      # The gateway copies exactly these three raw CAN C messages onto the private fusion bus
      adas_messages = [
        ("ACC_STATUS_1", 100),
        ("CRUISE_BUTTONS", 50),
        ("ACC_HUD", 1),           # 1 Hz plus on change
      ]
      return {
        Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
        Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
        Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.adas], adas_messages, 1),
      }

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
