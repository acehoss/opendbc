from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_meas_steer_torque_limits
from opendbc.car.chrysler import chryslercan
from opendbc.car.chrysler.values import CUSW_CARS, RAM_CARS, SUSW_CARS, CarControllerParams, ChryslerFlags
from opendbc.car.interfaces import CarControllerBase

# SUSW torque release at disengage, counts per 10 ms frame. The EPS faults on a torque cliff (every
# survived disengage cut <= 205 counts in one frame, both faults cut >= 279), and the stock camera
# never moves more than 6 per frame. 60 releases the 383 cap in 7 frames (70 ms) and stays under
# the proven-tolerated cliff; the panda accepts the release for at most 10 frames after controls
# drop (chrysler_susw.h, CHRYSLER_SUSW_RELEASE_*).
SUSW_RELEASE_RATE = 60


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_torque_last = 0

    self.hud_count = 0
    self.last_lkas_falling_edge = 0
    self.lkas_control_bit_prev = False
    self.last_button_frame = 0
    self.heartbeat_counter = 0
    self.lkas_counter = 0
    self.lkas_tx_nanos = 0

    self.packer = CANPacker(dbc_names[Bus.pt])
    self.params = CarControllerParams(CP)

  def update(self, CC, CS, now_nanos):
    can_sends = []

    lkas_active = CC.latActive and self.lkas_control_bit_prev

    # SUSW is lateral only: no cruise button TX (the buttons are on a bus openpilot cannot write)
    # and no HUD TX (the cluster HUD messages have not been decoded or safety validated)
    susw = self.CP.carFingerprint in SUSW_CARS

    # RPGW gateway heartbeat, 10 Hz on the private fusion bus, sent whether or not openpilot is
    # engaged. NOTE: while the platform is dashcamOnly, card.py substitutes the noOutput safety mode
    # and nothing is transmitted, so the gateway never leaves BYPASS and the port sees no ACC state
    # on bus 1. That is a documented limitation of dashcam mode, not of this message.
    if susw and self.frame % 10 == 0:
      can_sends.append(chryslercan.create_comma_heartbeat(self.packer, self.heartbeat_counter, CC.latActive))
      self.heartbeat_counter += 1

    # cruise buttons
    if not susw and (self.frame - self.last_button_frame) * DT_CTRL > 0.05:
      das_bus = 2 if self.CP.carFingerprint in RAM_CARS else 0

      # ACC cancellation
      if CC.cruiseControl.cancel:
        self.last_button_frame = self.frame
        can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, cancel=True))

      # ACC resume from standstill
      elif CC.cruiseControl.resume:
        self.last_button_frame = self.frame
        can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.button_counter + 1, das_bus, resume=True))

    # HUD alerts
    if not susw and self.frame % 25 == 0:
      if CS.lkas_car_model != -1:
        can_sends.append(chryslercan.create_lkas_hud(self.packer, self.CP, lkas_active, CC.hudControl.visualAlert,
                                                     self.hud_count, CS.lkas_car_model, CS.auto_high_beam))
        self.hud_count += 1

    # steering
    if self.frame % self.params.STEER_STEP == 0:

      # TODO: can we make this more sane? why is it different for all the cars?
      lkas_control_bit = self.lkas_control_bit_prev
      if CS.out.vEgo > self.CP.minSteerSpeed:
        lkas_control_bit = True
      elif self.CP.flags & ChryslerFlags.HIGHER_MIN_STEERING_SPEED:
        if CS.out.vEgo < (self.CP.minSteerSpeed - 3.0):
          lkas_control_bit = False
      elif self.CP.carFingerprint in RAM_CARS:
        if CS.out.vEgo < (self.CP.minSteerSpeed - 0.5):
          lkas_control_bit = False
      elif self.CP.carFingerprint in CUSW_CARS:
        if CS.out.vEgo < (self.CP.minSteerSpeed - 2.0):
          lkas_control_bit = False
      elif susw:
        # stock LaneSense hysteresis is 1.1 m/s wide (arms ~16.0, drops out ~14.9 m/s on both captured
        # drives). minSteerSpeed is the stock drop-out, so the control bit falls at 13.8 m/s, still above
        # the EPS's ~13.6 m/s LKA_LOW_SPEED_INHIBIT. In practice openpilot clears latActive at exactly
        # minSteerSpeed, so this band only matters if latActive ever outlives the speed gate.
        if CS.out.vEgo < (self.CP.minSteerSpeed - 1.1):
          lkas_control_bit = False

      # EPS faults if LKAS re-enables too quickly
      lkas_control_bit = lkas_control_bit and (self.frame - self.last_lkas_falling_edge > 200)

      if not lkas_control_bit and self.lkas_control_bit_prev:
        self.last_lkas_falling_edge = self.frame
      self.lkas_control_bit_prev = lkas_control_bit

      # steer torque
      new_torque = int(round(CC.actuators.torque * self.params.STEER_MAX))
      if susw:
        # EPS_2.TORQUE_MOTOR is the motor's own output (~0.24x the command), not a mirror of it, so SUSW
        # limits against the driver torque instead. See CarControllerParams and opendbc/safety.
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.params)
      else:
        apply_torque = apply_meas_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorqueEps, self.params)
      if susw:
        # Release instead of cut. Two disengage inputs are read straight from this CarState frame
        # (brake; ACC or LaneSense off via cruiseState.enabled) because the panda drops
        # controls_allowed on that same CAN frame, one to two 10 ms frames before latActive reflects
        # it, and a full-torque command in that window is refused - a hole plus a counter skip at the
        # EPS (route 00000133 t=2622.8). And the release is a ramp, not a zero: the EPS faults on a
        # torque cliff (route 00000149 t=1331.9, -279 -> 0 in one frame with nothing else wrong).
        disengaged = CS.out.brakePressed or not CS.out.cruiseState.enabled
        if disengaged or not lkas_active or not lkas_control_bit:
          if self.apply_torque_last > 0:
            apply_torque = max(self.apply_torque_last - SUSW_RELEASE_RATE, 0)
          else:
            apply_torque = min(self.apply_torque_last + SUSW_RELEASE_RATE, 0)
      elif not lkas_active or not lkas_control_bit:
        apply_torque = 0
      self.apply_torque_last = apply_torque

      if susw:
        # Single arbiter: openpilot sends continuously, with torque zero while idle; camera torque
        # cannot be relayed safely. The idle control bit mirrors the camera so EPS_2.LKA_STATUS stays
        # coherent. A stock frame that actually reached the EPS resyncs the counter at hand-over;
        # otherwise the openpilot counter free-runs through every state transition.
        if CS.lkas_fwd_nanos > self.lkas_tx_nanos:
          elapsed_frames = (now_nanos - CS.lkas_fwd_nanos) // 10_000_000
          self.lkas_counter = (CS.lkas_fwd_counter + elapsed_frames + 1) % 0x10
        # the control bit stays set while torque is still being released (the EPS, and the panda's
        # steer_req rule, both want torque only with the bit set); idle frames mirror the camera
        if apply_torque != 0:
          tx_control_bit = True
        else:
          tx_control_bit = lkas_control_bit if CC.latActive else CS.lkas_cam_control_bit
        can_sends.append(chryslercan.create_lkas_command(self.packer, self.CP, int(apply_torque),
                                                         tx_control_bit, counter=self.lkas_counter))
        self.lkas_counter = (self.lkas_counter + 1) % 0x10
        self.lkas_tx_nanos = now_nanos
      else:
        can_sends.append(chryslercan.create_lkas_command(self.packer, self.CP, int(apply_torque), lkas_control_bit))

    self.frame += 1

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    return new_actuators, can_sends
