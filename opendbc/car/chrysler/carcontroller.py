from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_meas_steer_torque_limits
from opendbc.car.chrysler import chryslercan
from opendbc.car.chrysler.values import CUSW_CARS, RAM_CARS, SUSW_CARS, CarControllerParams, ChryslerFlags
from opendbc.car.interfaces import CarControllerBase


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
    self.lkas_tx_prev = False

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
      if not lkas_active or not lkas_control_bit:
        apply_torque = 0
      self.apply_torque_last = apply_torque

      if susw:
        # G3 hand-over, the openpilot half of the contract in chrysler_susw_fwd_hook(). The panda now
        # FORWARDS the stock camera's 0x1F6 to the EPS whenever controls are not allowed, and blocks it
        # only while they are, so stock LaneSense still steers the car when openpilot is inactive. That
        # only works if exactly one sender is on the wire at a time: openpilot must therefore stay off
        # 0x1F6 unless it is the intended controller, or the EPS would see two 100 Hz senders running
        # two independent counter sequences and reject both.
        #
        # RESIDUAL, for the bench before actuation: the panda blocks stock only while controls_allowed
        # and openpilot's last accepted 0x1F6 is less than 50 ms old; this gate remains CC.latActive.
        # Below minSteerSpeed, during calibration, and across disable transitions, openpilot is silent
        # and the timeout hands the EPS back to stock. The M4 parked-EPS measurement in AH-148 sets the
        # final timeout; the provisional value allows a bounded hand-over gap, while an openpilot-first
        # transition can put both senders on the wire for at most one frame. Driver torque override and
        # blinkers do not clear latActive in this openpilot: overriding remains an ACTIVE state.
        if CC.latActive:
          # Continue the stock camera's counter across the hand-over: the camera keeps transmitting
          # while the panda blocks it, so CS.lkas_counter is the last value the EPS would have seen.
          # After the first frame openpilot free-runs its own sequence at the same 100 Hz cadence.
          self.lkas_counter = (self.lkas_counter if self.lkas_tx_prev else int(CS.lkas_counter) + 1) % 0x10
          can_sends.append(chryslercan.create_lkas_command(self.packer, self.CP, int(apply_torque),
                                                           lkas_control_bit, counter=self.lkas_counter))
          self.lkas_counter += 1
        self.lkas_tx_prev = CC.latActive
      else:
        can_sends.append(chryslercan.create_lkas_command(self.packer, self.CP, int(apply_torque), lkas_control_bit))

    self.frame += 1

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    return new_actuators, can_sends
