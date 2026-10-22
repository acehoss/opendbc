"""SUSW hands-off nag pre-warning (Redmine #1854).

The Renegade's cluster blares a hands-off nag (LKA_HUD_2.LKA_WARNING_BITS 0 -> 3 while LKA_HUD_STATE
is 12). Across 73 captured nags the only separating information on the bus is EPS_2.DRIVER_TORQUE,
and the working model is a ~5.3 s timer that resets on a driver input of >= 60 counts held for
SUSW_NAG_RESET_SAMPLES consecutive samples and fires only while the torque has also been calm. This tracker mirrors that
model so openpilot can chime first:

  quiet  seconds since the last reset. A reset is a driver input (the SUSW_NAG_RESET_SAMPLES rule),
         a nonzero warning, openpilot lateral inactive, or LKA_HUD_STATE 0 (LaneSense off) or 32
         (fault display). It counts in every other state: 12 green, 2 grey, 16/20/22/24 yellow.
         Logged raw, whatever the HUD state.
  sd     sample standard deviation (ddof=1, as pandas rolling().std() in the analysis) of
         DRIVER_TORQUE over the last SUSW_NAG_SD_WINDOW seconds, every sample, whatever the gate.
  calm   sd < SUSW_NAG_SD_CALM over a full window (never before it has filled) AND the camera is
         armed (12/16/20/22/24). The output, which drives the logged flag and the orange wheel, is
         False while grey; the chime gate uses the raw SD test, so grey only defers it.
  chime  one-sample pulse: the first armed sample of a counting stretch with quiet >=
         SUSW_NAG_CHIME_TIME and calm. It then stays disarmed until the next reset. Grey (2) keeps
         the timer counting but never chimes: back in an armed state already past the chime time
         and calm, it chimes on the first armed sample. If the torque is not calm at
         the chime time it waits for calm and fires when it arrives, still before any reset. That
         is the analysis' rule (first moment both hold); chiming only if calm on the exact chime-time
         sample warned 44 of 73 corpus nags instead of 58 (at 4.0 s, before the HUD-state fixes).

One sample per 100 Hz CarState frame. Its inputs are logged (carState.steeringTorque, the LKA_HUD_2
frames in can, carControl.latActive one frame earlier) and so are its outputs (carState.nagPrewarning),
so a route can be re-scored offline. All tunables are in values.py.
"""
import math

from opendbc.car import DT_CTRL
from opendbc.car.chrysler.values import (SUSW_NAG_CHIME_TIME, SUSW_NAG_RESET_SAMPLES, SUSW_NAG_RESET_TORQUE,
                                         SUSW_NAG_SD_CALM, SUSW_NAG_SD_WINDOW)

# LKA_HUD_2.LKA_HUD_STATE: the timer counts in 12 (LaneSense green/armed, the state the hands-off nag
# fires in), through 2 (grey, no lanes) and through the yellow lane-departure / intervention displays
# 16/20/22/24. Only 0 (LaneSense off) and 32 (fault display) reset. The chime and the calm output
# (orange wheel) are armed states only: route 00000265 1599-1632 s was 32 s hands-off on an unmarked
# road, grey with the camera control bit 0 throughout, no car nag, yet the chime fired and the wheel
# was orange for 30 s. Grey still has to count: on 2026-10-21 a nag came 2.5 s after re-arming from
# 4 s of grey. The car's own timer runs through yellow (11 of the 73 corpus nags had a 16/20 frame in the
# 6 s before them) and through grey: on route 0000025f four of the five un-chimed nags had a 2-4 s
# grey stretch in the 5 s before them (nag at 256.0 s: 17 grey frames then 10 green, torque never
# above 19). Decided on #1854: reset on every state != 12 warned 58/73 corpus nags, holding through
# yellow 61/73, counting through yellow 66/73.
LKA_HUD_STATES_ARMED = (12, 16, 20, 22, 24)
LKA_HUD_STATES_COUNTING = (2,) + LKA_HUD_STATES_ARMED


class RollingStd:
  """Sample standard deviation (ddof=1) of the last `size` integer samples, O(1) per update.

  A ring buffer with integer running sum and sum of squares: DRIVER_TORQUE is whole CAN counts, so
  the sums are exact and the variance cannot drift however long the drive. Over a partial window
  (the first `size` samples) it is the SD of what has been seen; `full` says when that ends.
  """
  def __init__(self, size: int):
    assert size >= 2
    self.size = size
    self._buf = [0] * size
    self._idx = 0
    self.count = 0
    self._sum = 0
    self._sumsq = 0

  @property
  def full(self) -> bool:
    return self.count == self.size

  def update(self, x: int) -> float:
    if self.full:
      old = self._buf[self._idx]
      self._sum -= old
      self._sumsq -= old * old
    else:
      self.count += 1
    self._buf[self._idx] = x
    self._idx = (self._idx + 1) % self.size
    self._sum += x
    self._sumsq += x * x
    return self.std

  @property
  def std(self) -> float:
    n = self.count
    if n < 2:
      return 0.
    # n * sumsq - sum^2 = n^2 * population variance, exact in integers and never negative
    return math.sqrt((n * self._sumsq - self._sum * self._sum) / (n * (n - 1)))


class NagPrewarning:
  def __init__(self):
    self._sd = RollingStd(round(SUSW_NAG_SD_WINDOW / DT_CTRL))
    self._chime_frames = round(SUSW_NAG_CHIME_TIME / DT_CTRL)
    self._torque_run = 0
    self._armed = True
    # counted in frames, not accumulated seconds, so 3.5 s is exactly frame 350 with no float drift
    self.quiet_frames = 0
    self.torque_sd = 0.
    self.calm = False
    self.chime = False

  @property
  def quiet(self) -> float:
    return self.quiet_frames * DT_CTRL

  def update(self, driver_torque: float, hud_state: int, warning_bits: int, lat_active: bool) -> bool:
    """Feed one 100 Hz sample. Returns True on the one sample the pre-warning chimes."""
    torque = int(round(driver_torque))
    self._torque_run = self._torque_run + 1 if abs(torque) >= SUSW_NAG_RESET_TORQUE else 0

    self.torque_sd = self._sd.update(torque)
    camera_armed = hud_state in LKA_HUD_STATES_ARMED
    calm = self._sd.full and self.torque_sd < SUSW_NAG_SD_CALM
    self.calm = calm and camera_armed

    reset = (self._torque_run >= SUSW_NAG_RESET_SAMPLES or warning_bits != 0 or not lat_active or
             hud_state not in LKA_HUD_STATES_COUNTING)
    if reset:
      self.quiet_frames = 0
      self._armed = True
    else:
      self.quiet_frames += 1

    self.chime = self._armed and camera_armed and calm and self.quiet_frames >= self._chime_frames
    if self.chime:
      self._armed = False
    return self.chime
