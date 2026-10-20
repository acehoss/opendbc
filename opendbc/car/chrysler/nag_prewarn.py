"""SUSW hands-off nag pre-warning (Redmine #1854).

The Renegade's cluster blares a hands-off nag (LKA_HUD_2.LKA_WARNING_BITS 0 -> 3 while LKA_HUD_STATE
is 12). Across 73 captured nags the only separating information on the bus is EPS_2.DRIVER_TORQUE,
and the working model is a ~5.3 s timer that resets on a driver input of >= 60 counts for 2
consecutive samples and fires only while the torque has also been calm. This tracker mirrors that
model so openpilot can chime first:

  quiet  seconds since the last reset. A reset is a driver input (the 2-sample rule), a nonzero
         warning, openpilot lateral inactive, or an LKA_HUD_STATE outside 12 and the yellow states
         16/20/22/24. It counts in all five of those.
  sd     sample standard deviation (ddof=1, as pandas rolling().std() in the analysis) of
         DRIVER_TORQUE over the last SUSW_NAG_SD_WINDOW seconds, every sample, whatever the gate.
  calm   sd < SUSW_NAG_SD_CALM over a full window. Never calm before the window has filled.
  chime  one-sample pulse: the first sample of a counting stretch with quiet >= SUSW_NAG_CHIME_TIME
         while calm. It then stays disarmed until the next reset. If the torque is not calm at
         4.0 s the chime waits for calm and fires when it arrives, still before any reset. That is
         the analysis' rule (first moment both hold); chiming only if calm on the exact 4.0 s
         sample warns 44 of 73 corpus nags instead of 58.

One sample per 100 Hz CarState frame. Its inputs are logged (carState.steeringTorque, the LKA_HUD_2
frames in can, carControl.latActive one frame earlier) and so are its outputs (carState.nagPrewarning),
so a route can be re-scored offline. All tunables are in values.py.
"""
import math

from opendbc.car import DT_CTRL
from opendbc.car.chrysler.values import (SUSW_NAG_CHIME_TIME, SUSW_NAG_RESET_SAMPLES, SUSW_NAG_RESET_TORQUE,
                                         SUSW_NAG_SD_CALM, SUSW_NAG_SD_WINDOW)

# LKA_HUD_2.LKA_HUD_STATE: the timer counts in 12 (LaneSense green/armed, the state the hands-off nag
# fires in) and straight through the yellow lane-departure / intervention displays 16/20/22/24, and the
# chime may fire in any of them. The car's own timer runs through the yellow states: 11 of the 73
# corpus nags had a 16/20 frame in the 6 s before them. Any other state - 0 LaneSense off, 2 grey,
# 32 fault display - resets. Decided 2026-10-20 on #1854 from the corpus replay: reset on every
# state != 12 warned 58/73 nags, holding through yellow 61/73, counting through it 66/73.
LKA_HUD_STATES_COUNTING = (12, 16, 20, 22, 24)


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
    # counted in frames, not accumulated seconds, so 4.0 s is exactly frame 400 with no float drift
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
    self.calm = self._sd.full and self.torque_sd < SUSW_NAG_SD_CALM

    reset = (self._torque_run >= SUSW_NAG_RESET_SAMPLES or warning_bits != 0 or not lat_active or
             hud_state not in LKA_HUD_STATES_COUNTING)
    if reset:
      self.quiet_frames = 0
      self._armed = True
    else:
      self.quiet_frames += 1

    self.chime = self._armed and self.calm and self.quiet_frames >= self._chime_frames
    if self.chime:
      self._armed = False
    return self.chime
