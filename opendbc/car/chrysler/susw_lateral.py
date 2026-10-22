"""SUSW (Jeep Renegade) lateral torque map: LKAS command <-> road curvature.

The SUSW EPS does not apply a torque proportional to LKAS_COMMAND.STEERING_TORQUE. Measured on the
car (#1879, routes 0000025e and 00000265, 2026-10-21; hands off, lat active, column steady, applied
command lagged by the 0.3 s live delay), the steady-state *curvature* the column settles at is a
function of the command alone - the same at 15-19 m/s as at 25-31 m/s up to ~225 counts - and that
function is convex: nothing below ~100 counts, roughly cmd^1.67 from there, steeper again above 250.
EPS_2.EPS_TORQUE_MOTOR tracks cmd^2 (r = 0.935) better than cmd (0.846), so the shape is the EPS
overlay map, not the tires.

Two consequences for the torque controller, which works in lateral-acceleration space:
  1. the plant gain in lat accel scales with v^2, so a constant latAccelFactor is wrong by ~2.6x
     between 40 and 65 mph (over-commanding at highway speed, railing at town speed);
  2. the dead zone + steep knee is a relay: the controller hunts across it at ~1 Hz on a curve.

So the map is kept in curvature and the controller passes vEgo in. The table is the measured
steady-state map; 0-250 counts is the pooled fit over both speed bands (n = 500-4500 per bin), above
250 it follows the 25-31 m/s band (n = 100-440 per bin), because the only low-speed data up there is
one crowned off-ramp held with hands on. The top point is extrapolated. Linear interpolation in
between; outside the table it clamps, so torque never exceeds 1.0 and curvature never exceeds the
top of the table. Below BLEND_LO the feedforward is the linear model instead (see BLEND_LO).
"""
import numpy as np

STEER_MAX_COUNTS = 383  # CarControllerParams.STEER_MAX for SUSW_CARS; the table is in those counts
MIN_SPEED = 1.0  # m/s; below this the v^2 division is meaningless and lat control is off anyway

# LKAS command (counts) -> steady-state curvature (1/m)
_COUNTS = np.array([0, 25, 50, 75, 100, 125, 150, 175, 200, 225, 250, 275, 300, 325, 350, STEER_MAX_COUNTS], dtype=float)
_CURV_PER_KM = np.array([0.0, 0.03, 0.06, 0.12, 0.18, 0.28, 0.42, 0.57, 0.69, 0.83, 1.10, 1.35, 1.85, 2.05, 2.20, 2.40])

TORQUE_TABLE = _COUNTS / STEER_MAX_COUNTS  # normalised [0, 1], what the controller outputs
CURVATURE_TABLE = _CURV_PER_KM * 1e-3     # 1/m

# Small-signal blend (#1879, drive 00000266). The static table is the centre of a wide hysteresis band:
# at low curvature the column holds its angle over a +-100-150 count range of command and only moves
# when the command leaves the band, so the exact inverse of the table turns the planner's +-0.1 m/s2
# wander on a straight into a +-120 count wander at 34 mph (vs +-50 for the linear model), which
# crosses the band, snaps the column 3-5 deg, and hunts. The linear model's small-signal gain stays
# inside the band. So below BLEND_LO (1/m) the feedforward is the linear model, above BLEND_HI it is
# the table, smoothstep in between. The problem curve (0.8-1.0 m/s2 at 38 mph) is 2.5-3.1e-3 1/m,
# pure table; straights (|lat accel| < 0.15 m/s2) are below 0.6e-3 at any speed above 16 m/s.
BLEND_LO = 0.6e-3
BLEND_HI = 1.6e-3
_INVERSE_GRID = 64


def _speed(v_ego: float) -> float:
  return max(abs(float(v_ego)), MIN_SPEED)


def _blend_weight(curvature: float) -> float:
  w = float(np.clip((curvature - BLEND_LO) / (BLEND_HI - BLEND_LO), 0.0, 1.0))
  return w * w * (3.0 - 2.0 * w)


def _table_torque(curvature: float) -> float:
  return float(np.interp(curvature, CURVATURE_TABLE, TORQUE_TABLE))


def susw_torque_from_lateral_accel(lateral_acceleration: float, torque_params, v_ego: float) -> float:
  la = abs(float(lateral_acceleration))
  curvature = la / _speed(v_ego) ** 2
  w = _blend_weight(curvature)
  linear = la / float(torque_params.latAccelFactor)
  table = _table_torque(curvature)
  # the linear model only ever softens the table (its small-signal gain is lower at town speed); where
  # it is above the table (highway speed, mid curvature) the table wins, which also keeps this monotone
  torque = min((1.0 - w) * linear + w * table, table, 1.0)
  return float(np.sign(lateral_acceleration) * torque)


def susw_lateral_accel_from_torque(torque: float, torque_params, v_ego: float) -> float:
  # numeric inverse of the blend at this speed; at |torque| >= 1 it is the top of the table
  v2 = _speed(v_ego) ** 2
  la_top = CURVATURE_TABLE[-1] * v2
  la_grid = np.linspace(0.0, la_top, _INVERSE_GRID)
  torque_grid = np.array([susw_torque_from_lateral_accel(la, torque_params, v_ego) for la in la_grid])
  return float(np.sign(torque) * np.interp(abs(float(torque)), torque_grid, la_grid))
