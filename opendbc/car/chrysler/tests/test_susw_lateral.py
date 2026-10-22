import numpy as np
import pytest

from opendbc.car.car_helpers import interfaces
from opendbc.car.chrysler.values import CAR, SUSW_CARS, CarControllerParams
from opendbc.car.chrysler.susw_lateral import (BLEND_HI, BLEND_LO, CURVATURE_TABLE, TORQUE_TABLE, STEER_MAX_COUNTS,
                                               susw_lateral_accel_from_torque, susw_torque_from_lateral_accel)
from opendbc.car.toyota.values import CAR as TOYOTA


class _TP:
  latAccelFactor = 1.2


TP = _TP()


def _ci(car):
  CarInterface = interfaces[car]
  CP = CarInterface.get_non_essential_params(car)
  return CarInterface(CP), CP


class TestSuswLateral:
  def test_table_is_monotone_and_covers_full_scale(self):
    assert np.all(np.diff(TORQUE_TABLE) > 0) and np.all(np.diff(CURVATURE_TABLE) > 0)
    assert TORQUE_TABLE[0] == 0 and CURVATURE_TABLE[0] == 0
    assert TORQUE_TABLE[-1] == 1.0

  def test_steer_max_matches_car_controller(self):
    _, CP = _ci(CAR.JEEP_RENEGADE)
    assert CarControllerParams(CP).STEER_MAX == STEER_MAX_COUNTS

  @pytest.mark.parametrize("v_ego", [15., 18., 25., 29., 35.])
  def test_monotone_and_round_trip(self, v_ego):
    prev = -1.0
    for la in np.linspace(0.0, 3.0, 121):
      torque = susw_torque_from_lateral_accel(la, TP, v_ego)
      assert 0.0 <= torque <= 1.0 and torque >= prev
      prev = torque
      back = susw_lateral_accel_from_torque(torque, TP, v_ego)
      if torque < 1.0:
        assert back == pytest.approx(la, abs=0.02)  # numeric inverse on a 64-point grid
      else:  # railed: the car can do at most the top of the table at this speed
        assert back == pytest.approx(CURVATURE_TABLE[-1] * v_ego ** 2, abs=1e-6)
        assert la >= back

  def test_blend_regimes(self):
    # straights: below BLEND_LO the feedforward is exactly the linear model, so the planner's wander
    # on a straight produces the same command it did before the map (no hysteresis-band crossing)
    for v in (15., 18., 29.):
      la = 0.9 * BLEND_LO * v ** 2
      assert susw_torque_from_lateral_accel(la, TP, v) == pytest.approx(min(la / TP.latAccelFactor, np.interp(0.9 * BLEND_LO, CURVATURE_TABLE, TORQUE_TABLE)))
    # the linear model never commands more than the table (no over-commanding at highway speed)
    for v in (25., 29., 36.):
      for la in np.linspace(0.1, 2.0, 20):
        assert susw_torque_from_lateral_accel(la, TP, v) <= np.interp(la / v ** 2, CURVATURE_TABLE, TORQUE_TABLE) + 1e-12
    # curves: above BLEND_HI it is the measured table
    for v in (15., 18., 29.):
      la = 1.1 * BLEND_HI * v ** 2
      assert susw_torque_from_lateral_accel(la, TP, v) == pytest.approx(np.interp(1.1 * BLEND_HI, CURVATURE_TABLE, TORQUE_TABLE))
    # the problem curve, 0.8-1.0 m/s2 at 38 mph (17 m/s), is 2.8-3.5/km: pure table, railed
    assert susw_torque_from_lateral_accel(0.8, TP, 17.) == pytest.approx(np.interp(0.8 / 17. ** 2, CURVATURE_TABLE, TORQUE_TABLE))
    assert susw_torque_from_lateral_accel(1.0, TP, 17.) == 1.0

  def test_measured_points(self):
    # 1.0 m/s2 at 29 m/s is 1.19/km, in the blend; the table alone says ~259 counts, the linear model
    # 319; the result sits between and well under the rail the old model (with friction) commanded.
    t = susw_torque_from_lateral_accel(1.0, TP, 29.) * STEER_MAX_COUNTS
    assert 259 - 15 < t < 319 + 1
    # 1.5 m/s2 at 29 m/s is 1.78/km: pure table, ~297 counts
    assert susw_torque_from_lateral_accel(1.5, TP, 29.) * STEER_MAX_COUNTS == pytest.approx(297, abs=15)
    # 1.0 m/s2 at 18 m/s is 3.1/km, past the top of the table: railed, and the limit in lat-accel space says so
    assert susw_torque_from_lateral_accel(1.0, TP, 18.) == 1.0
    assert susw_lateral_accel_from_torque(1.0, TP, 18.) == pytest.approx(CURVATURE_TABLE[-1] * 18. ** 2)
    # sign and zero
    assert susw_torque_from_lateral_accel(-0.5, TP, 20.) == -susw_torque_from_lateral_accel(0.5, TP, 20.)
    assert susw_torque_from_lateral_accel(0.0, TP, 20.) == 0.0

  def test_low_speed_guard(self):
    # no division by zero, and at standstill any request rails rather than raising
    assert susw_torque_from_lateral_accel(0.1, TP, 0.0) == 1.0
    assert susw_lateral_accel_from_torque(1.0, TP, 0.0) == pytest.approx(CURVATURE_TABLE[-1])

  def test_interface_wiring(self):
    for car in SUSW_CARS:
      CI, CP = _ci(car)
      assert CI.torque_from_lateral_accel() is susw_torque_from_lateral_accel
      assert CI.lateral_accel_from_torque() is susw_lateral_accel_from_torque
      # the dead zone lives in the map now; the friction kick on top of it is nominal (<= ~30 counts at 40 mph)
      assert 0.0 < CP.lateralTuning.torque.friction <= 0.01
    # non-SUSW Chrysler and other brands keep the linear map, and the linear map ignores speed
    CI, CP = _ci(CAR.CHRYSLER_PACIFICA_2018)
    assert CI.torque_from_lateral_accel() == CI.torque_from_lateral_accel_linear
    CI, CP = _ci(TOYOTA.TOYOTA_RAV4)
    tp = CP.lateralTuning.torque
    assert CI.torque_from_lateral_accel()(1.0, tp, 10.) == CI.torque_from_lateral_accel()(1.0, tp, 30.) == pytest.approx(1.0 / tp.latAccelFactor)
