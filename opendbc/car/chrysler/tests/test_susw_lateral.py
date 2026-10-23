import numpy as np
import pytest

from opendbc.car.car_helpers import interfaces
from opendbc.car.chrysler.values import CAR, SUSW_CARS, CarControllerParams
from opendbc.car.chrysler.susw_lateral import (CURVATURE_TABLE, TORQUE_TABLE, STEER_MAX_COUNTS,
                                               susw_lateral_accel_from_torque, susw_torque_from_lateral_accel)
from opendbc.car.toyota.values import CAR as TOYOTA


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
  def test_round_trip(self, v_ego):
    tp = None
    for la in np.linspace(-2.0, 2.0, 41):
      torque = susw_torque_from_lateral_accel(la, tp, v_ego)
      assert -1.0 <= torque <= 1.0
      back = susw_lateral_accel_from_torque(torque, tp, v_ego)
      if abs(torque) < 1.0:  # inside the table the map is a bijection
        assert back == pytest.approx(la, abs=1e-6)
      else:  # railed: the car can do at most the top of the table at this speed
        assert abs(back) == pytest.approx(CURVATURE_TABLE[-1] * v_ego ** 2, abs=1e-9)
        assert abs(la) >= abs(back)

  def test_measured_points(self):
    # 1.0 m/s2 at 29 m/s is 1.19/km, which the car holds at ~270 counts (25-31 m/s band). The linear
    # model (1.2, friction 0.23) commanded 1.06, i.e. the rail, for the same request.
    assert susw_torque_from_lateral_accel(1.0, None, 29.) * STEER_MAX_COUNTS == pytest.approx(259, abs=15)
    # 0.3 m/s2 at 18 m/s is 0.93/km: ~235 counts. The linear model gave 0.3/1.2 = 0.25 -> 96 counts, in the dead zone.
    assert susw_torque_from_lateral_accel(0.3, None, 18.) * STEER_MAX_COUNTS == pytest.approx(235, abs=15)
    # 1.0 m/s2 at 18 m/s is 3.1/km, past the top of the table: railed, and the limit in lat-accel space says so.
    assert susw_torque_from_lateral_accel(1.0, None, 18.) == 1.0
    assert susw_lateral_accel_from_torque(1.0, None, 18.) == pytest.approx(CURVATURE_TABLE[-1] * 18. ** 2)
    # sign and zero
    assert susw_torque_from_lateral_accel(-0.5, None, 20.) == -susw_torque_from_lateral_accel(0.5, None, 20.)
    assert susw_torque_from_lateral_accel(0.0, None, 20.) == 0.0

  def test_low_speed_guard(self):
    # no division by zero, and at standstill any request rails rather than raising
    assert susw_torque_from_lateral_accel(0.1, None, 0.0) == 1.0
    assert susw_lateral_accel_from_torque(1.0, None, 0.0) == pytest.approx(CURVATURE_TABLE[-1])

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
