from enum import IntFlag
from dataclasses import dataclass, field

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16

Ecu = CarParams.Ecu


class ChryslerSafetyFlags(IntFlag):
  RAM_DT = 1
  RAM_HD = 2


class ChryslerFlags(IntFlag):
  # Detected flags
  HIGHER_MIN_STEERING_SPEED = 1
  HAS_BSM = 2  # blind spot monitoring


@dataclass
class ChryslerCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC)"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.fca]))


@dataclass
class ChryslerPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'chrysler_pacifica_2017_hybrid_generated',
    Bus.radar: 'chrysler_pacifica_2017_hybrid_private_fusion',
  })


@dataclass(frozen=True)
class ChryslerCarSpecs(CarSpecs):
  minSteerSpeed: float = 3.8  # m/s


class CAR(Platforms):
  # Chrysler
  CHRYSLER_PACIFICA_2018_HYBRID = ChryslerPlatformConfig(
    [ChryslerCarDocs("Chrysler Pacifica Hybrid 2017-18")],
    ChryslerCarSpecs(wheelbase=3.089, steerRatio=16.2),
  )
  CHRYSLER_PACIFICA_2019_HYBRID = ChryslerPlatformConfig(
    [ChryslerCarDocs("Chrysler Pacifica Hybrid 2019-25")],
    CHRYSLER_PACIFICA_2018_HYBRID.specs,
  )
  CHRYSLER_PACIFICA_2018 = ChryslerPlatformConfig(
    [ChryslerCarDocs("Chrysler Pacifica 2017-18")],
    CHRYSLER_PACIFICA_2018_HYBRID.specs,
  )
  CHRYSLER_PACIFICA_2020 = ChryslerPlatformConfig(
    [
      ChryslerCarDocs("Chrysler Pacifica 2019-20"),
      ChryslerCarDocs("Chrysler Pacifica 2021-23", package="All"),
    ],
    CHRYSLER_PACIFICA_2018_HYBRID.specs,
  )

  # Dodge
  DODGE_DURANGO = ChryslerPlatformConfig(
    [ChryslerCarDocs("Dodge Durango 2020-21")],
    CHRYSLER_PACIFICA_2018_HYBRID.specs,
  )

  # Jeep
  JEEP_CHEROKEE_5TH_GEN = ChryslerPlatformConfig(
    [ChryslerCarDocs("Jeep Cherokee 2019-23")],
    ChryslerCarSpecs(wheelbase=2.70, steerRatio=17.0, minSteerSpeed=18.5),
    {Bus.pt: 'chrysler_cusw'},
  )
  JEEP_GRAND_CHEROKEE = ChryslerPlatformConfig(  # includes 2017 Trailhawk
    [ChryslerCarDocs("Jeep Grand Cherokee 2016-18", video="https://www.youtube.com/watch?v=eLR9o2JkuRk")],
    ChryslerCarSpecs(wheelbase=2.71, steerRatio=16.7),
  )

  JEEP_GRAND_CHEROKEE_2019 = ChryslerPlatformConfig(  # includes 2020 Trailhawk
    [ChryslerCarDocs("Jeep Grand Cherokee 2019-21")],
    JEEP_GRAND_CHEROKEE.specs,
  )
  # FCA 2023 Renegade press kit (2023_JP_Renegade_SP.pdf): curb 3320 lb (1513 kg) 4x4 1.3T, wheelbase 101.2 in,
  # steering overall ratio 15.7. Front/rear weight split is not published.
  # minSteerSpeed is measured from the stock camera: LaneSense arms at ~16.0 m/s on both captured drives
  # (rising edges of LKAS_CONTROL_BIT, min 15.86 / 16.01 m/s) and drops out at ~14.9 m/s (falling edges
  # 14.88 / 14.91). The EPS accepted the control bit down to 14.86 m/s, so 14.9 is the floor of the
  # measured stock envelope. openpilot gates latActive at exactly minSteerSpeed (no hysteresis), and on
  # the 2026-08-26 city drives 16.0 kept openpilot out for 36 % of the ACC-engaged time (AH-161).
  # Below 14.9 m/s is unmeasured: the EPS's own LKA_LOW_SPEED_INHIBIT sits at ~13.6 m/s (AH-149).
  JEEP_RENEGADE = ChryslerPlatformConfig(
    [ChryslerCarDocs("Jeep Renegade 2023", package="Adaptive Cruise Control (ACC) & LaneSense")],
    ChryslerCarSpecs(wheelbase=2.570, steerRatio=15.7, minSteerSpeed=14.9),
    # Bus.pt is the camera-side powertrain bus (bus 0), Bus.adas is the private fusion bus (bus 1) that
    # a gateway populates with the three raw CAN C ACC messages. Same DBC, parsed on two buses.
    {Bus.pt: 'chrysler_susw', Bus.adas: 'chrysler_susw'},
  )

  # Ram
  RAM_1500_5TH_GEN = ChryslerPlatformConfig(
    [ChryslerCarDocs("Ram 1500 2019-24", car_parts=CarParts.common([CarHarness.ram]))],
    ChryslerCarSpecs(wheelbase=3.88, steerRatio=16.3, minSteerSpeed=14.5),
    {Bus.pt: 'chrysler_ram_dt_generated'},
  )
  RAM_HD_5TH_GEN = ChryslerPlatformConfig(
    [
      ChryslerCarDocs("Ram 2500 2020-24", car_parts=CarParts.common([CarHarness.ram])),
      ChryslerCarDocs("Ram 3500 2019-22", car_parts=CarParts.common([CarHarness.ram])),
    ],
    ChryslerCarSpecs(wheelbase=3.785, steerRatio=15.61, minSteerSpeed=16.),
    {Bus.pt: 'chrysler_ram_hd_generated'},
  )


class CarControllerParams:
  def __init__(self, CP):
    self.STEER_STEP = 2  # 50 Hz
    self.STEER_ERROR_MAX = 80
    if CP.carFingerprint in RAM_HD:
      self.STEER_DELTA_UP = 14
      self.STEER_DELTA_DOWN = 14
      self.STEER_MAX = 361  # higher than this faults the EPS
    elif CP.carFingerprint in RAM_DT:
      self.STEER_DELTA_UP = 6
      self.STEER_DELTA_DOWN = 6
      self.STEER_MAX = 261  # EPS allows more, up to 350?
    elif CP.carFingerprint in CUSW_CARS:
      self.STEER_STEP = 1  # 100 Hz
      self.STEER_DELTA_UP = 4
      self.STEER_DELTA_DOWN = 4
      self.STEER_MAX = 250  # TODO: Some CUSW will go to 261, some not quite, exact boundaries not yet determined
    elif CP.carFingerprint in SUSW_CARS:
      self.STEER_STEP = 1  # 100 Hz
      # The stock camera rate limits at exactly 6 counts per 10 ms frame in both directions and never
      # exceeds it in 673,913 captured frames (nonzero |delta| histogram peaks hard at 6 on both
      # drives). We ramp up one count slower than stock and release at the stock rate.
      # ISO lateral jerk (test_lateral_limits): 5 counts/frame over a 383 cap reaches 65 % of the
      # measured MAX_LAT_ACCEL_MEASURED = 1.4 in 0.5 s, i.e. 1.83 m/s^3 up, under the 2.5 + 0.5
      # limit. Down is 2.19 of 5.0. Revisit if STEER_MAX moves.
      self.STEER_DELTA_UP = 5
      self.STEER_DELTA_DOWN = 6
      # The measured stock envelope: the camera commands up to 383 (route 000000d8, -346..+383) and the
      # EPS takes it without a fault, so 383 is proven accepted. The EPS's own ceiling above that is
      # unprobed (AH-149). Raised from the CUSW placeholder 250 after the 2026-08-26 drives, where the
      # assist felt weak (AH-161). Note the torque params are still the Cherokee substitute, so this
      # scales every command by 383/250 for the same lateral-acceleration request.
      self.STEER_MAX = 383
      # SUSW limits against EPS_2.DRIVER_TORQUE (TorqueDriverLimited), not the motor torque:
      # TORQUE_MOTOR is only ~0.23-0.25x the command, so |command - TORQUE_MOTOR| reaches 397 and the
      # stock camera's own frames would fail a TorqueMotorLimited check 36-44 % of the time.
      # With TorqueMotorLimited dropped this allowance is the entire override margin, so it is set
      # Raised 80 -> 160 on 2026-08-27 (AH-161). At 80 the clamp lowered the opposing-direction
      # ceiling on 37.6 % of engaged frames of route 00000123 (6754 s, one deliberate intervention),
      # holding the command to 152 of 383 in the worst 5 %; resting hands ran p90 137 / p99.5 208,
      # so ordinary hands-on driving was trimming the assist. At 160 the clamp is active on 4.4 %.
      # Raised 160 -> 220 on 2026-10-28 (#1879) together with SUSW_STEER_THRESHOLD: resting hands on
      # the Chicago drives ran p90 154 / p99 220 (40 min hands-on, route 00000284), so the clamp was
      # trimming the assist on ordinary hands-on driving again. Override authority is still real and
      # measured: the command is forced to zero at 383 + (220 - d) * 3 = 0, i.e. d ~= 348 counts;
      # the 43 deliberate inputs on those drives peaked 247-371 (median ~310) and the parked
      # lock-to-lock sweep saturates the sensor at 1024.
      self.STEER_DRIVER_ALLOWANCE = 220
      self.STEER_DRIVER_MULTIPLIER = 3
      self.STEER_DRIVER_FACTOR = 1
    else:
      self.STEER_DELTA_UP = 3
      self.STEER_DELTA_DOWN = 3
      self.STEER_MAX = 261  # higher than this faults the EPS


STEER_THRESHOLD = 120

# SUSW only. 120 was inherited from the other Chrysler platforms and sits far below this car's
# resting-hands torque: over 6754 s of engaged driving on route 00000123 (2026-08-27, one
# deliberate intervention in 112 minutes) |DRIVER_TORQUE| ran p50 62, p90 137, p95 157, p99 194,
# p99.5 208, and 16.2 % of engaged frames read as an override that never happened. That matters
# beyond the UI: steeringPressed freezes the lateral PID integrator and suppresses the
# steerSaturated alert, so a threshold this low both weakens tracking and hides the warning.
# 160 was the first step. On the 2026-10-24..26 Chicago drives (#1879) the driver rested hands on the
# wheel for 40 min of route 00000284: |DRIVER_TORQUE| p50 78, p90 154, p99 220 there, 8.3 % of frames
# read as pressed and the flag flickered 1932 times an hour (steerOverride greys the path, freezes
# the lateral integrator, aborts a stiction pulse). The 43 deliberate steering inputs across seven
# routes (held >= 1 s, wheel moved >= 2 deg) peaked at 247-371, median ~310, none under 220. So:
# pressed sets after SUSW_STEER_PRESSED_FRAMES consecutive frames above SUSW_STEER_THRESHOLD, or
# at once above SUSW_STEER_THRESHOLD_INSTANT (a yank), and clears below SUSW_STEER_THRESHOLD_RELEASE
# (hysteresis so a hand hovering at the threshold does not chatter). That leaves ~40 flags an hour
# on the same hands-on stretch. Driver override AUTHORITY is STEER_DRIVER_ALLOWANCE below, raised to
# the same 220 in the same change (panda safety chrysler_susw.h, driver_torque_allowance).
SUSW_STEER_THRESHOLD = 220
SUSW_STEER_THRESHOLD_RELEASE = 180
SUSW_STEER_THRESHOLD_INSTANT = 300
SUSW_STEER_PRESSED_FRAMES = 20   # 0.2 s at the 100 Hz EPS_2 rate

RAM_DT = {CAR.RAM_1500_5TH_GEN, }
RAM_HD = {CAR.RAM_HD_5TH_GEN, }
RAM_CARS = RAM_DT | RAM_HD
CUSW_CARS = {CAR.JEEP_CHEROKEE_5TH_GEN, }
SUSW_CARS = {CAR.JEEP_RENEGADE, }


CHRYSLER_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf132)
CHRYSLER_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(0xf132)

CHRYSLER_SOFTWARE_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.SYSTEM_SUPPLIER_ECU_SOFTWARE_NUMBER)
CHRYSLER_SOFTWARE_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.SYSTEM_SUPPLIER_ECU_SOFTWARE_NUMBER)

CHRYSLER_RX_OFFSET = -0x280

FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"[A-Z0-9_]{10} ?",
  requests=[
    Request(
      [CHRYSLER_VERSION_REQUEST],
      [CHRYSLER_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.eps, Ecu.srs, Ecu.fwdRadar, Ecu.combinationMeter],
      rx_offset=CHRYSLER_RX_OFFSET,
      bus=0,
    ),
    Request(
      [CHRYSLER_VERSION_REQUEST],
      [CHRYSLER_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.hybrid, Ecu.engine, Ecu.transmission],
      bus=0,
    ),
    Request(
      [CHRYSLER_SOFTWARE_VERSION_REQUEST],
      [CHRYSLER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine, Ecu.transmission],
      bus=0,
    ),
  ],
  extra_ecus=[
    (Ecu.abs, 0x7e4, None),  # alt address for abs on hybrids, NOTE: not on all hybrid platforms
  ],
)

DBC = CAR.create_dbc_map()
