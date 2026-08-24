#!/usr/bin/env python3
import unittest

import opendbc.safety.tests.common as common
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py

# The three route-validated CAN C -> fusion whitelist frames (AH-134). The copy
# itself lives in the panda CAN driver, not in the safety hooks, but these are the
# addresses that matter most for the forwarding policy, so they get named checks.
WHITELIST_ADDRS = (0x103, 0x2FA, 0x73C)

# A representative spread: the whitelist, the SUSW LKAS command, and the bus edges.
SAMPLE_ADDRS = (0x000, 0x103, 0x1F6, 0x2FA, 0x73C, 0x7FF)


class TestSuswGateway(common.SafetyTest):
  """SAFETY_SUSW_GATEWAY: transparent bidirectional CAN C gateway, no host TX, no controls."""

  # The gateway never transmits anything of its own and never lets the USB host transmit.
  TX_MSGS: list[list[int]] = []

  # Transparent: everything on bus 0 goes to bus 2 and vice versa, nothing is blacklisted.
  FWD_BUS_LOOKUP = {0: 2, 2: 0}
  FWD_BLACKLISTED_ADDRS: dict[int, list[int]] = {}

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.suswGateway, 0)
    self.safety.init_tests()

  # ***** mode is registered and selectable *****

  def test_mode_is_set(self):
    self.assertEqual(self.safety.get_current_safety_mode(), CarParams.SafetyModel.suswGateway)

  def test_init_succeeds(self):
    # set_safety_hooks returns 0 when the mode id was found in the registry
    self.assertEqual(self.safety.set_safety_hooks(CarParams.SafetyModel.suswGateway, 0), 0)

  def test_init_ignores_param(self):
    # the gateway takes no parameters; every param must behave identically
    for param in (0, 1, 2, 0xFFFF):
      self.assertEqual(self.safety.set_safety_hooks(CarParams.SafetyModel.suswGateway, param), 0)
      self.assertEqual(self.safety.safety_fwd_hook(0, 0x103), 2)
      self.assertEqual(self.safety.safety_fwd_hook(2, 0x103), 0)
      self.assertFalse(self._tx(common.make_msg(0, 0x103, 8)))

  # ***** transparent bidirectional forwarding *****

  def test_forward_body_to_radar(self):
    for addr in SAMPLE_ADDRS:
      self.assertEqual(self.safety.safety_fwd_hook(0, addr), 2, f"{addr=:#x} bus 0 -> 2")

  def test_forward_radar_to_body(self):
    for addr in SAMPLE_ADDRS:
      self.assertEqual(self.safety.safety_fwd_hook(2, addr), 0, f"{addr=:#x} bus 2 -> 0")

  def test_whitelist_addrs_forwarded_both_ways(self):
    # the fusion copy must not come at the cost of transparency on CAN C
    for addr in WHITELIST_ADDRS:
      self.assertEqual(self.safety.safety_fwd_hook(0, addr), 2, f"{addr=:#x}")
      self.assertEqual(self.safety.safety_fwd_hook(2, addr), 0, f"{addr=:#x}")

  def test_no_forward_from_fusion_bus(self):
    # bus 1 is the private fusion link; nothing on it may reach CAN C
    for addr in (*SAMPLE_ADDRS, *common.SafetyTest.SCANNED_ADDRS[:0x100]):
      self.assertEqual(self.safety.safety_fwd_hook(1, addr), -1, f"{addr=:#x} bus 1")

  def test_no_forward_from_unknown_bus(self):
    for bus in (3, 4, 15):
      for addr in SAMPLE_ADDRS:
        self.assertEqual(self.safety.safety_fwd_hook(bus, addr), -1, f"{addr=:#x} {bus=}")

  def test_forwarding_blocked_on_relay_malfunction(self):
    # the mode declares no check_relay tx msgs so it can never set the flag itself,
    # but if anything else does, safety.h must stop forwarding
    self.safety.set_relay_malfunction(True)
    for addr in SAMPLE_ADDRS:
      self.assertEqual(self.safety.safety_fwd_hook(0, addr), -1)
      self.assertEqual(self.safety.safety_fwd_hook(2, addr), -1)
    self.safety.set_relay_malfunction(False)

  # ***** host TX is never allowed *****

  def test_no_tx_on_any_bus(self):
    for bus in range(4):
      for addr in SAMPLE_ADDRS:
        for length in (1, 4, 8):
          self.assertFalse(self._tx(common.make_msg(bus, addr, length)), f"allowed TX {addr=:#x} {bus=} {length=}")

  def test_no_tx_even_with_controls_allowed(self):
    # controls_allowed must not unlock anything; the host is observational only
    self.safety.set_controls_allowed(True)
    for bus in range(4):
      for addr in WHITELIST_ADDRS:
        self.assertFalse(self._tx(common.make_msg(bus, addr, 8)), f"allowed TX {addr=:#x} {bus=}")

  # ***** rx classifies nothing and never allows controls *****

  def test_rx_hook_accepts_everything(self):
    # no rx_checks means nothing is ever reported invalid
    for bus in range(4):
      for addr in SAMPLE_ADDRS:
        self.assertTrue(self._rx(common.make_msg(bus, addr, 8)), f"failed RX {addr=:#x} {bus=}")

  def test_controls_never_allowed_by_rx(self):
    for bus in range(4):
      for addr in SAMPLE_ADDRS:
        self._rx(common.make_msg(bus, addr, 8))
        self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed after RX {addr=:#x} {bus=}")

  def test_rx_never_sets_relay_malfunction(self):
    # empty tx_msgs => stock_ecu_check() is never called => transparent forwarding
    # can never latch a false relay malfunction on its own echo
    for _ in range(10):
      for bus in (0, 1, 2):
        for addr in SAMPLE_ADDRS:
          self._rx(common.make_msg(bus, addr, 8))
    self.assertFalse(self.safety.get_relay_malfunction())

  def test_rx_checks_never_invalid(self):
    # no rx_checks means safety_tick() can never report a stale/missing message,
    # so the gateway's liveness policy is entirely firmware-owned
    for _ in range(10):
      self.safety.set_timer(0)
      self.safety.safety_tick_current_safety_config()
    self.assertTrue(self._rx(common.make_msg(0, 0x103, 8)))


if __name__ == "__main__":
  unittest.main()
