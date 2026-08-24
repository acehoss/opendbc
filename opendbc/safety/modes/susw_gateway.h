#pragma once

#include "opendbc/safety/declarations.h"

// SUSW gateway (AH-134): the safety policy for the RPGW Red Panda sitting inline
// on CAN C at XY005A. It is *not* a car port -- openpilot never selects it and
// never talks through it. The firmware-owned gateway state machine
// (panda/board/gateway.h, built with -DSUSW_GATEWAY) is the only thing that
// enters this mode, and it only does so while the harness relay is driven.
//
// Policy, in full:
//
//   forwarding  transparent and bidirectional, bus 0 <-> bus 2, every address.
//               This is exactly what safety.h's get_fwd_bus() already does, so
//               .fwd stays NULL and disable_forwarding stays false. Bus 1 (the
//               private fusion link to the comma four) is never a forwarding
//               source or destination: get_fwd_bus() returns -1 for it.
//
//   tx          nothing. current_hooks->tx is nooutput_tx_hook, which returns
//               false for every message, so safety_tx_hook() blocks the USB host
//               on every address and every bus -- including the two addresses
//               listed below, which are here only as relay probes and are never
//               transmittable. The capture host is observational only.
//
//   rx          no classification, no controls. rx_checks is empty, so
//               safety_rx_hook() never whitelists a message and never calls
//               current_hooks->rx -- default_rx_hook is only here to satisfy the
//               non-NULL hook contract. controls_allowed is cleared by
//               set_safety_hooks() and there is no code path in this mode that
//               can ever set it, so controls are never allowed.
//
// *** The relay probe ***
//
// The harness box's DG419 analog switches can fail to separate CAN C (marginal
// box power, an SBU1 wiring fault, a switch failure, a mis-assembled harness on
// first bring-up). If that happens while forwarding is on, bus 0 and bus 2 are
// still the same wire: CAN1 receives everything CAN3 transmits and vice versa,
// every frame is re-forwarded both ways, and CAN C saturates in milliseconds.
// Nothing else in the gateway detects it -- total_fwd_cnt grows *faster*, so the
// liveness check is satisfied, and neither controller is bus-off at first.
//
// safety.h's stock_ecu_check() is exactly the right detector, and
// `.disable_static_blocking = true` is what keeps transparency while using it:
// safety_fwd_hook() skips its static-blocking loop for such an entry, so these
// addresses are still forwarded normally, but safety_rx_hook() still calls
// stock_ecu_check() on them.
//
// The probe is "a frame that can only exist on the other half". ABS_1 and EPS_2
// are sent by the ABS module and the EPS, both of which are on the body harness
// side of XY005A; the fascia side of the inline carries the radar. The panda
// never receives its own transmissions (TX echoes go to can_rx_q from
// process_can(), not through can_rx()), so in a working INTERCEPT these two are
// only ever received on bus 0. Receiving either on bus 2 means the two halves
// are joined -> relay_malfunction -> safety_fwd_hook() blocks all forwarding and
// board/gateway.h drops to POWERED BYPASS. stock_ecu_check() allows
// safety_mode_cnt > 1 s of settling after the mode change first.
//
// Only the body -> bus 2 direction is probed, deliberately. The obvious
// reverse-direction candidates are ACC_STATUS_1 (0x103) and ACC_STATUS_2 (0x15C),
// but this project has not established their sender: docs/susw-dbc-notes.md
// records "key-on order was not decisive (0x103 appears in the same millisecond
// as the ABS frames)" and still lists "identify the sender of 0x103/0x15c (radar
// vs ABS)" as an open task. If either is actually body-originated, listing it on
// bus 0 would latch relay_malfunction about a second into *every healthy*
// INTERCEPT and permanently disable the gateway -- a fail-dangerous false
// positive dressed up as a hardware fault. One evidence-backed direction detects
// a stuck-closed relay just as reliably; the second direction is a one-line
// addition once a key-on capture settles the sender.
static safety_config susw_gateway_init(uint16_t param) {
  static const CanMsg SUSW_GATEWAY_TX_MSGS[] = {
    // ABS_1, 9.7 Hz, from the ABS module on the body side
    {0x0EEU, 2, 8, .check_relay = true, .disable_static_blocking = true},
    // EPS_2, 9.7 Hz, from the EPS on the body side
    {0x106U, 2, 7, .check_relay = true, .disable_static_blocking = true},
  };

  SAFETY_UNUSED(param);
  safety_config ret = {NULL, 0, NULL, 0, false};  // NOLINT(readability/braces)
  SET_TX_MSGS(SUSW_GATEWAY_TX_MSGS, ret);
  return ret;
}

const safety_hooks susw_gateway_hooks = {
  .init = susw_gateway_init,
  .rx = default_rx_hook,
  .tx = nooutput_tx_hook,
};
