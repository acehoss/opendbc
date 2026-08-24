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
// The probe is "a frame that can only exist on the other half". ABS_1 and EPS_1
// are sent by the ABS module and the EPS, both of which are on the body harness
// side of XY005A; the fascia side of the inline carries the radar. Both are
// ~100 Hz on raw CAN C (9.7 ms period, analysis/decode-baseline-summary.md), so a
// stuck-closed switch pair is detected within a frame time of the 1 s settle. The panda
// never receives its own transmissions (TX echoes go to can_rx_q from
// process_can(), not through can_rx()), so in a working INTERCEPT these two are
// only ever received on bus 0. Receiving either on bus 2 means the two halves
// are joined -> relay_malfunction -> safety_fwd_hook() blocks all forwarding and
// board/gateway.h drops to POWERED BYPASS. stock_ecu_check() allows
// safety_mode_cnt > 1 s of settling after the mode change first.
//
// Note that EPS_2 (0x106) would be a *dead* probe and is deliberately not used:
// analysis/decode-baseline-summary.md lists it among the four CH-only addresses
// (0x106 EPS_2, 0x10E ABS_7, 0x1F6 LKAS_COMMAND, 0x547 LKA_HUD_2), so it never
// appears on raw CAN C at all and could never be heard on bus 2 however badly the
// switches failed. EPS_1 (0x0DE) is the EPS frame that is actually on this bus.
//
// Only the body -> bus 2 direction is probed, deliberately. The obvious
// reverse-direction candidate is ACC_COMMAND (0x15C), but its sender is not
// established: docs/susw-dbc-notes.md records "key-on order was not decisive
// (0x103 appears in the same millisecond as the ABS frames)" and still lists
// "identify the sender of 0x103/0x15c (radar vs ABS)" as open, and the
// crystal-signature analysis in LONGITUDINAL.md attributes 0x103 to the ABS while
// leaving 0x15C only as "not ABS-sent", which is not the same as "radar-sent". If
// 0x15C is neither, listing it on bus 0 would latch relay_malfunction about a
// second into *every healthy* INTERCEPT and permanently disable the gateway -- a
// fail-dangerous false positive dressed up as a hardware fault. One
// evidence-backed direction detects a stuck-closed relay just as reliably.
//
// Add the radar-side probe once the first successful INTERCEPT capture attributes
// senders directly: with the halves split, whatever is received on bus 2 is by
// construction radar-originated, so one parked INTERCEPT run settles it and the
// second entry becomes a one-line addition to the table below.
static safety_config susw_gateway_init(uint16_t param) {
  static const CanMsg SUSW_GATEWAY_TX_MSGS[] = {
    // ABS_1, ~100 Hz, from the ABS module on the body side
    {0x0EEU, 2, 8, .check_relay = true, .disable_static_blocking = true},
    // EPS_1, ~100 Hz, from the EPS on the body side
    {0x0DEU, 2, 6, .check_relay = true, .disable_static_blocking = true},
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
