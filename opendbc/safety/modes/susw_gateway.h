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
//   tx          nothing. tx_msgs is empty, so tx_msg_safety_check() whitelists
//               nothing and safety_tx_hook() returns false for every address on
//               every bus. The USB capture host is observational only and can
//               never put a frame on the wire. (nooutput_tx_hook is the same
//               "always false" hook NOOUTPUT/SILENT use; reusing it keeps this
//               file free of unreachable code.)
//
//   rx          no classification, no controls. rx_checks is empty, so
//               safety_rx_hook() never whitelists a message and never calls
//               current_hooks->rx -- default_rx_hook is only here to satisfy the
//               non-NULL hook contract. controls_allowed is cleared by
//               set_safety_hooks() and there is no code path in this mode that
//               can ever set it, so controls are never allowed.
//
// Why tx_msgs is empty even though an empty list means no relay-malfunction
// detection: relay_malfunction is inferred by stock_ecu_check() from hearing a
// `.check_relay` tx address on the bus the relay should have cut off. A
// transparent gateway forwards *every* address in both directions, so any
// address we listed with .check_relay would (a) be blocked from forwarding by
// safety_fwd_hook()'s static-blocking loop, breaking transparency, and (b) be
// re-heard on the destination bus one hop later because we ourselves just
// forwarded it, latching a permanent false relay malfunction. There is no
// subset of addresses that is both forwarded transparently and usable as a
// relay-liveness probe, so this mode opts out entirely and the gateway state
// machine owns fault detection instead (CAN bus-off/error counters, ignition,
// harness orientation, fault_status, and a forwarded-frame liveness check).

static safety_config susw_gateway_init(uint16_t param) {
  SAFETY_UNUSED(param);
  return (safety_config){NULL, 0, NULL, 0, false};  // NOLINT(readability/braces)
}

const safety_hooks susw_gateway_hooks = {
  .init = susw_gateway_init,
  .rx = default_rx_hook,
  .tx = nooutput_tx_hook,
};
