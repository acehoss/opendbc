#pragma once

#include "opendbc/safety/declarations.h"

// 2023 Jeep Renegade (FCA small-wide, "SUSW").
//
// Bus layout at the panda:
//   bus 0: CAN CH, car side (EPS, ABS, ENGINE and the LKAS actuation message)
//   bus 1: private fusion bus. A separate gateway copies three raw CAN C messages onto it,
//          nothing is ever forwarded to or from this bus
//   bus 2: CAN CH, camera side
//
// Lateral control only, on top of the stock ACC: the only message openpilot sends is
// LKAS_COMMAND, and engagement follows ACC_STATUS_1.ACC_ENGAGED off the gateway bus.

// FCA CRC-8: poly 0x1D, init 0xFF, final xor 0xFF. Same algorithm as chrysler_compute_checksum(),
// but SUSW needs the covered length to be per-message so the table driven form is used here.
static uint8_t chrysler_susw_crc8_lut[256];

// Every checked message but CRUISE_BUTTONS puts the checksum in its last byte. CRUISE_BUTTONS
// has a trailing padding byte, so its checksum sits in byte 2 and covers bytes 0-1.
static int chrysler_susw_checksum_byte(const CANPacket_t *msg) {
  int checksum_byte = GET_LEN(msg) - 1U;
  if (msg->addr == 0x2FAU) {
    checksum_byte = 2;
  }
  return checksum_byte;
}

static uint32_t chrysler_susw_get_checksum(const CANPacket_t *msg) {
  return (uint32_t)msg->data[chrysler_susw_checksum_byte(msg)];
}

static uint32_t chrysler_susw_compute_checksum(const CANPacket_t *msg) {
  // the checksum covers every byte in front of it
  const int len = chrysler_susw_checksum_byte(msg);
  uint8_t crc = 0xFFU;
  for (int i = 0; i < len; i++) {
    crc = chrysler_susw_crc8_lut[crc ^ msg->data[i]];
  }
  return (uint32_t)(uint8_t)(crc ^ 0xFFU);
}

static uint8_t chrysler_susw_get_counter(const CANPacket_t *msg) {
  uint8_t counter;
  if (msg->addr == 0x2FAU) {
    // Signal: CRUISE_BUTTONS.COUNTER
    counter = (uint8_t)(msg->data[1] & 0xFU);
  } else if (msg->addr == 0xFAU) {
    // Signal: ABS_3.COUNTER
    counter = (uint8_t)((msg->data[4] >> 3) & 0xFU);
  } else {
    // Signal: EPS_2.COUNTER, ENGINE_1.COUNTER, ACC_STATUS_1.COUNTER
    int counter_byte = GET_LEN(msg) - 2U;
    counter = (uint8_t)(msg->data[counter_byte] & 0xFU);
  }
  return counter;
}

static safety_config chrysler_susw_init(uint16_t param) {
  SAFETY_UNUSED(param);

  static const CanMsg CHRYSLER_SUSW_TX_MSGS[] = {
    {0x1F6U, 0, 4, .check_relay = true},  // LKAS_COMMAND, to the EPS on the car side
    // COMMA_HEARTBEAT, on the private fusion bus only. This is the gateway's opt-in for
    // INTERCEPT, so it has to be sendable whether or not controls are allowed, and it is not a
    // vehicle message, so there is no stock ECU to check the relay against.
    {0x5F0U, 1, 8, .check_relay = false},
  };

  static RxCheck chrysler_susw_rx_checks[] = {
    {.msg = {{0xFAU,  0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // ABS_3
    {.msg = {{0x101U, 0, 8, 100U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // ABS_6, has no counter
    {.msg = {{0x106U, 0, 7, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // EPS_2
    // ACCEL_PEDAL_DRIVER carries neither a counter nor a checksum, only its rate is checked
    {.msg = {{0x1F0U, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    // gatewayed from raw CAN C, checked so a dead gateway link also drops controls
    {.msg = {{0x103U, 1, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // ACC_STATUS_1
    {.msg = {{0x2FAU, 1, 4, 50U,  .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // CRUISE_BUTTONS
  };

  gen_crc_lookup_table_8(0x1DU, chrysler_susw_crc8_lut);

  return BUILD_SAFETY_CFG(chrysler_susw_rx_checks, CHRYSLER_SUSW_TX_MSGS);
}

static void chrysler_susw_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    if (msg->addr == 0x106U) {
      // Signal: EPS_2.DRIVER_TORQUE, all of byte 2 and the top 3 bits of byte 3.
      // Same sign convention as LKAS_COMMAND.STEERING_TORQUE, positive is left.
      int torque_driver_new = ((msg->data[2]) << 3) | (msg->data[3] >> 5);
      torque_driver_new -= 1024;
      update_sample(&torque_driver, torque_driver_new);
    }

    if (msg->addr == 0x101U) {
      // Signal: ABS_6.VEHICLE_SPEED, all of byte 1 and the top 3 bits of byte 2
      vehicle_moving = (msg->data[1] != 0U) || ((msg->data[2] >> 5) != 0U);
    }

    if (msg->addr == 0xFAU) {
      // Signal: ABS_3.BRAKE_PEDAL_SWITCH
      brake_pressed = GET_BIT(msg, 3U);
    }

    if (msg->addr == 0x1F0U) {
      // Signal: ACCEL_PEDAL_DRIVER.ACCEL_PEDAL_DRIVER, the low nibble of byte 0 and the top 3
      // bits of byte 1. This is the driver's own accelerator: it is exactly 0 across 2596 s of
      // settled ACC-engaged driving. ENGINE_1.THROTTLE_VIRTUAL (0xFC) is the PCM's resolved
      // demand, driver or ACC, and is nonzero on 97.5 % of ACC-engaged frames, so it must not
      // be used here.
      gas_pressed = ((msg->data[0] & 0x0FU) != 0U) || ((msg->data[1] & 0xE0U) != 0U);
    }
  }

  // the stock ACC only exists on raw CAN C, which the gateway copies onto bus 1
  if ((msg->bus == 1U) && (msg->addr == 0x103U)) {
    // Signal: ACC_STATUS_1.ACC_ENGAGED
    bool cruise_engaged = GET_BIT(msg, 21U);
    pcm_cruise_check(cruise_engaged);
  }
}

static bool chrysler_susw_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits CHRYSLER_SUSW_STEERING_LIMITS = {
    // TODO: placeholder, the stock camera peaks at 383 on the measured routes
    .max_torque = 250,
    // the stock camera rate limits at exactly 6 counts per 10 ms frame in both directions. Ramping
    // up is held one count below that for lateral jerk headroom, since the torque cap and the
    // steering ratio behind it are still borrowed numbers; releasing torque keeps the full 6.
    .max_rate_up = 5,
    .max_rate_down = 6,
    // MAX_RT_INTERVAL is 250 ms and the window rolls on ts_elapsed > MAX_RT_INTERVAL, so 26 command
    // frames fit one window at 100 Hz. This has to clear the fastest legal movement in either
    // direction, 26 * max_rate_down = 156, with the ~1.2x headroom the other torque modes keep.
    // At 150 a legal ramp was blocked at frame 26 and then latched off, because the violation
    // zeroes desired_torque_last while openpilot keeps climbing.
    .max_rt_delta = 180,
    // The driver takes the torque away at allowance + max_torque/multiplier = 80 + 250/3 = 163
    // counts, ~1.36x the 120 count hands-on threshold. The worst hands-off |DRIVER_TORQUE| seen in
    // 6740 s of driving is 87, which costs (87 - 80) * 3 = 21 counts of the 250 cap, so ordinary
    // road noise cannot wind the assist down on its own.
    .driver_torque_allowance = 80,
    .driver_torque_multiplier = 3,
    // EPS_2.TORQUE_MOTOR is the motor's own output, ~0.23-0.25x the command plus ~0.12x the
    // driver, so |command - TORQUE_MOTOR| runs to 397 on stock camera frames. A motor limited
    // check would reject the stock envelope, so this is limited against the driver's torque.
    .type = TorqueDriverLimited,
  };

  bool tx = true;

  if (msg->addr == 0x1F6U) {
    // Signal: LKAS_COMMAND.STEERING_TORQUE
    int desired_torque = ((msg->data[0]) << 3) | (msg->data[1] >> 5);
    desired_torque -= 1024;

    // Signal: LKAS_COMMAND.LKAS_CONTROL_BIT
    const bool steer_req = GET_BIT(msg, 12U);
    if (steer_torque_cmd_checks(desired_torque, steer_req, CHRYSLER_SUSW_STEERING_LIMITS)) {
      tx = false;
    }
  }

  return tx;
}

const safety_hooks chrysler_susw_hooks = {
  .init = chrysler_susw_init,
  .rx = chrysler_susw_rx_hook,
  .tx = chrysler_susw_tx_hook,
  .get_counter = chrysler_susw_get_counter,
  .get_checksum = chrysler_susw_get_checksum,
  .compute_checksum = chrysler_susw_compute_checksum,
};
