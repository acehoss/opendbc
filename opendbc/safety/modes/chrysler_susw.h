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
    {0x1F6U, 0, 4, .check_relay = true},  // LKAS_COMMAND
  };

  static RxCheck chrysler_susw_rx_checks[] = {
    {.msg = {{0xFAU,  0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // ABS_3
    {.msg = {{0xFCU,  0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // ENGINE_1
    {.msg = {{0x101U, 0, 8, 100U, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // ABS_6, has no counter
    {.msg = {{0x106U, 0, 7, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // EPS_2
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
      // Signal: EPS_2.TORQUE_MOTOR
      int torque_meas_new = ((msg->data[0]) << 4) | (msg->data[1] >> 4);
      torque_meas_new -= 2000;
      update_sample(&torque_meas, torque_meas_new);
    }

    if (msg->addr == 0x101U) {
      // Signal: ABS_6.VEHICLE_SPEED, all of byte 1 and the top 3 bits of byte 2
      vehicle_moving = (msg->data[1] != 0U) || ((msg->data[2] >> 5) != 0U);
    }

    if (msg->addr == 0xFAU) {
      // Signal: ABS_3.BRAKE_PEDAL_SWITCH
      brake_pressed = GET_BIT(msg, 3U);
    }

    if (msg->addr == 0xFCU) {
      // Signal: ENGINE_1.ACCEL_PEDAL, the low 5 bits of byte 2 and the top 3 bits of byte 3
      gas_pressed = ((msg->data[2] & 0x1FU) != 0U) || ((msg->data[3] >> 5) != 0U);
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
    // TODO: placeholders carried over from CUSW, tune against the car
    .max_torque = 250,
    .max_rt_delta = 150,
    .max_rate_up = 4,
    .max_rate_down = 4,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
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
