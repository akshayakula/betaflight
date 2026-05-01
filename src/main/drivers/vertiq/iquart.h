/*
 * IQUART packet builder for Vertiq 23-06 2306 2200Kv
 *
 * Implements the minimal subset of the IQ Module Communication protocol
 * needed to send SET commands to the Vertiq motor running pulsing firmware.
 *
 * Packet format (11 bytes total):
 *   [0]    0x55              start-of-frame
 *   [1]    0x06              payload length
 *   [2]    type_idn          client type
 *   [3]    sub_idn           entry sub-index
 *   [4]    access_byte       (obj_idn << 2) | kSet
 *   [5..8] float_le          value, little-endian IEEE-754
 *   [9..10] crc16_le         CRC-16, little-endian
 *
 * CRC: Vertiq CRC-16 (poly 0x1021, init 0xFFFF) over bytes [1..8].
 *
 * Constants verified against iq-module-communication-cpp (master):
 *   propeller_motor_control_client.hpp
 *   voltage_superposition_client.hpp
 *   client_communication.hpp
 *   crc_helper.c
 *
 * This file is part of Betaflight / EXark.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include <stdint.h>

#ifdef USE_VERTIQ

uint8_t iquartBuildSetFloat(uint8_t *buf, float value);
uint8_t iquartBuildSetFloatEx(uint8_t *buf, uint8_t typeIdn, uint8_t subIdn, float value);

#define IQUART_SET_FLOAT_LEN 11

/* PropellerMotorControl (type 52) — propeller_motor_control_client.hpp */
#define IQUART_PROP_MOTOR_TYPE_IDN      52
#define IQUART_CTRL_COAST_SUB_IDN        2  /* kSubCtrlCoast  */
#define IQUART_CTRL_VELOCITY_SUB_IDN     5  /* kSubCtrlVelocity */

/* VoltageSuperPosition (type 74) — voltage_superposition_client.hpp */
#define IQUART_VSP_TYPE_IDN             74
#define IQUART_VSP_PHASE_SUB_IDN         2  /* kSubPhase */
#define IQUART_VSP_AMPLITUDE_SUB_IDN     3  /* kSubAmplitude */
#define IQUART_VSP_VELOCITY_CUTOFF_SUB_IDN 6 /* kSubVelocityCutoff */

#endif /* USE_VERTIQ */
