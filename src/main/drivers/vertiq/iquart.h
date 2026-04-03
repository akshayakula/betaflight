/*
 * IQUART packet builder for Vertiq 23-06 2306 2200Kv
 *
 * Implements the minimal subset of the IQ Module Communication protocol
 * needed to send SET commands to the Propeller Motor Controller.
 *
 * Packet format (11 bytes total):
 *   [0]    0x55              start-of-frame
 *   [1]    0x06              payload length (6 bytes follow)
 *   [2]    0x34              type_idn (PropellerMotorControl = 52)
 *   [3]    0x05              sub_idn (ctrl_velocity = 5)
 *   [4]    0x01              reserved/subindex
 *   [5..8] float_le          velocity in rad/s, little-endian IEEE-754
 *   [9..10] crc16_le         CRC-16, little-endian
 *
 * Reference: IQ Propeller Motor Controller IDD / Vertiq documentation
 *
 * This file is part of Betaflight / EXark.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include <stdint.h>

#ifdef USE_VERTIQ

/*
 * Build a complete IQUART SET velocity frame for PropellerMotorControl.
 *
 *   buf    – caller-supplied buffer, must be >= IQUART_SET_FLOAT_LEN
 *   value  – velocity in rad/s (IEEE-754 float, little-endian)
 *
 * Returns the number of bytes written (always IQUART_SET_FLOAT_LEN).
 */
uint8_t iquartBuildSetFloat(uint8_t *buf, float value);

#define IQUART_SET_FLOAT_LEN 11
#define IQUART_PROP_MOTOR_TYPE_IDN 52
#define IQUART_CTRL_VELOCITY_SUB_IDN 5

#endif /* USE_VERTIQ */
