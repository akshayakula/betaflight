/*
 * IQUART packet builder for Vertiq 23-06 2306 2200Kv
 *
 * CRC algorithm verbatim from Vertiq crc_helper.c (MIT license).
 * Polynomial 0x1021, init 0xFFFF, computed over bytes [1..8].
 *
 * This file is part of Betaflight / EXark.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "platform.h"

#ifdef USE_VERTIQ

#include <string.h>
#include "drivers/vertiq/iquart.h"

/* ---- CRC-16 from Vertiq crc_helper.c (verbatim) ---- */

static uint16_t ByteUpdateCrc(uint16_t crc, uint8_t data)
{
    uint16_t x = (crc >> 8) ^ data;
    x ^= x >> 4;
    crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
    return crc;
}

static uint16_t MakeCrc(const uint8_t *data, uint16_t count)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < count; i++) {
        crc = ByteUpdateCrc(crc, data[i]);
    }
    return crc;
}

/* ---- Packet builder ---- */

/*
 * iquartBuildSetFloatEx — general-purpose SET float frame builder
 *
 * Frame layout (11 bytes):
 *   [0]    0x55          start byte
 *   [1]    0x06          payload length (type + sub + access + 4 float)
 *   [2]    typeIdn       client type_idn
 *   [3]    subIdn        entry sub_idn
 *   [4]    0x01          access byte: (obj_idn=0 << 2) | kSet=1
 *   [5..8] float LE      value
 *   [9..10] CRC-16 LE    over bytes [1..8]
 */
uint8_t iquartBuildSetFloatEx(uint8_t *buf, uint8_t typeIdn, uint8_t subIdn, float value)
{
    buf[0] = 0x55;
    buf[1] = 0x06;
    buf[2] = typeIdn;
    buf[3] = subIdn;
    buf[4] = 0x01;                              /* (0 << 2) | kSet */

    memcpy(&buf[5], &value, sizeof(float));     /* LE on Cortex-M */

    /* CRC over bytes [1..8] — length through last float byte */
    uint16_t crc = MakeCrc(&buf[1], 8);

    buf[9]  = (uint8_t)(crc & 0xFF);
    buf[10] = (uint8_t)(crc >> 8);

    return IQUART_SET_FLOAT_LEN;
}

/*
 * iquartBuildSetFloat — convenience wrapper for ctrl_velocity
 */
uint8_t iquartBuildSetFloat(uint8_t *buf, float value)
{
    return iquartBuildSetFloatEx(buf,
                                IQUART_PROP_MOTOR_TYPE_IDN,
                                IQUART_CTRL_VELOCITY_SUB_IDN,
                                value);
}

#endif /* USE_VERTIQ */
