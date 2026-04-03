/*
 * Vertiq 23-06 motor driver for Betaflight
 *
 * 500 Hz control task: reads RC channels, slew-limits velocity and
 * current targets, then streams IQUART SET packets over UART.
 *
 * This file is part of Betaflight / EXark.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include "common/time.h"

#ifdef USE_VERTIQ

bool vertiqInit(void);
void vertiqTask(timeUs_t currentTimeUs);

#endif /* USE_VERTIQ */
