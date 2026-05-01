/*
 * Vertiq 23-06 IQUART motor output driver (pulsing firmware)
 *
 * 500 Hz control task that reads RC channels, slew-limits velocity and
 * amplitude commands, then streams three IQUART SET packets per cycle
 * over a dedicated serial port.
 *
 * RC mapping:
 *   THROTTLE (ch 3) -> PropellerMotorControl.ctrl_velocity  [0 .. 478.6] rad/s
 *   AUX1     (ch 4) -> VoltageSuperPosition.amplitude       [0 .. 40.0]  V
 *   AUX2     (ch 5) -> VoltageSuperPosition.phase           [-pi .. +pi] rad
 *
 * All output is arm-gated: when disarmed every target is zero.
 *
 * Task rate: 500 Hz (TASK_PERIOD_US = 2000)
 *
 * This file is part of Betaflight / EXark.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "platform.h"

#ifdef USE_VERTIQ

#include <math.h>
#include <string.h>

#include "common/maths.h"
#include "common/time.h"

#include "drivers/serial.h"
#include "drivers/time.h"
#include "drivers/vertiq/iquart.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "io/serial.h"
#include "io/vertiq.h"

#include "rx/rx.h"

/* ========== motor limits (matches Arduino sketch) ========== */

#define V_VEL_MAX       478.6f      /* rad/s  (~4570 RPM mechanical) */
#define V_AMP_MAX        40.0f      /* volts  (VSP amplitude)        */
#define V_PHASE_MIN     (-3.14159265f)
#define V_PHASE_MAX      (3.14159265f)

/* ========== slew rates (per second, match Arduino exactly) ========== */

#define V_VEL_ACCEL     200.0f      /* rad/s per second  (accel) */
#define V_VEL_DECEL     120.0f      /* rad/s per second  (decel) */
#define V_AMP_ATTACK     60.0f      /* V per second      (attack) */
#define V_AMP_DECAY      90.0f      /* V per second      (decay)  */

/* ========== state variables ========== */

static float velCmd;                /* current slewed velocity (rad/s) */
static float ampCmd;                /* current slewed amplitude (V)    */
static float velTarget;             /* target velocity from RC         */
static float ampTarget;             /* target amplitude from RC        */
static float phaseCmd;              /* phase command (rad)             */

static timeUs_t lastUs;             /* timestamp of previous cycle     */
static serialPort_t *vPort;         /* UART handle                     */
static bool vReady;                 /* true after first-run init done  */

/* ========== helpers ========== */

/*
 * slew()  --  rate-limited step toward a target
 *
 * Matches the Arduino slewTo() exactly:
 *   if target > current  -> step at most  +accel * dt
 *   if target < current  -> step at most  -decel * dt
 *   if target == current -> no change
 */
static float slew(float current, float target, float accel, float decel, float dt)
{
    if (target > current) {
        const float step = accel * dt;
        return (target - current > step) ? current + step : target;
    } else if (target < current) {
        const float step = decel * dt;
        return (current - target > step) ? current - step : target;
    }
    return current;
}

/*
 * rc_map()  --  linear map from RC pulse [1000, 2000] to [lo, hi]
 */
static float rc_map(float rc, float lo, float hi)
{
    const float t = constrainf((rc - 1000.0f) / 1000.0f, 0.0f, 1.0f);
    return lo + t * (hi - lo);
}

/* ========== public API ========== */

bool vertiqInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_VERTIQ);
    if (!portConfig) {
        return false;
    }

    /* Open TX-only at 115200 8N1 */
    vPort = openSerialPort(portConfig->identifier, FUNCTION_VERTIQ,
                           NULL, NULL,
                           115200, MODE_TX,
                           SERIAL_STOPBITS_1 | SERIAL_PARITY_NO);

    if (!vPort) {
        return false;
    }

    /* Zero all state */
    velCmd     = 0.0f;
    ampCmd     = 0.0f;
    velTarget  = 0.0f;
    ampTarget  = 0.0f;
    phaseCmd   = 0.0f;
    lastUs     = 0;
    vReady     = false;

    return true;
}

void vertiqTask(timeUs_t currentTimeUs)
{
    if (!vPort) {
        return;
    }

    /* ---- first-run init: disable VSP velocity cutoff (Arduino setup pattern) ---- */
    if (!vReady) {
        uint8_t initFrame[IQUART_SET_FLOAT_LEN];
        iquartBuildSetFloatEx(initFrame,
                              IQUART_VSP_TYPE_IDN,
                              IQUART_VSP_VELOCITY_CUTOFF_SUB_IDN,
                              0.0f);
        serialWriteBuf(vPort, initFrame, IQUART_SET_FLOAT_LEN);

        vReady = true;
        lastUs = currentTimeUs;
        return;
    }

    /* ---- compute dt ---- */
    const timeDelta_t deltaUs = cmpTimeUs(currentTimeUs, lastUs);
    lastUs = currentTimeUs;

    /* Guard against huge dt on first real cycle or scheduler hiccup */
    const float dt = constrainf((float)deltaUs * 1e-6f, 0.0f, 0.01f);

    /* ---- read RC targets ---- */
    if (ARMING_FLAG(ARMED)) {
        velTarget = rc_map(rcData[THROTTLE], 0.0f, V_VEL_MAX);
        ampTarget = rc_map(rcData[AUX1],     0.0f, V_AMP_MAX);
        phaseCmd  = rc_map(rcData[AUX2],     V_PHASE_MIN, V_PHASE_MAX);
    } else {
        velTarget = 0.0f;
        ampTarget = 0.0f;
        phaseCmd  = 0.0f;
    }

    /* ---- slew velocity and amplitude ---- */
    velCmd = slew(velCmd, velTarget, V_VEL_ACCEL, V_VEL_DECEL, dt);
    ampCmd = slew(ampCmd, ampTarget, V_AMP_ATTACK, V_AMP_DECAY, dt);

    /* ---- build and send 3 packets (33 bytes) ---- */

    uint8_t pkt[IQUART_SET_FLOAT_LEN];

    /* Packet 1: PropellerMotorControl.ctrl_velocity (type=52, sub=5) */
    iquartBuildSetFloatEx(pkt,
                          IQUART_PROP_MOTOR_TYPE_IDN,
                          IQUART_CTRL_VELOCITY_SUB_IDN,
                          velCmd);
    serialWriteBuf(vPort, pkt, IQUART_SET_FLOAT_LEN);

    /* Packet 2: VoltageSuperPosition.amplitude (type=74, sub=3) */
    iquartBuildSetFloatEx(pkt,
                          IQUART_VSP_TYPE_IDN,
                          IQUART_VSP_AMPLITUDE_SUB_IDN,
                          ampCmd);
    serialWriteBuf(vPort, pkt, IQUART_SET_FLOAT_LEN);

    /* Packet 3: VoltageSuperPosition.phase (type=74, sub=2) */
    iquartBuildSetFloatEx(pkt,
                          IQUART_VSP_TYPE_IDN,
                          IQUART_VSP_PHASE_SUB_IDN,
                          phaseCmd);
    serialWriteBuf(vPort, pkt, IQUART_SET_FLOAT_LEN);
}

#endif /* USE_VERTIQ */
