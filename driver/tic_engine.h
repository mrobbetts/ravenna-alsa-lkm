/****************************************************************************
*
*  Module Name    : tic_engine.h
*  Version        :
*
*  Abstract       : RAVENNA/AES67 ALSA LKM
*
*  Written by     : van Kempen Bertrand (original TIC machinery in PTP.c)
*  Date           : 27/07/2010
*  Modified by    : Baume Florian (Linux port); multi-rate extraction 2026
*  Modification   : Multi-rate W5: the per-rate TIC engine, extracted
*                   verbatim from TClock_PTP so one PTP servo can discipline
*                   N engines (one per tick rate). See MULTI_PCM_PLAN.md W5.
*  Known problems : None
*
* Copyright(C) 2017 Merging Technologies
*
* RAVENNA/AES67 ALSA LKM is free software; you can redistribute it and / or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* RAVENNA/AES67 ALSA LKM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with RAVAENNA ALSA LKM ; if not, see <http://www.gnu.org/licenses/>.
*
****************************************************************************/
#pragma once

#include <linux/types.h>
#include <linux/spinlock.h>

#include "MTAL_stdint.h"
#include "audio_streamer_clock_PTP_defs.h"

/* REF_UNIT is cent of microseconde (100us). Shared by the servo (PTP.c)
 * and the TIC engines; moved here from PTP.c in the W5 extraction. */
#define PS_2_REF_UNIT 100000000 /* ps is the PTP unit */
#define NS_2_REF_UNIT 100000    /* ns is the linux time unit */

#define TIC_LOCK_HYSTERESIS 5

/* One engine per valid tick rate (44.1/48/88.2/96/176.4/192/352.8/384 kHz;
 * DSD ticks in the 352.8k clock domain). */
#define MAX_TIC_ENGINES_PER_SERVO 8

struct TClock_PTP_s; /* parent servo (PTP.h); engines hold a backpointer only */

/*
 * The TIC engine: everything in the old TClock_PTP that was keyed on ONE
 * rate/frame size — the steered tick period and phase, the PI controller
 * state, the TIC lock counter, and the published frame-quantized media
 * clock (TICSAC + the (SAC, time, perf) triple).
 *
 * Field names are preserved from TClock_PTP so the extraction diff is
 * auditable as a pure move.
 *
 * Locking: the steering/phase state below is protected by the PARENT
 * SERVO's m_csPTPTime (same discipline as before the extraction — one lock
 * per servo covering its engines). The published triple has its own
 * per-engine m_csSAC_Time_Lock.
 */
typedef struct TTicEngine_s
{
    struct TClock_PTP_s* m_pServo;

    uint64_t m_ui64GlobalSAC;  /* this variable will not change during AudioFrameTIC() */
    uint64_t m_ui64GlobalTime; /* [100ns] this variable will not change during AudioFrameTIC() */
    uint64_t m_ui64GlobalPerformanceCounter; /* see MTAL_QueryPerformanceCounter() */
    uint32_t m_ui32FrameSize;
    uint32_t m_ui32SamplingRate;
    volatile bool m_bAudioFrameTICTimerStarted;

    uint64_t m_ui64TICSAC;

    /* TIC frame */
    uint16_t m_usTICLockCounter; /* == 0 means that TIC frame PLL reached the lock state */

    volatile uint64_t m_ui64TIC_LastRTXClockTime; /* [100us] */
    uint64_t m_ui64TIC_LastRTXClockTimeAtT2;      /* [100us] */

    volatile uint64_t m_ui64TIC_NextAbsoluteTime; /* [100us] */
    uint64_t m_dTIC_NextAbsoluteTime_frac;        /* [ps] */

    uint64_t m_dTIC_BasePeriod;             /* [ps] */
    volatile uint64_t m_dTIC_CurrentPeriod; /* [ps] */

    int64_t m_dTIC_IGR; /* [ps] */

    /* Timer */
    uint64_t m_ui64LastCurrentRTXClockTime;
    uint64_t m_ui64LastAbsoluteTime;

    unsigned int m_uiTIC_DropCounter;
    unsigned int m_uiTIC_LastDropCounter;

    /* only for debug check */
    uint64_t m_ui64LastTIC_Count;

    /* Stats */
    int32_t m_maxClkJitter;

    /* 2026-06-11 review fix: embedded, not kmalloc'd — engine creation
     * moved onto the runtime AddPCM path where an unchecked GFP_ATOMIC
     * allocation was an oops waiting for memory pressure. */
    spinlock_t m_csSAC_Time_Lock;

} TTicEngine;

/*
 * Context carried between the two halves of the per-tick work. The old
 * timerProcess interleaved engine work and servo-level checks (link,
 * watchdog); splitting it in two around those checks preserves the exact
 * execution order.
 */
typedef struct
{
    bool bStarted;                    /* engine was started when advance ran */
    uint64_t ui64CurrentTICCount;
    uint64_t ui64AbsoluteTime;        /* [100us] snapshot of next-abs-time */
    uint64_t ui64CurrentRTXClockTime; /* [100us] */
} TTicEngineTickCtx;

#if defined(__cplusplus)
extern "C"
{
#endif

void tic_engine_init(TTicEngine* self, struct TClock_PTP_s* pServo);
void tic_engine_destroy(TTicEngine* self);

/* Engine half of ResetPTPLock. Caller holds the servo's m_csPTPTime
 * (or is on a path where the lock is not required, mirroring the old
 * ResetPTPLock bUseMutex=false callers). */
void tic_engine_reset(TTicEngine* self);

/* Engine half of Start/StopAudioFrameTICTimer. Caller holds the servo's
 * m_csPTPTime for start. */
void tic_engine_start(TTicEngine* self, uint32_t ui32FrameSize, uint32_t ui32SamplingRate);
void tic_engine_stop(TTicEngine* self);

/* Sync-arrival snapshot (old PTP.c sync handler line: LastRTXClockTimeAtT2
 * = LastRTXClockTime). Caller holds the servo's m_csPTPTime. */
void tic_engine_snapshot_at_t2(TTicEngine* self);

/* ProcessT1 fan-outs. Caller holds the servo's m_csPTPTime.
 * prelock_phase_init: the old PTPLockCounter==1 branch (initial period
 * syntonization + phase placement). steer: the old locked-branch frame
 * alignment + PI controller + TIC lock detection. */
void tic_engine_prelock_phase_init(TTicEngine* self, uint64_t ui64T1, uint64_t ui64DeltaT1);
void tic_engine_steer(TTicEngine* self, uint64_t ui64T1);

/* W5: phase-init for an engine started while its servo is already
 * PTP-locked (adding a rate must not glitch running engines — no
 * ResetPTPLock). First tick lands just past the next frame boundary per
 * the live RTX<->PTP offset; TIC convergence follows via steering.
 * Caller holds the servo's m_csPTPTime. */
void tic_engine_phase_init_from_locked(TTicEngine* self);

/* The two halves of the old timerProcess (servo link/watchdog checks run
 * between them). Take the servo lock internally, as timerProcess did.
 * advance: tick bookkeeping, Q/R classification, TICSAC publication.
 * schedule: drop report, overrun handling, next hrtimer wakeup out. */
void tic_engine_tick_advance(TTicEngine* self, TTicEngineTickCtx* pCtx);
void tic_engine_tick_schedule(TTicEngine* self, TTicEngineTickCtx* pCtx, uint64_t* pui64NextRTXClockTime);

/* ST2022-7 standby rephase (old timerSetNextAbsoluteTime). Takes the
 * servo lock internally. */
void tic_engine_set_next_abs_time(TTicEngine* self, uint64_t ui64NextAbsoluteTime);

/* Composite lock status for THIS engine: servo PTP lock + this engine's
 * TIC lock. With one engine per servo this is exactly the old
 * GetLockStatus(). */
EPTPLockStatus tic_engine_lock_status(TTicEngine* self);

bool tic_engine_is_drop(TTicEngine* self, bool bReset);

uint64_t tic_engine_get_sac(TTicEngine* self);
uint64_t tic_engine_get_time(TTicEngine* self);
void tic_engine_get_times(TTicEngine* self, uint64_t* pui64GlobalSAC, uint64_t* pui64GlobalTime, uint64_t* pui64GlobalPerformanceCounter);
uint32_t tic_engine_get_rate(TTicEngine* self);

#if defined(__cplusplus)
}
#endif
