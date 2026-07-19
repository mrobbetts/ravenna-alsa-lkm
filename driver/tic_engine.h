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
/* W16 (revised on bench): a GM whose measured rate offset exceeds this is
 * UNTRACKABLE — beyond the media servo's steering authority (the IGR bound
 * allows ~±370-400ppm of period nudge) — so the engine is SATURATED. Detected
 * from the PTP servo's GM-rate ESTIMATE (the T2−T1 slope), NOT from the
 * integrator railing: the phase error feeding the IGR is folded modulo the TIC
 * frame, so a fast freewheel (~25,000ppm) aliases into bounded sign-flipping
 * samples that random-walk the integrator — it never sustains the rail (bench:
 * a −25,810ppm Digiface freewheel read "locked"). 500ppm sits above the
 * steering authority and far below any real freewheel (healthy GMs are <±50ppm,
 * broken ones >±10,000ppm). The estimate's EMA is the hysteresis: onset crosses
 * the threshold within a sync; recovery decays it over ~4 s. */
#define GM_SATURATION_PPB 500000LL
/* W17: a per-tick count step (deviation from a normal +1 frame advance) larger
 * than this is a media-clock RE-ANCHOR, not scheduling jitter — the GM returning
 * after an outage, or the saturation clamp releasing on freewheel recovery. ~64
 * frames (~64 ms at the 1 ms tick) is far above a dropped tick or two and far
 * below any real re-anchor (which is thousands to millions of frames). */
#define TIMELINE_BREAK_FRAMES 64

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
    /* W17: latched when the tick count re-anchors by more than TIMELINE_BREAK_
     * FRAMES (a GM returning after an outage, or the saturation clamp releasing).
     * The manager consumes it and xruns the open substreams so clients re-prepare
     * onto the new timeline instead of playing garbled audio. Set in tick_advance,
     * read+cleared in the tick's manager pass (same softirq context). */
    bool m_bTimelineBreak;

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
/* W16 (revised): the GM is beyond the servo's steering authority — PTP-locked
 * with |measured GM rate offset| > GM_SATURATION_PPB. A servo-level fact (all
 * of a domain's engines agree), read via the engine for call-site convenience. */
bool tic_engine_is_saturated(TTicEngine* self);
/* W16 slice 3: the canonical media-clock state (EClockState) — lock status WITH
 * the reason for unlockedness. Single derivation site for the whole stack. */
EClockState tic_engine_clock_state(TTicEngine* self);
/* W17: read-and-clear the timeline-break latch (a large media-clock re-anchor).
 * Returns true once per re-anchor; the manager turns it into an ALSA xrun. */
bool tic_engine_take_timeline_break(TTicEngine* self);

bool tic_engine_is_drop(TTicEngine* self, bool bReset);

uint64_t tic_engine_get_sac(TTicEngine* self);
uint64_t tic_engine_get_time(TTicEngine* self);
void tic_engine_get_times(TTicEngine* self, uint64_t* pui64GlobalSAC, uint64_t* pui64GlobalTime, uint64_t* pui64GlobalPerformanceCounter);
uint32_t tic_engine_get_rate(TTicEngine* self);

#if defined(__cplusplus)
}
#endif
