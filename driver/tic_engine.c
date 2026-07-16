/****************************************************************************
*
*  Module Name    : tic_engine.c
*  Version        :
*
*  Abstract       : RAVENNA/AES67 ALSA LKM
*
*  Written by     : van Kempen Bertrand (original TIC machinery in PTP.c)
*  Date           : 27/07/2010
*  Modified by    : Baume Florian (Linux port); multi-rate extraction 2026
*  Modification   : Multi-rate W5: per-rate TIC engine extracted verbatim
*                   from TClock_PTP (PTP.c). The steering math, Q/R frame
*                   classification, PI gains, clamps and hysteresis are the
*                   original code, parameterized by the engine's own
*                   rate/frame size instead of the servo-wide one. The
*                   shared PTP measurements (T1/T2/DeltaT2, the RTX<->PTP
*                   offset, the PTP lock counter) stay in the servo and are
*                   reached through m_pServo.
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

#include "tic_engine.h"

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/slab.h>

#include "PTP.h"
#include "c_wrapper_lib.h"
#include "module_timer.h"
#include "MTAL_LKernelAPI.h"
#include "MTAL_DP.h"

///////////////////////////////////////////////////////////////////////////////
void tic_engine_init(TTicEngine* self, struct TClock_PTP_s* pServo)
{
    self->m_pServo = pServo;

    self->m_ui64GlobalSAC = 0;
    self->m_ui64GlobalTime = 0;
    self->m_ui64GlobalPerformanceCounter = 0;
    self->m_ui32FrameSize = 64;
    self->m_ui32SamplingRate = 48000;
    self->m_bAudioFrameTICTimerStarted = false;

    self->m_ui64TICSAC = 0;

    /* armed via tic_engine_reset — by ResetPTPLock on the legacy start
     * path, explicitly by tic_entry_start on the no-reset AddPCM path
     * (2026-06-11 review fix) — before the engine can report locked */
    self->m_usTICLockCounter = 0;
    self->m_usSaturatedCounter = 0;  /* W16 */
    self->m_bTimelineBreak = false;  /* W17 */

    self->m_uiTIC_DropCounter = 0;
    self->m_uiTIC_LastDropCounter = 0;

    self->m_ui64TIC_LastRTXClockTime = 0; // [100us]
    self->m_ui64TIC_LastRTXClockTimeAtT2 = 0; // [100us]
    self->m_ui64TIC_NextAbsoluteTime = 0; // [100us]
    self->m_dTIC_NextAbsoluteTime_frac = 0;
    self->m_dTIC_BasePeriod = 64 * 1000000000000 / 48000; // [ps] in that case we loose 0.33333...
    self->m_dTIC_CurrentPeriod = self->m_dTIC_BasePeriod;
    self->m_dTIC_IGR = 0;

    self->m_ui64LastCurrentRTXClockTime = 0;
    self->m_ui64LastAbsoluteTime = 0;
    self->m_ui64LastTIC_Count = 0;

    self->m_maxClkJitter = 0; // [us]

    spin_lock_init(&self->m_csSAC_Time_Lock);
}

///////////////////////////////////////////////////////////////////////////////
void tic_engine_destroy(TTicEngine* self)
{
    /* SAC lock is embedded (2026-06-11 review fix) — nothing to free.
     * Kept as the symmetric teardown hook for the registry. */
    (void)self;
}

///////////////////////////////////////////////////////////////////////////////
void tic_engine_reset(TTicEngine* self)
{
    self->m_dTIC_IGR = 0;
    self->m_usTICLockCounter = TIC_LOCK_HYSTERESIS;
    self->m_usSaturatedCounter = 0;  /* W16 */
    self->m_bTimelineBreak = false;  /* W17 */
}

///////////////////////////////////////////////////////////////////////////////
void tic_engine_start(TTicEngine* self, uint32_t ui32FrameSize, uint32_t ui32SamplingRate)
{
    self->m_ui32FrameSize = ui32FrameSize;
    self->m_ui32SamplingRate = ui32SamplingRate;
    self->m_bAudioFrameTICTimerStarted = true;
    self->m_dTIC_CurrentPeriod = self->m_dTIC_BasePeriod = (self->m_ui32FrameSize * 1000000000000) / self->m_ui32SamplingRate; // [ps]
}

///////////////////////////////////////////////////////////////////////////////
void tic_engine_stop(TTicEngine* self)
{
    self->m_bAudioFrameTICTimerStarted = false;
}

///////////////////////////////////////////////////////////////////////////////
void tic_engine_snapshot_at_t2(TTicEngine* self)
{
    self->m_ui64TIC_LastRTXClockTimeAtT2 = self->m_ui64TIC_LastRTXClockTime;
}

///////////////////////////////////////////////////////////////////////////////
// old ProcessT1, PTPLockCounter==1 branch
void tic_engine_prelock_phase_init(TTicEngine* self, uint64_t ui64T1, uint64_t ui64DeltaT1)
{
    struct TClock_PTP_s* pServo = self->m_pServo;

    //uint64_t a = CW_ll_modulo((uint64_t)(CInt128(ui64T1) * CInt128(self->m_ui32SamplingRate) / CInt128(10000000)), self->m_ui32FrameSize);
    uint64_t a = CW_ll_modulo(((ui64T1) * self->m_ui32SamplingRate) / 10000, self->m_ui32FrameSize);
    uint64_t ui64LastTICFrameFraction = ((a) * 10000 / self->m_ui32SamplingRate); // in RTX clock
    int32_t i32DeltaTICFrame = (int32_t)((pServo->m_ui64T2 - ui64LastTICFrameFraction) - self->m_ui64TIC_LastRTXClockTimeAtT2); // [100ns]

    int32_t i32MidPeriod = (int32_t)(self->m_dTIC_BasePeriod / (PS_2_REF_UNIT * 2));
    if (i32DeltaTICFrame > i32MidPeriod)
    {
        i32DeltaTICFrame -= (int32_t)(self->m_dTIC_BasePeriod / PS_2_REF_UNIT);
    }
    else if (i32DeltaTICFrame < -i32MidPeriod)
    {
        i32DeltaTICFrame += (int32_t)(self->m_dTIC_BasePeriod / PS_2_REF_UNIT);
    }

    self->m_dTIC_CurrentPeriod = (self->m_dTIC_BasePeriod * pServo->m_ui64DeltaT2) / ui64DeltaT1; // [ps]
    MTAL_DP("[%u] self->m_dTIC_CurrentPeriod = %ull , self->m_dTIC_BasePeriod = %ull\n", pServo->m_pEth_netfilter->nic_id, self->m_dTIC_CurrentPeriod, self->m_dTIC_BasePeriod);

    self->m_ui64TIC_NextAbsoluteTime = self->m_ui64TIC_LastRTXClockTimeAtT2 + i32DeltaTICFrame + (5 * self->m_dTIC_CurrentPeriod / PS_2_REF_UNIT);
    self->m_dTIC_NextAbsoluteTime_frac = 0;
}

///////////////////////////////////////////////////////////////////////////////
// old ProcessT1, PTPLockCounter==0 branch (frame alignment + PI controller)
void tic_engine_steer(TTicEngine* self, uint64_t ui64T1)
{
    struct TClock_PTP_s* pServo = self->m_pServo;

    ////////////////////////////////////////
    // Audio TIC

    // Syntonization
    //self->m_dTIC_CurrentPeriod = (self->m_dTIC_BasePeriod * pServo->m_ui64DeltaT2) / (ui64DeltaT1); // [ps]
    self->m_dTIC_CurrentPeriod = self->m_dTIC_BasePeriod; // [ps]

    // period must not derive from base period from more than X ppm
    if (self->m_dTIC_CurrentPeriod == 0)
    {
        MTAL_DP("[%u] self->m_dTIC_CurrentPeriod = 0 (self->m_dTIC_BasePeriod = %llu)!!!", pServo->m_pEth_netfilter->nic_id, self->m_dTIC_BasePeriod);
        self->m_dTIC_CurrentPeriod = self->m_dTIC_BasePeriod;
    }
    else
    {
        if (abs((((signed)self->m_dTIC_CurrentPeriod - (signed)self->m_dTIC_BasePeriod) * 100) / (signed)self->m_dTIC_CurrentPeriod) > 20)
        {
            //bug: cannot print 2 doubles in one single MTAL_DP, in RTX, to investigate....
            MTAL_DP("[%u] self->m_dTIC_CurrentPeriod = %llu , ", pServo->m_pEth_netfilter->nic_id, self->m_dTIC_CurrentPeriod);
            MTAL_DP("[%u] self->m_dTIC_BasePeriod = %llu\n", pServo->m_pEth_netfilter->nic_id, self->m_dTIC_BasePeriod);
            self->m_dTIC_CurrentPeriod = self->m_dTIC_BasePeriod;
        }
    }

    do
    {
        // TIC frame alignment
        // TODO: verify if we have enough word length (is 128bits need?)
        uint64_t a = CW_ll_modulo(((ui64T1) * self->m_ui32SamplingRate) / 10000, (uint64_t)self->m_ui32FrameSize);
        uint64_t ui64LastTICFrameFraction = ((a * 10000) / self->m_ui32SamplingRate); // in RTX clock
        int32_t i32DeltaTICFrame = (int32_t)((pServo->m_ui64T2 - ui64LastTICFrameFraction) - self->m_ui64TIC_LastRTXClockTimeAtT2); // [100ns]
        int64_t dProportional;
        int64_t dPhaseAdj;

        int32_t i32MidPeriod = (int32_t)(self->m_dTIC_BasePeriod / (PS_2_REF_UNIT * 2));

        if (i32DeltaTICFrame > 3 * i32MidPeriod || i32DeltaTICFrame < -3 * i32MidPeriod)
        {
            MTAL_DP("[%u] i32DeltaTICFrame(%d) is out of range. ProcessT1 is not procceed\n", pServo->m_pEth_netfilter->nic_id, i32DeltaTICFrame);
            break;
        }

        if (i32DeltaTICFrame > i32MidPeriod)
        {
            i32DeltaTICFrame -= (int32_t)(self->m_dTIC_BasePeriod / PS_2_REF_UNIT);
        }
        else if (i32DeltaTICFrame < -i32MidPeriod)
        {
            i32DeltaTICFrame += (int32_t)(self->m_dTIC_BasePeriod / PS_2_REF_UNIT);
        }

        // compute proportional
        dProportional = (((i32DeltaTICFrame) * self->m_dTIC_BasePeriod));
        dProportional /= 5000000; // implicitly converted i32DeltaTICFrame into [ps]

        // compute leaky integrator
        self->m_dTIC_IGR = ((self->m_dTIC_IGR + dProportional) * 95) / 100; // [ps]
        self->m_dTIC_IGR = max(min(self->m_dTIC_IGR, 4000000LL), -4000000LL); // integral part is bound to +/- 4us which corresponds to +/- 400ns in the formula below

        /* W16: the integrator pinned at its bound == the servo is railed: the GM
         * is beyond our steering range and we cannot track it. This is the real
         * "can't track" signal — dPhaseAdj below is IGR/10, capped well under the
         * lock thresholds, so it can never reflect a railed servo (which is why
         * the engine used to claim lock to a freewheeling GM). Track it with
         * hysteresis; the integrator's leak un-winds a transient quickly. */
        if (self->m_dTIC_IGR >= 4000000LL || self->m_dTIC_IGR <= -4000000LL)
        {
            if (self->m_usSaturatedCounter < SATURATION_HYSTERESIS)
                self->m_usSaturatedCounter++;
        }
        else if (self->m_usSaturatedCounter > 0)
        {
            self->m_usSaturatedCounter--;
        }

        // Proportional + integral part
        dPhaseAdj = /*dProportional + */self->m_dTIC_IGR / 10; // the leaky integral is the whole output: +/- 400ns period nudge per sync (~+/-370ppm at the 1ms tick) — the IGR bound above IS the steering-range limitor

        self->m_dTIC_CurrentPeriod += dPhaseAdj * 1000; // [ps]

        /* Lock detection (W16 slice 3: honest form). The legacy dPhaseAdj
         * thresholds were provably unreachable — dPhaseAdj = IGR/10 with the
         * IGR clamped to +/-4,000,000 caps at +/-400,000, so the "< 800000"
         * converging test was always true and the "> 1000000" lock-lost branch
         * (and a second +/-4,000,000 limitor on dPhaseAdj itself) were dead
         * code. What the ladder ACTUALLY implemented — and now states plainly —
         * is: locked = TIC_LOCK_HYSTERESIS clean syncs while not saturated.
         * Saturation (W16) is the one real "cannot track" signal; PTP loss
         * re-arms the counter via ResetPTPLock. */
        if (self->m_usSaturatedCounter >= SATURATION_HYSTERESIS)
        {
            if (self->m_usTICLockCounter == 0)
                printk("[nic %u dom %u] TIC saturated (rate %u) — GM beyond steering range, not locked\n", pServo->m_pEth_netfilter->nic_id, pServo->m_PTPConfig.ui8Domain, self->m_ui32SamplingRate);
            self->m_usTICLockCounter = TIC_LOCK_HYSTERESIS;
        }
        else if (self->m_usTICLockCounter > 0)
        {
            self->m_usTICLockCounter--;
            if (self->m_usTICLockCounter == 0)
            {
                /* printk (not MTAL_DP): the old composite lock-status
                 * print made TIC transitions dmesg-visible; keep that
                 * operational signal, now with per-rate identity. */
                {
                    uint64_t period_us = self->m_dTIC_BasePeriod / 1000000ULL; /* [ps] -> [us] */
                    printk("[nic %u dom %u] TIC locked (rate %u, period %llu.%03llu ms)\n", pServo->m_pEth_netfilter->nic_id, pServo->m_PTPConfig.ui8Domain, self->m_ui32SamplingRate, period_us / 1000, period_us % 1000);
                }
            }
        }
    } while (0);
}

///////////////////////////////////////////////////////////////////////////////
/* W5: phase-init for an engine started while its servo is already
 * PTP-locked. The legacy path got its phase from the PTPLockCounter==1
 * branch of ProcessT1 — which only runs during PTP lock acquisition — so a
 * rate added to a live domain would otherwise free-run with arbitrary
 * phase until convergence. Place the first tick just past the next frame
 * boundary derived from the live RTX<->PTP offset (the same alignment math
 * as the Q/R classification, inverted); the PI steering pulls in the
 * residual (sub-period by construction) without touching the servo's PTP
 * lock — running engines on the same domain are never disturbed.
 * Caller holds the servo's m_csPTPTime. */
void tic_engine_phase_init_from_locked(TTicEngine* self)
{
    struct TClock_PTP_s* pServo = self->m_pServo;
    uint64_t ui64Now;
    uint64_t ui64Samples;
    uint32_t ui32R;
    uint64_t ui64ToNextBoundary;

    get_clock_time(&ui64Now);
    ui64Now /= NS_2_REF_UNIT; // [100us]

    ui64Samples = (uint64_t)((ui64Now - pServo->m_i64TIC_PTPToRTXClockOffset) * (self->m_ui32SamplingRate / 100) / 100);
    ui32R = CW_ll_modulo(ui64Samples, self->m_ui32FrameSize);
    ui64ToNextBoundary = ((uint64_t)(self->m_ui32FrameSize - ui32R) * 10000) / self->m_ui32SamplingRate; // [100us]

    self->m_dTIC_CurrentPeriod = self->m_dTIC_BasePeriod;
    self->m_dTIC_IGR = 0;
    self->m_usTICLockCounter = TIC_LOCK_HYSTERESIS;
    self->m_usSaturatedCounter = 0;  /* W16 */
    self->m_bTimelineBreak = false;  /* W17 */

    self->m_ui64TIC_LastRTXClockTime = ui64Now;
    self->m_ui64TIC_LastRTXClockTimeAtT2 = ui64Now; /* sane until the next sync snapshot */

    /* 2026-06-11 review fix: seed the tick count from the live Q so a
     * middle-band first tick (the 100us-grid sawtooth guarantees some at
     * 44.1k) free-runs from the true count, not from 0 — otherwise the
     * engine publishes a near-zero SAC until an edge-window tick snaps it
     * by ~1e14 samples. */
    self->m_ui64LastTIC_Count = ui64Samples / self->m_ui32FrameSize;

    /* +5 periods of margin, multiply BEFORE the [ps -> 100us] divide
     * exactly like the prelock branch (2026-06-11 review fix: the
     * divide-first form truncated 10.884 -> 10 per period, landing the
     * first tick 442us off the frame grid for the whole 44.1k family —
     * outside the ±90.7us Q/R edge window; multiply-first leaves 42us,
     * inside it). Residual error is steered out by the PI loop while the
     * engine is still gated off (not yet TIC-locked). */
    self->m_ui64TIC_NextAbsoluteTime = ui64Now + ui64ToNextBoundary + (5 * self->m_dTIC_BasePeriod) / PS_2_REF_UNIT;
    self->m_dTIC_NextAbsoluteTime_frac = 0;
}

///////////////////////////////////////////////////////////////////////////////
static void computeNextAbsoluteTime(TTicEngine* self, uint32_t ui32FrameCount)
{
    uint64_t ui64Period = self->m_dTIC_CurrentPeriod / PS_2_REF_UNIT; // [ps -> 100us]

    self->m_ui64TIC_NextAbsoluteTime += ui64Period * ui32FrameCount;
    self->m_dTIC_NextAbsoluteTime_frac += CW_ll_modulo(self->m_dTIC_CurrentPeriod, PS_2_REF_UNIT) * ui32FrameCount;
    if (self->m_dTIC_NextAbsoluteTime_frac > PS_2_REF_UNIT)
    {
        uint32_t ui32EpsilonCount = (uint32_t)(self->m_dTIC_NextAbsoluteTime_frac) / PS_2_REF_UNIT;
        self->m_dTIC_NextAbsoluteTime_frac -= PS_2_REF_UNIT * ui32EpsilonCount;
        self->m_ui64TIC_NextAbsoluteTime += ui32EpsilonCount;
    }
}

///////////////////////////////////////////////////////////////////////////////
// first half of the old timerProcess: tick bookkeeping, Q/R classification,
// TICSAC + (SAC, time, perf) publication
void tic_engine_tick_advance(TTicEngine* self, TTicEngineTickCtx* pCtx)
{
    struct TClock_PTP_s* pServo = self->m_pServo;
    int32_t clkJitter;
    uint64_t ui64CurrentTICCount = 0;
    uint64_t ui64CurrentRTXClockTime;

    get_clock_time(&ui64CurrentRTXClockTime);

    clkJitter = (int32_t)((signed)(self->m_ui64TIC_NextAbsoluteTime * 100) - (signed)(ui64CurrentRTXClockTime / 1000));
    self->m_maxClkJitter = max(clkJitter, self->m_maxClkJitter);

    ui64CurrentRTXClockTime /= NS_2_REF_UNIT; // [100us]

    pCtx->ui64CurrentRTXClockTime = ui64CurrentRTXClockTime;
    pCtx->ui64CurrentTICCount = 0;
    pCtx->ui64AbsoluteTime = 0;
    pCtx->bStarted = self->m_bAudioFrameTICTimerStarted;

    if (!pCtx->bStarted)
    {
        return;
    }

    // Atomicity
    {
        spin_lock((spinlock_t*)pServo->m_csPTPTime);

        self->m_ui64TIC_LastRTXClockTime = ui64CurrentRTXClockTime;

        computeNextAbsoluteTime(self, 1);
        {
            uint64_t ui64Q = (uint64_t)((ui64CurrentRTXClockTime - pServo->m_i64TIC_PTPToRTXClockOffset) * (self->m_ui32SamplingRate / 100) / 100) / self->m_ui32FrameSize; // [frame count]
            uint32_t ui32R = CW_ll_modulo((uint64_t)((ui64CurrentRTXClockTime - pServo->m_i64TIC_PTPToRTXClockOffset) * (self->m_ui32SamplingRate / 100) / 100), self->m_ui32FrameSize);

            if(ui32R < 4 * self->m_ui32SamplingRate / 44100) // to avoid using timestamps outside 80us of theoretical time
            {
                ui64CurrentTICCount = ui64Q + 1; // we add 1 because ui64Q is the count for the previous frame
            }
            else if(ui32R > self->m_ui32FrameSize - 4 * self->m_ui32SamplingRate / 44100)
            {
                ui64CurrentTICCount = ui64Q + 1 + 1; // we add 1 because ui64Q is the count for the previous frame
            }
            else
            {
                ui64CurrentTICCount = self->m_ui64LastTIC_Count + 1;
            }
        }

        /* W16: monotonic + CONTIGUOUS media clock, but ONLY while the servo is
         * railed. A railed media servo (an untrackable GM, beyond the steering
         * range) re-anchors the count BACKWARD via the edge-window branches above,
         * which the SAC publish turns into a non-monotonic RTP timestamp that
         * mutes receivers; free-wheel the count forward then (the SAC drifts off
         * the bad GM but stays monotonic + contiguous — the TX reads the SAC once
         * per tick and steps within it, so a held/flat SAC would make it re-emit
         * the same range -> a backward step at the tick boundary).
         *
         * CRUCIAL: gate this on the rail (m_usSaturatedCounter > 0). Once the GM
         * is trackable AGAIN, do NOT clamp — follow the GM directly (ui64Q),
         * re-anchoring even if that steps the count back. The clamp is a forward
         * ratchet; left ungated it NEVER re-syncs, so the forward drift it
         * accumulates during a freewheel becomes a PERMANENT timestamp offset that
         * outlives the freewheel (receiver: a huge fixed offset / "out of bound"
         * long after the GM recovered). The re-anchor is a one-time re-sync at
         * recovery, which receivers expect; a stuck offset is forever. The gate
         * counter increments on the first railed Sync (immediate protection — no
         * onset leak) and decays over SATURATION_HYSTERESIS clean Syncs (no flap
         * at the rail boundary), releasing the clamp shortly after recovery. */
        if (self->m_usSaturatedCounter > 0 &&
            ui64CurrentTICCount <= self->m_ui64LastTIC_Count)
            ui64CurrentTICCount = self->m_ui64LastTIC_Count + 1;

        pCtx->ui64AbsoluteTime = self->m_ui64TIC_NextAbsoluteTime;
        spin_unlock((spinlock_t*)pServo->m_csPTPTime);
    }

    /* W17: detect a media-clock RE-ANCHOR — the count leaping far from a normal
     * single-frame advance. This happens when the GM returns after an outage (the
     * PTP offset is recomputed, so ui64Q leaps) or when the saturation clamp above
     * releases on freewheel recovery (the count snaps back to the trackable GM).
     * Either way the SAC-derived ring cursors jump out from under a client's steady
     * hw_ptr, so latch a break for the manager to turn into an ALSA xrun -> the
     * client re-prepares onto the new timeline instead of playing garbled audio.
     * Ungated by lock: the leap can land during the post-relock LOCKING hysteresis,
     * before the engine reports LOCKED. Skipped while m_ui64LastTIC_Count is still
     * 0 (the cold-start first tick, before phase_init/the first schedule seed it):
     * a running count is always large, so 0 unambiguously means "not seeded yet". */
    if (self->m_ui64LastTIC_Count != 0)
    {
        int64_t step = (int64_t)ui64CurrentTICCount - (int64_t)self->m_ui64LastTIC_Count;
        if (step < 1 - (int64_t)TIMELINE_BREAK_FRAMES ||
            step > 1 + (int64_t)TIMELINE_BREAK_FRAMES)
            self->m_bTimelineBreak = true;
    }

    self->m_ui64TICSAC = (ui64CurrentTICCount - 1) * self->m_ui32FrameSize;
    {
        spin_lock(&self->m_csSAC_Time_Lock);
        {
            self->m_ui64GlobalPerformanceCounter = MTAL_LK_GetCounterTime();
            self->m_ui64GlobalTime = ui64CurrentRTXClockTime;
            self->m_ui64GlobalSAC = self->m_ui64TICSAC;  /* monotonic via the count clamp above */
        }
        spin_unlock(&self->m_csSAC_Time_Lock);
    }

    pCtx->ui64CurrentTICCount = ui64CurrentTICCount;
}

///////////////////////////////////////////////////////////////////////////////
// second half of the old timerProcess: drop report, overrun/late handling,
// next hrtimer wakeup out
void tic_engine_tick_schedule(TTicEngine* self, TTicEngineTickCtx* pCtx, uint64_t* pui64NextRTXClockTime)
{
    uint64_t ui64AbsoluteTime = pCtx->ui64AbsoluteTime;
    uint64_t ui64CurrentRTXClockTime = pCtx->ui64CurrentRTXClockTime;
    uint64_t ui64CurrentTICCount = pCtx->ui64CurrentTICCount;

    if (!pCtx->bStarted)
    {
        /* old timerProcess returned before this point when not started;
         * *pui64NextRTXClockTime is deliberately left untouched */
        return;
    }

    // Report drop
    if(tic_engine_lock_status(self) == PTPLS_LOCKED && ui64CurrentTICCount != self->m_ui64LastTIC_Count + 1)
    {
        MTAL_DP("[%u] LastTICCounter = %llu ui64TICCounter = %llu (Timer period = %llu [100us])\n", self->m_pServo->m_pEth_netfilter->nic_id, self->m_ui64LastTIC_Count, ui64CurrentTICCount, ui64CurrentRTXClockTime - self->m_ui64LastCurrentRTXClockTime);
        self->m_uiTIC_DropCounter++;
    }
    self->m_ui64LastTIC_Count = ui64CurrentTICCount;

    {
        // too late detection
        uint64_t ui64CurrentTime;
        bool dropout_every_5second = false; // DSD mute debug

        get_clock_time(&ui64CurrentTime);
        ui64CurrentTime /= NS_2_REF_UNIT; // [100us]

        if (ui64AbsoluteTime <= ui64CurrentTime || dropout_every_5second)
        {
            if (ui64CurrentTime - ui64AbsoluteTime < self->m_dTIC_CurrentPeriod / 2 / PS_2_REF_UNIT) // give a chance to be late of 200us
            {
                MTAL_DP("[%u] %llu [100us] Overrun (upto Period / 2 (%llu [100us]), let try to catch up)\n", self->m_pServo->m_pEth_netfilter->nic_id, ui64CurrentTime - ui64AbsoluteTime, self->m_dTIC_CurrentPeriod / 2 / PS_2_REF_UNIT);
            }
            else if (tic_engine_lock_status(self) == PTPLS_LOCKED)
            {
                MTAL_DP("[%u] timerProcess elapsed time = %llu [100us]", self->m_pServo->m_pEth_netfilter->nic_id, ui64CurrentTime - ui64CurrentRTXClockTime);
                self->m_ui64LastAbsoluteTime = ui64AbsoluteTime;
                self->m_ui64LastCurrentRTXClockTime = ui64CurrentRTXClockTime;
            }
            else
            {
                // When we are not locked, we continue to wakeup the timer on the period time
                // This allow to continue to update self->m_ui64TIC_LastRTXClockTime which is mandatory to adjust the tic phase when PTP packet is received (TIC lock)
                self->m_ui64TIC_NextAbsoluteTime = ui64AbsoluteTime = ui64CurrentTime + self->m_dTIC_BasePeriod / PS_2_REF_UNIT;
            }
        }
        self->m_ui64LastAbsoluteTime = ui64AbsoluteTime;
        self->m_ui64LastCurrentRTXClockTime = ui64CurrentRTXClockTime;

        *pui64NextRTXClockTime = ui64AbsoluteTime * NS_2_REF_UNIT;
    }
}

///////////////////////////////////////////////////////////////////////////////
void tic_engine_set_next_abs_time(TTicEngine* self, uint64_t ui64NextAbsoluteTime)
{
    spin_lock/*_irqsave*/((spinlock_t*)self->m_pServo->m_csPTPTime/*, flags*/);
    self->m_ui64TIC_NextAbsoluteTime = ui64NextAbsoluteTime / NS_2_REF_UNIT;
    spin_unlock/*_irqrestore*/((spinlock_t*)self->m_pServo->m_csPTPTime/*, flags*/);
}

///////////////////////////////////////////////////////////////////////////////
EPTPLockStatus tic_engine_lock_status(TTicEngine* self)
{
    if(self->m_pServo->m_usPTPLockCounter != 0)
    {
        return PTPLS_UNLOCKED;
    }
    if(self->m_usTICLockCounter != 0)
    {
        return PTPLS_LOCKING;
    }
    return PTPLS_LOCKED;
}

///////////////////////////////////////////////////////////////////////////////
/* W16: the media servo is railed — the GM is beyond our steering range and
 * untrackable. Combined with lock_status, lets a consumer distinguish
 * "saturated (GM too far off)" from "acquiring (converging)". */
bool tic_engine_is_saturated(TTicEngine* self)
{
    return self->m_usSaturatedCounter >= SATURATION_HYSTERESIS;
}

///////////////////////////////////////////////////////////////////////////////
/* W16 slice 3: THE canonical media-clock state — the one derivation of "is this
 * clock usable, and if not, why". Ordering is deliberate: a stopped engine says
 * nothing about clocks; without PTP the saturation counter is stale (no Syncs
 * feed the servo) so NO_SIGNAL outranks SATURATED; saturation then outranks
 * ACQUIRING because a railed servo holds the lock counter re-armed forever —
 * "converging" would be a lie. Reads are unlocked (u16/bool snapshots) — status
 * reporting, same discipline as tic_engine_lock_status. */
EClockState tic_engine_clock_state(TTicEngine* self)
{
    if (!self->m_bAudioFrameTICTimerStarted)
        return CLK_STOPPED;
    if (self->m_pServo->m_usPTPLockCounter != 0)
        return CLK_NO_SIGNAL;
    if (self->m_usSaturatedCounter >= SATURATION_HYSTERESIS)
        return CLK_SATURATED;
    if (self->m_usTICLockCounter != 0)
        return CLK_ACQUIRING;
    return CLK_LOCKED;
}

///////////////////////////////////////////////////////////////////////////////
/* W17: read-and-clear the timeline-break latch. Same softirq context as the
 * setter (tick_advance) and this consumer (the manager tick), so no lock. */
bool tic_engine_take_timeline_break(TTicEngine* self)
{
    bool broke = self->m_bTimelineBreak;
    self->m_bTimelineBreak = false;
    return broke;
}

///////////////////////////////////////////////////////////////////////////////
bool tic_engine_is_drop(TTicEngine* self, bool bReset)
{
    bool bDrop = self->m_uiTIC_DropCounter != self->m_uiTIC_LastDropCounter;
    if(bDrop && bReset)
    {
        self->m_uiTIC_LastDropCounter = self->m_uiTIC_DropCounter;
    }
    return bDrop;
}

///////////////////////////////////////////////////////////////////////////////
uint64_t tic_engine_get_sac(TTicEngine* self)
{
    return self->m_ui64GlobalSAC;
}

///////////////////////////////////////////////////////////////////////////////
uint64_t tic_engine_get_time(TTicEngine* self)
{
    return self->m_ui64GlobalTime;
}

///////////////////////////////////////////////////////////////////////////////
void tic_engine_get_times(TTicEngine* self, uint64_t* pui64GlobalSAC, uint64_t* pui64GlobalTime, uint64_t* pui64GlobalPerformanceCounter)
{
    spin_lock(&self->m_csSAC_Time_Lock);

    *pui64GlobalSAC = self->m_ui64GlobalSAC;
    *pui64GlobalTime = self->m_ui64GlobalTime;
    *pui64GlobalPerformanceCounter = self->m_ui64GlobalPerformanceCounter;

    spin_unlock(&self->m_csSAC_Time_Lock);
}

///////////////////////////////////////////////////////////////////////////////
/* Multi-rate Stage 3: the sampling rate this engine's SAC is counted at. Set
 * in tic_engine_start (IO stopped); a plain word-sized read here is fine for
 * the hot path. */
uint32_t tic_engine_get_rate(TTicEngine* self)
{
    return self->m_ui32SamplingRate;
}
