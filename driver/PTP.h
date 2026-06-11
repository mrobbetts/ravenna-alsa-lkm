/****************************************************************************
*
*  Module Name    : PTP.h
*  Version        :
*
*  Abstract       : RAVENNA/AES67 ALSA LKM
*
*  Written by     : van Kempen Bertrand
*  Date           : 27/07/2010
*  Modified by    : Baume Florian
*  Date           : 14/04/2016
*  Modification   : Linux driver port, removed floating point,
*                   changed time ref unit, stabiliy increased
*  Modified by    : multi-pcm-stage1 fork
*  Date           : 06/2026
*  Modification   : multi-rate W5 — TClock_PTP slimmed to PTP discipline;
*                   rate-keyed TIC state moved to TTicEngine (tic_engine.h)
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

#include "MTAL_stdint.h"
#include "MTAL_EthUtils.h"

#include "audio_streamer_clock_PTP_defs.h"
#include "EtherTubeNetfilter.h"
#include "PTP_defs.h"
#include "tic_engine.h"

/*
 * Multi-rate W5: TClock_PTP is now the pure PTP DISCIPLINE for one NIC —
 * packet processing, master election, the PTP lock counter, the T1/T2
 * measurements and the RTX<->PTP clock offset. Everything that was keyed on
 * a single rate/frame size (tick period steering, TIC lock, TICSAC and the
 * published (SAC, time, perf) triple) lives in TTicEngine (tic_engine.h),
 * and the servo fans its measurements out to the engines attached to it.
 *
 * Step 1 of the extraction: exactly ONE engine, embedded (m_TicEngine) and
 * attached at init — behaviorally identical to the pre-extraction code.
 * Step 2 moves engine ownership into the manager's (domain, rate) timer
 * registry; the attach array is already the final shape.
 *
 * The struct is now named (TClock_PTP_s) so tic_engine.h can hold a
 * backpointer without a circular include.
 */
typedef struct TClock_PTP_s
{
    TEtherTubeNetfilter *m_pEth_netfilter;

    bool m_bInitialized;
    clock_ptp_ops* m_audio_streamer_clock_PTP_callback_ptr;

    void* m_csPTPTime;
    //CMTAL_CriticalSection m_csPTPTime;

    uint16_t m_usPTPLockCounter; // m_usPTPLockCounter == 0 means that PTP(sync + follow) are good and stable

    volatile int64_t m_i64TIC_PTPToRTXClockOffset; // [100us]

    // PTP Master
    volatile uint16_t m_usPTPMasterPortNumber;


	uint16_t m_wLastAnnounceSequenceId;
    uint16_t m_wLastSyncSequenceId;
    uint16_t m_wLastFollowUp;


    uint64_t m_ui64T1; //[100us]
    uint64_t m_ui64T2; //[100us]
    uint64_t m_ui64DeltaT2; //[100us]

    uint16_t m_wLastDelayReqSequenceId;
    TPTPV2MsgDelayReqPacket m_PTPV2MsgDelayReqPacket;

    // PTP WatchDog
    uint64_t m_ui64LastWatchDogTime;
    uint16_t m_wLastWatchDogSyncSequenceId;

    uint64_t m_ui64PTP_GMID;
    uint8_t m_ui8PTPClockDomain;

    //######################################################
    TPTPConfig m_PTPConfig;
    uint32_t m_ui32PTPConfigChangedCounter;
    uint32_t m_ui32LastPTPConfigChangedCounter;

    TV2MsgAnnounce m_PTPMaster_Announce;
    uint64_t m_ui64PTPMaster_AnnounceTime;

    uint64_t m_ui64PTPMaster_ClockIdentity;
    uint64_t m_ui64PTPMaster_GMID;
    //######################################################

    // Multi-rate W5: TIC engines disciplined by this servo. The servo's
    // sync handler / ProcessT1 / ResetPTPLock fan out to every attached
    // engine. Steering state inside the engines is protected by THIS
    // servo's m_csPTPTime.
    TTicEngine m_TicEngine; /* step 1: the single engine, embedded */
    TTicEngine* m_apEngines[MAX_TIC_ENGINES_PER_SERVO];
    unsigned int m_uNumEngines;

} TClock_PTP;



#if defined(__cplusplus)
extern "C"
{
#endif // defined(__cplusplus) f10b pourra etre retire  +extern quand le port C sera termine

 void get_ptp_global_times(TClock_PTP* self, uint64_t* pui64GlobalSAC, uint64_t* pui64GlobalTime, uint64_t* pui64GlobalPerformanceCounter); // get the time and the SAC atomically

 bool init_ptp(TClock_PTP* self, TEtherTubeNetfilter* pEth_netfilter, clock_ptp_ops* audio_streamer_clock_PTP_callback_ptr);
 void destroy_ptp(TClock_PTP* self);

 void SetPTPMasterPortNumber(TClock_PTP* self, uint16_t usPTPMasterPortNumber);

 EDispatchResult process_PTP_packet(TClock_PTP* self, TUDPPacketBase* pUDPPacketBase, uint32_t ui32PacketSize);

 bool StartAudioFrameTICTimer(TClock_PTP* self, uint32_t ui32FrameSize, uint32_t ui32SamplingRate);
 bool StopAudioFrameTICTimer(TClock_PTP* self);
 bool IsAudioFrameTICDropped(TClock_PTP* self, bool bReset);

 /* Composite (PTP + TIC) status of this servo's single step-1 engine.
  * Step 2+ callers that care about a specific rate use
  * tic_engine_lock_status() on that entry's engine directly. */
 EPTPLockStatus GetLockStatus(TClock_PTP* self);

 void SetPTPConfig(TClock_PTP* self, TPTPConfig* pPTPConfig);
 void GetPTPConfig(TClock_PTP* self, TPTPConfig* pPTPConfig);

 void GetPTPStatus(TClock_PTP* self, TPTPStatus* pPTPStatus);
 uint8_t GetPTPPriority(TClock_PTP* self);

 uint64_t get_ptp_global_SAC(TClock_PTP* self);
 uint64_t get_ptp_global_time(TClock_PTP* self);
 /* Multi-rate Stage 3: rate this clock's SAC is counted at (for per-PCM SAC scaling). */
 uint32_t get_ptp_sampling_rate(TClock_PTP* self);

 void ResetPTPLock(TClock_PTP* self, bool bUseMutex);

 //######################################################
 void ResetPTPMaster(TClock_PTP* self);
 //######################################################

 void ProcessT1(TClock_PTP* self, uint64_t ui64T1); // from Sync or Follow_up

 bool SendDelayReq(TClock_PTP* self, TPTPV2MsgFollowUpPacket* pPTPV2MsgFollowUpPacket);

// Timer
// Audio TIC
 void timerSetNextAbsoluteTime(TClock_PTP* self, uint64_t ui64NextAbsoluteTime);
 void timerProcess(TClock_PTP* self, uint64_t* pui64NextRTXClockTime, uint64_t ui64RTXClockTime);


#if defined(__cplusplus)
}
#endif  // defined(__cplusplus)
