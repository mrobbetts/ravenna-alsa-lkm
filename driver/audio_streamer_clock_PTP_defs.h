/****************************************************************************
*
*  Module Name    : audio_streamer_clock_PTP_defs.h
*  Version        : 
*
*  Abstract       : RAVENNA/AES67
*
*  Written by     : van Kempen Bertrand
*  Date           : 27/07/2010
*  Modified by    : 
*  Date           : 
*  Modification   : 
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

#pragma pack(push, 1)
///////////////////////////
typedef enum
{
    PTPLS_UNLOCKED	= 0,
    PTPLS_LOCKING	= 1,
    PTPLS_LOCKED	= 2
} EPTPLockStatus;

/* W16 slice 3: the media-clock state of one PCM's engine — the kernel's single
 * canonical answer to "is this clock usable, and if not, WHY". Richer than
 * EPTPLockStatus, which cannot distinguish a converging servo from one railed
 * by an untrackable GM (both reported "locking"). Derived in exactly one place
 * (tic_engine_clock_state); netlink status, the daemon mirror and the UI carry
 * these values verbatim. */
typedef enum
{
    CLK_STOPPED   = 0, /* engine not running (no started PCM on this (domain, rate)) */
    CLK_NO_SIGNAL = 1, /* no usable PTP: no GM elected / Syncs missing / PTP acquiring */
    CLK_ACQUIRING = 2, /* PTP present; media servo converging (lock hysteresis) */
    CLK_LOCKED    = 3, /* tracking the GM */
    CLK_SATURATED = 4  /* GM beyond the servo's steering range — untrackable; free-wheeling at nominal */
} EClockState;

typedef struct
{
	uint8_t		ui8Domain;
	uint8_t		ui8DSCP;
} TPTPConfig;

typedef struct
{
	EPTPLockStatus nPTPLockStatus;
	uint64_t        ui64GMID[2];
	int32_t         i32GMIDStats[2];                // 0 link down, 1 link up and locked, 2 link up not locked(i.e. lock the other one)
	int32_t         i32NetworkJitter;
	int32_t         i32ClockJitter;
	/* W16 slice 3 (appended, pack(1) — older readers that stop above still
	 * work): the elected GM's Announce properties, surfaced VERBATIM for the
	 * daemon/UI to display and judge — deliberately not gated on (a
	 * freewheeling GM is reported, never vetoed on clockClass alone) — plus
	 * the servo's estimate of the GM's rate offset vs our local reference
	 * (negative = GM slow; meaningful once PTP-locked, else 0). */
	int64_t         i64GMRateOffsetPPB;
	uint8_t         ui8GMPriority1;
	uint8_t         ui8GMClockClass;
	uint8_t         ui8GMClockAccuracy;
	uint16_t        ui16GMOffsetScaledLogVariance;
	uint8_t         ui8GMPriority2;
	uint16_t        ui16GMStepsRemoved;
	uint8_t         ui8GMTimeSource;
	/* W16 slice 3b (bench review): clock-source health is a DOMAIN-level fact —
	 * the GM + servo, one truth for every PCM on the domain. EClockState
	 * composite: NO_SIGNAL (PTP unlocked) / SATURATED (any engine railed by the
	 * GM) / ACQUIRING (any engine converging) / LOCKED. The per-PCM status
	 * carries engine-local execution health instead (see TPCMStatus). */
	int32_t         clock_state;
} TPTPStatus;

typedef struct
{
	float		fPTPSyncRatio;

	uint32_t	ui32PTPSyncMinArrivalDelta;
	uint32_t	ui32PTPSyncAvgArrivalDelta;
	uint32_t	ui32PTPSyncMaxArrivalDelta;

	uint32_t	ui32PTPFollowMinArrivalDelta;
	uint32_t	ui32PTPFollowAvgArrivalDelta;
	uint32_t	ui32PTPFollowMaxArrivalDelta;

	int32_t		i32PTPMinDeltaTICFrame;
	int32_t		i32PTPAvgDeltaTICFrame;
	int32_t		i32PTPMaxDeltaTICFrame;
} TPTPStats;

typedef struct
{
	uint32_t	ui32TICMinDelta;
	uint32_t	ui32TICMaxDelta;
} TTICStats;

#define TIMER_LATENCY_MAX       5500//1600 // [us]
#define NB_TIMER_LATENCY_RANGES 55 //16

typedef struct
{
    uint8_t     ui8NumberOfTimerLatencies;
    uint32_t    aui32TimerLatencyRanges[NB_TIMER_LATENCY_RANGES]; // ]n-1.n] in [us]
    uint64_t	aui64TimerLatencyOccurences[NB_TIMER_LATENCY_RANGES];
} TTICOSStats;



///////////////////////////
typedef enum
{	
	PTPTCFT_FILM_2398		= 1,
	PTPTCFT_FILM			= 2,
	PTPTCFT_PAL				= 3,
	PTPTCFT_NTSC_NODROP		= 4,
	PTPTCFT_NTSC_DROP		= 5,
	PTPTCFT_SMPTE_NODROP	= 6,
	PTPTCFT_SMPTE_DROP		= 7
} EPTPTimeCodeFrameType;

#pragma pack(pop)

