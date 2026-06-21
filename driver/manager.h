/****************************************************************************
*
*  Module Name    : manager.h
*  Version        : 
*
*  Abstract       : RAVENNA/AES67 ALSA LKM
*
*  Written by     : Baume Florian, Beguec Frederic
*  Date           : 29/03/2016
*  Modified by    : Baume Florian
*  Date           : 13/01/2017
*  Modification   : C port
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
//	System Includes
#include "manager_defs.h"

#include "PTP.h"
#include "RTP_streams_manager.h"
#include "module_timer.h"

#include "../common/MergingRAVENNACommon.h"
#include "../common/MT_ALSA_message_defs.h"

#include "MR_AudioDriverTypes.h"

#include "EtherTubeNetfilter.h"

#include "audio_driver.h"

#ifdef MTTRANSPARENCY_CHECK
    #include "MTTransparencyCheck.h"
#endif

#define MAX_INTERFACE_NAME 64

/*
 * Multi-PCM (multi-rate project):
 * PCM 0 is created at module load. Additional PCMs are added on demand via
 * MT_ALSA_Msg_AddPCM. All PCMs share PTP and NICs. From Stage 2 onward each
 * PCM carries its own sample rate and tick cadence (one hrtimer per unique
 * rate). MAX_PCMS bounds the per-card device index range.
 *
 * Sized for the target deployment: HT chain @ 48k + music chain at up to
 * five rates (44.1 / 48 / 88.2 / 96 / 176.4 / 192 kHz), 2 PCMs per chain
 * pair — comfortably under 16. The marginal cost of the larger array is
 * negligible (a few pointer slots per chip table).
 */
#define MAX_PCMS 16
/* Keep the ALSA layer's extra-PCM cap in lockstep — the 8→16 bump
 * originally updated only this constant and the daemon, leaving AddPCM
 * rejecting ids 8..15 ("completed must be verified at every boundary"). */
_Static_assert(MR_ALSA_MAX_EXTRA_PCMS == MAX_PCMS - 1,
               "MR_ALSA_MAX_EXTRA_PCMS (audio_driver.h) must equal MAX_PCMS - 1");

#ifndef nullptr
    #define nullptr NULL
#endif // nullptr

#include "linux/kernel.h"


//#define MT_TONE_TEST 1
//#define MT_RAMP_TEST 1

/*
 * Multi-rate W5: the (domain, tick_rate) timer registry. One entry per
 * unique tick cadence; same-rate chips share an entry (Decision 2). Each
 * entry owns its own hrtimer instance, a TIC engine per NIC servo
 * (ST2022-7 pair, slaved standby), a chip refcount (W10 RemovePCM hook)
 * and its own active-NIC selection.
 *
 * active_nic is single-writer: only this entry's timer callback
 * (manager_entry_tick → tic_entry_select_nic) writes it; every other
 * context reads with READ_ONCE. That makes per-entry NIC selection
 * race-free by construction — there is no domain-global selection state
 * (Decision 9 annotation, 2026-06-11).
 *
 * Eight valid tick rates (44.1/48/88.2/96/176.4/192/352.8/384 kHz; DSD
 * keys on its 352.8k tick-rate clock domain, not the 2.8M bit rate).
 *
 * W11: with multiple PTP domains an entry is per (domain, tick_rate), so 8
 * rates no longer bound the count. Each entry still needs ≥1 chip (chip_refcount
 * > 0), so chips bound it — size to MAX_PCMS.
 */
#define MAX_TIC_ENTRIES MAX_PCMS

/* W11: PTP servos are instanced per (NIC, domain). A card supplies exactly one
 * domain and cards may share, so distinct domains can never exceed the card
 * count — size the per-domain dimension to MR_ALSA_MAX_CARDS. */
#define MAX_DOMAINS MR_ALSA_MAX_CARDS

struct TManager;

struct tic_timer_entry
{
    bool active;                        /// slot in use; published with release
    uint8_t domain;                     /// PTP domain (0 until W11)
    uint32_t tick_rate;                 /// registry key (DSD: 352800)
    uint32_t frame_size;                /// samples per tick at tick_rate
    unsigned int chip_refcount;         /// chips bound to this cadence
    volatile unsigned short active_nic; /// single-writer: this entry's timer callback
    TTicEngine engine[_MAX_NICS];       /// one per NIC servo
    struct clock_timer timer;           /// this entry's own hrtimer
    struct TManager* mgr;
};

struct TManager
{
    bool m_Is_NIC_Active[_MAX_NICS];

    TEtherTubeNetfilter m_EthernetFilter[_MAX_NICS];
    /* W11: one PTP servo per (NIC, domain). Domains index directly (slot ==
     * domain number, capped at MAX_DOMAINS); a tic_timer_entry attaches its
     * engine[nic] to m_PTP[nic][entry->domain]. Static resources — no per-domain
     * refcount/registry; the entry's chip_refcount already governs attach/detach. */
    TClock_PTP m_PTP[_MAX_NICS][MAX_DOMAINS];
    TRTP_streams_manager m_RTP_streams_manager;
    uint32_t m_NumberOfInputs;
    uint32_t m_NumberOfOutputs;
    uint64_t m_RingBufferFrameSize;
    /* W14: NOT authoritative anymore. Every chip carries its own rate (set at
     * AddPCM, stored per-chip); there is no chip-0 special path. This stays at
     * DEFAULT_SAMPLERATE (never mutated at runtime — the daemon no longer sends
     * SetSampleRate) purely as a benign rate-CLASS default for a few global
     * readers (audio sample-format / RTP mute pattern / tone test) whose
     * per-PCM refactor is deferred; those only distinguish PCM vs DSD, and all
     * current chips are PCM. Retire fully when those go per-PCM (W14b). */
    uint32_t m_SampleRate;
    enum eAudioMode m_AudioMode;

    uint64_t m_TICFrameSizeAt1FS;
    uint32_t m_ui32FrameSize;
    uint32_t m_MaxFrameSize;

    /* W9 #14: playout/capture delay moved onto each chip (per-pcm_id). */

    bool m_bIsPlaybackIO;
    bool m_bIsRecordingIO;

    volatile bool m_bIsStarted;
    volatile bool m_bIORunning;

    /* 2026-06-09 review fix (io-flags race): spinlock_t*, allocated in
     * init(). Serializes recompute_global_io_flags so concurrent ALSA
     * triggers on different chips can't lose each other's updates. */
    void* m_csIOState;

    char m_cInterfaceName[MAX_INTERFACE_NAME];


    rtp_audio_stream_ops m_c_callbacks;
    //dispatch_packet_ops m_c_dispatch_callbacks;
    clock_ptp_ops m_c_audio_streamer_clock_PTP_callback;


// ALSA <> Manager communication

    void* m_apALSAChip[MAX_PCMS];                   /// per-PCM ALSA chip pointers (struct mr_alsa_audio_chip*); slot 0 is the default PCM created at probe
    uint32_t m_uPCMCount;                           /// number of valid entries in m_apALSAChip
    const struct ravenna_mgr_ops *m_alsa_driver_frontend;   /// Manager to ALSA driver (e.g. buffers access and lock)
    struct alsa_ops m_alsa_callbacks;                /// ALSA driver to Manager (e.g. audio setup at runtime)

    /* Multi-rate W5: the (domain, tick_rate) timer registry, and the
     * chip → entry map (the per-chip clock handle — the W11 chokepoint).
     * m_apChipEntry slots are published with release before the matching
     * m_apALSAChip slot, read with acquire from the tick/RTP paths. */
    struct tic_timer_entry m_TicTimers[MAX_TIC_ENTRIES];
    struct tic_timer_entry* m_apChipEntry[MAX_PCMS];
};


bool init(struct TManager* self, int* errorCode);
void destroy(struct TManager* self);

bool start(struct TManager* self);
bool stop(struct TManager* self);

/* 2026-06-09 review fix (cross-chip mute wipe): startIO/stopIO take the
 * triggering chip so the mute targets THAT chip's ring buffers — a trigger
 * on PCM 1 must never memset PCM 0's live audio. alsa_chip_pointer may be
 * NULL (mute skipped, IO flags still recomputed). */
bool startIO(struct TManager* self, void* alsa_chip_pointer, bool is_playback);
bool stopIO(struct TManager* self, void* alsa_chip_pointer, bool is_playback);

bool SetInterfaceName(struct TManager* self, const char* cInterfaceName, const int iEthFilterIndex);
bool SetSamplingRate(struct TManager* self, uint32_t SamplingRate);
bool SetDSDSamplingRate(struct TManager* self, uint32_t SamplingRate);
bool SetTICFrameSizeAt1FS(struct TManager* self, uint64_t TICFrameSize);
bool SetMaxTICFrameSize(struct TManager* self, uint64_t max_frameSize);
bool SetNumberOfInputs(struct TManager* self, uint32_t NumberOfChannels);
bool SetNumberOfOutputs(struct TManager* self, uint32_t NumberOfChannels);

/* W5: per-(domain, rate) tick — called from each entry's hrtimer callback
 * via t_clock_timer_tick. Advances both NIC engines, runs the servo
 * periodic checks, re-evaluates this entry's NIC selection, programs the
 * next wakeup from the active engine, rephases the standby, and pumps
 * audio for this entry. */
void manager_entry_tick(struct tic_timer_entry* entry, uint64_t* pui64NextRTXClockTime, uint64_t ui64Now);

bool IsStarted(struct TManager* self);
bool IsIOStarted(struct TManager* self);

// Netfilter
int EtherTubeRxPacket(struct TManager* self, void* packet, int packet_size, const char* ifname, int mac_header);
void EtherTubeHookFct(struct TManager* self, void* hook_fct, void* hook_struct);

// Messaging
void OnNewMessage(struct TManager* self, struct MT_ALSA_msg* msg_rcv);

// Statistics
bool GetHALToTICDelta(struct TManager* self, THALToTICDelta* pHALToTICDelta);
//mutable CMTAL_CriticalSection	m_csStats;
//CMTAL_PerfMonMinMax<int32_t>	m_pmmmHALToTICDelta;

void UpdateFrameSize(struct TManager* self);

/*
 * Multi-rate Stage 2: derive frame_size (samples per TIC tick) from a
 * sample rate. Pure function — no manager state needed beyond the
 * TICFrameSizeAt1FS / MaxTICFrameSize constants the caller passes in.
 *
 * Extracted from UpdateFrameSize so per-chip rate code paths can call it
 * without going through manager-wide state. UpdateFrameSize itself stays
 * (operating on manager-wide m_SampleRate / m_ui32FrameSize) as the
 * transitional reader for legacy chip-0-only code paths.
 */
uint32_t compute_frame_size_for_rate(uint32_t sample_rate, uint64_t tic_frame_size_at_1fs, uint32_t max_frame_size);

/*
 * Multi-rate Stage 2: validate a sample rate against the supported PCM
 * set (44.1/48k families through 8FS). DSD raw rates (2822400 etc.) are
 * deliberately NOT accepted via AddPCM — DSD streaming uses the
 * MT_ALSA_Msg_SetDSDSampleRate path on the manager-wide rate, and per-PCM
 * DSD is out of scope for Stage 2.
 */
bool is_valid_pcm_rate(uint32_t sample_rate);

void MuteInputBuffer(struct TManager* self, void* alsa_chip_pointer);
void MuteOutputBuffer(struct TManager* self, void* alsa_chip_pointer);

uint32_t GetTICFrameSizeAt1FS(struct TManager* self);
uint32_t GetMaxTICFrameSize(struct TManager* self);

// Caudio_streamer_clock_PTP_callback
// C++ style
uint32_t GetIPAddress(void* user);// TODO
void AudioFrameTIC(void* user);
// C style
//static void AudioFrameTIC_(void* self) { return ((CManager*)self)->AudioFrameTIC(); }
//static uint32_t GetIPAddress_(void* self) { return ((CManager*)self)->GetIPAddress(); }
// CEtherTubeAdviseSink
EDispatchResult DispatchPacket(struct TManager* self, void* pBuffer, uint32_t packetsize, int mac_header, unsigned char nicId);

//////////////////////////////////////
// Ex-CRTP_audio_stream_callback was defined in RTP_audio_stream.hpp
uint64_t get_global_SAC(void* user);
uint64_t get_global_time(void* user);
void get_global_times(void* user, uint64_t* pui64GlobalSAC, uint64_t* pui64GlobalTime, uint64_t* pui64GlobalPerformanceCounter);
uint32_t get_frame_size(void* user);
void get_audio_engine_sample_format(void* user, enum EAudioEngineSampleFormat* pnSampleFormat);
char get_audio_engine_sample_bytelength(void* user);
void* get_live_in_jitter_buffer(void* user, uint32_t ulChannelId);	// Note: buffer type is retrieved through get_audio_engine_sample_format
void* get_live_out_jitter_buffer(void* user, uint32_t ulChannelId);	// Note: buffer type is retrieved through get_audio_engine_sample_format
/* Stage 1 multi-PCM: per-PCM variant for stream Init buffer cache. */
void* get_live_buffer_for_pcm(void* user, uint32_t pcm_id, uint32_t ulChannelId, int is_capture);
uint32_t get_live_in_jitter_buffer_length(void* user);
uint32_t get_live_out_jitter_buffer_length(void* user);
uint32_t get_live_in_jitter_buffer_offset(void* user, const uint64_t ui64CurrentSAC);
uint32_t get_live_out_jitter_buffer_offset(void* user, const uint64_t ui64CurrentSAC);
int update_live_in_audio_data_format(void* user, uint32_t /*ulChannelId*/, char const * /*pszCodec*/);
unsigned char get_live_in_mute_pattern(void* user, uint32_t ulChannelId);
unsigned char get_live_out_mute_pattern(void* user, uint32_t /*ulChannelId*/);

/*
 * Multi-rate Stage 2: per-PCM tick-path callbacks. Each RTP stream
 * carries a pcm_id (TRTP_stream_info::m_uiPCMId, Stage 1) and queries
 * these variants on the hot path so frame size / buffer length / mute
 * pattern route to the owning chip's per-PCM state.
 *
 * All variants do an acquire-load on the chip slot via the manager's
 * m_apALSAChip[] array. If pcm_id is out of range or the slot is empty
 * they return a safe zero/no-op value (length=0, offset=0, pattern=0).
 *
 * For chips at the same sample rate these return identical values to
 * the manager-wide callbacks above; they diverge when chips run at
 * different rates (different frame_size per tick, different buffer
 * lengths if Stage 4 grows per-chip ring-buffer sizing).
 */
/* Per-PCM SAC: the chip's media clock (sample count at the chip's own
 * rate), derived from the anchoring PTP clock's SAC scaled by
 * chip_rate/clock_rate. Identity when chip_rate == clock_rate. */
uint64_t get_global_SAC_for_pcm(void* user, uint32_t pcm_id);
/* W5 step 3: tick-rate (registry-key) resolution for the pump filter. */
uint32_t get_tick_rate_for_pcm(void* user, uint32_t pcm_id);
uint32_t get_frame_size_for_pcm(void* user, uint32_t pcm_id);
uint32_t get_live_in_jitter_buffer_length_for_pcm(void* user, uint32_t pcm_id);
uint32_t get_live_out_jitter_buffer_length_for_pcm(void* user, uint32_t pcm_id);
uint32_t get_live_in_jitter_buffer_offset_for_pcm(void* user, uint32_t pcm_id, const uint64_t ui64CurrentSAC);
uint32_t get_live_out_jitter_buffer_offset_for_pcm(void* user, uint32_t pcm_id, const uint64_t ui64CurrentSAC);
unsigned char get_live_in_mute_pattern_for_pcm(void* user, uint32_t pcm_id, uint32_t ulChannelId);
unsigned char get_live_out_mute_pattern_for_pcm(void* user, uint32_t pcm_id, uint32_t ulChannelId);


void Init_C_Callbacks(struct TManager* self);
rtp_audio_stream_ops* Get_C_Callbacks(struct TManager* self);
//dispatch_packet_ops* Get_C_Dispatch_Callbacks() {return &m_c_dispatch_callbacks;}


int attach_alsa_driver(void* user, const struct ravenna_mgr_ops *ops, void *alsa_chip_pointer, int pcm_id);
void detach_alsa_driver(void* user, void *alsa_chip_pointer);
/*
 * Returns m_apALSAChip[pcm_id], or NULL if pcm_id is out of range or that
 * slot is empty. Stage 1 transitional helper: most code paths still operate
 * on the default chip (id 0). Tasks 5-7 add per-PCM dispatch.
 */
void* get_chip_by_pcm_id(struct TManager* self, int32_t pcm_id);
void init_alsa_callbacks(struct TManager* self);
int get_input_jitter_buffer_offset(void* user, uint32_t *offset);
/* 2026-06-09 review fix (chip-N capture misalignment): per-PCM variant —
 * chip N's capture prepare must align to ITS ring length and ITS SAC,
 * not chip 0's. */
int get_input_jitter_buffer_offset_for_pcm(void* user, uint32_t pcm_id, uint32_t *offset);
int get_output_jitter_buffer_offset(void* user, uint32_t *offset);
int get_min_interrupts_frame_size(void* user, uint32_t *framesize);
int get_max_interrupts_frame_size(void* user, uint32_t *framesize);
int get_interrupts_frame_size(void* user, uint32_t *framesize); // ALSA PCM period size must be a multiple of this framesize
int set_sample_rate(void* user, uint32_t rate);
int get_sample_rate(void* user, uint32_t *rate);
int get_jitter_buffer_sample_bytelength(void* user, char *byte_len);
//int set_nb_inputs(void* user, uint32_t nb_channels);
//int set_nb_outputs(void* user, uint32_t nb_channels);
int get_nb_inputs(void* user, uint32_t *nb_Channels);
int get_nb_outputs(void* user, uint32_t *nb_Channels);
int start_interrupts(void* user, void* alsa_chip_pointer, bool is_playback);
int stop_interrupts(void* user, void* alsa_chip_pointer, bool is_playback);
int notify_master_volume_change(void* user, int direction, int32_t value);
int notify_master_switch_change(void* user, int direction, int32_t value);
int get_master_volume_value(void* user, int direction, int32_t* value);
int get_master_switch_value(void* user, int direction, int32_t* value);

// helpers
bool IsDSDRate(const uint32_t sample_rate);
enum eAudioMode GetAudioModeFromRate(const uint32_t sample_rate);

// Debug
#ifdef MTTRANSPARENCY_CHECK
    CMTTransparencyCheck    m_Transparencycheck;
#endif

#if defined(MT_TONE_TEST)
    unsigned long m_tone_test_phase;
#elif defined(MT_RAMP_TEST)
    int32_t m_ramp_test_phase;
#endif // MT_TONE_TEST

