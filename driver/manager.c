/****************************************************************************
*
*  Module Name    : manager.c
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

#include "manager.h"
#include "EtherTubeNetfilter.h"
#include <linux/errno.h>
#include <linux/compiler.h>  /* READ_ONCE / WRITE_ONCE / smp_*_acquire/release */
#include <linux/spinlock.h>  /* m_csIOState (io-flags recompute serialization) */
#include <linux/slab.h>      /* kmalloc/kfree for m_csIOState */

#include "RTP_stream_info.h"

#include "module_timer.h"
#include "c_wrapper_lib.h"

#if defined(DEBUG) || defined(_DEBUG)
    #define DebugMsg(x) MTAL_DP(x)
#else
    #define DebugMsg(x)
#endif // defined(DEBUG) || defined(_DEBUG)

#define MTLOOPBACK_CHANNEL_IDX 63


#ifdef MT_TONE_TEST

///  Basic LUT of 1k sine for SR=48000, 16bit.
static int sinebuf[48] = {
    0, 4276, 8480, 12539, 16383, 19947, 23169, 25995,
    28377, 30272, 31650, 32486, 32767, 32486, 31650, 30272,
    28377, 25995, 23169, 19947, 16383, 12539, 8480, 4276,
    0, -4276, -8480, -12539, -16383, -19947, -23169, -25995,
    -28377, -30272, -31650, -32486, -32767, -32486, -31650, -30272,
    -28377, -25995, -23169, -19947, -16383, -12539, -8480, -4276
  };

static int cosbuf[48] = {
    32767,32488,31651,30274,28378,25997,23170,19948,16384,12540,8481,4277,0,-4277,-8481,-12540,
    -16384,-19948,-23170,-25997,-28378,-30274,-31651,-32488,-32768,-32488,-31651,-30274,-28378,-25997,-23170,-19948,
    -16384,-12540,-8481,-4277,-0,4277,8481,12540,16384,19948,23170,25997,28378,30274,31651,32488
  };


///  Basic LUT of 1k sine for SR=96000, 16bit.
  static int sinebuf_96k[96] = {
        0,2143,4277,6393,8481,10533,12540,14493,16384,18205,19948,21605,23170,24636,25997,27246,
        28378,29389,30274,31029,31651,32138,32488,32698,32767,32698,32488,32138,31651,31029,30274,29389,
        28378,27246,25997,24636,23170,21605,19948,18205,16384,14493,12540,10533,8481,6393,4277,2143,
        0,-2143,-4277,-6393,-8481,-10533,-12540,-14493,-16384,-18205,-19948,-21605,-23170,-24636,-25997,-27246,
        -28378,-29389,-30274,-31029,-31651,-32138,-32488,-32698,-32768,-32698,-32488,-32138,-31651,-31029,-30274,-29389,
        -28378,-27246,-25997,-24636,-23170,-21605,-19948,-18205,-16384,-14493,-12540,-10533,-8481,-6393,-4277,-2143
    };

///  Basic LUT of 1k sine for SR=192000, 16bit.
    static int sinebuf_192k[192] = {
        0,1072,2143,3212,4277,5338,6393,7441,8481,9512,10533,11543,12540,13524,14493,15447,
        16384,17304,18205,19087,19948,20788,21605,22400,23170,23916,24636,25330,25997,26635,27246,27827,
        28378,28899,29389,29847,30274,30668,31029,31357,31651,31912,32138,32330,32488,32610,32698,32750,
        32767,32750,32698,32610,32488,32330,32138,31912,31651,31357,31029,30668,30274,29847,29389,28899,
        28378,27827,27246,26635,25997,25330,24636,23916,23170,22400,21605,20788,19948,19087,18205,17304,
        16384,15447,14493,13524,12540,11543,10533,9512,8481,7441,6393,5338,4277,3212,2143,1072,
        0,-1072,-2143,-3212,-4277,-5338,-6393,-7441,-8481,-9512,-10533,-11543,-12540,-13524,-14493,-15447,
        -16384,-17304,-18205,-19087,-19948,-20788,-21605,-22400,-23170,-23916,-24636,-25330,-25997,-26635,-27246,-27827,
        -28378,-28899,-29389,-29847,-30274,-30668,-31029,-31357,-31651,-31912,-32138,-32330,-32488,-32610,-32698,-32750,
        -32768,-32750,-32698,-32610,-32488,-32330,-32138,-31912,-31651,-31357,-31029,-30668,-30274,-29847,-29389,-28899,
        -28378,-27827,-27246,-26635,-25997,-25330,-24636,-23916,-23170,-22400,-21605,-20788,-19948,-19087,-18205,-17304,
        -16384,-15447,-14493,-13524,-12540,-11543,-10533,-9512,-8481,-7441,-6393,-5338,-4277,-3212,-2143,-1072
    };

///  Basic LUT of 1k sine for SR=384000, 16bit.
    static int sinebuf_384k[384] = {
        0,536,1072,1608,2143,2678,3212,3745,4277,4808,5338,5866,6393,6918,7441,7962,
        8481,8998,9512,10024,10533,11039,11543,12043,12540,13033,13524,14010,14493,14972,15447,15917,
        16384,16846,17304,17757,18205,18648,19087,19520,19948,20371,20788,21199,21605,22006,22400,22788,
        23170,23546,23916,24279,24636,24986,25330,25667,25997,26320,26635,26944,27246,27540,27827,28106,
        28378,28642,28899,29148,29389,29622,29847,30064,30274,30475,30668,30853,31029,31197,31357,31508,
        31651,31786,31912,32029,32138,32239,32330,32413,32488,32553,32610,32658,32698,32729,32750,32764,
        32767,32764,32750,32729,32698,32658,32610,32553,32488,32413,32330,32239,32138,32029,31912,31786,
        31651,31508,31357,31197,31029,30853,30668,30475,30274,30064,29847,29622,29389,29148,28899,28642,
        28378,28106,27827,27540,27246,26944,26635,26320,25997,25667,25330,24986,24636,24279,23916,23546,
        23170,22788,22400,22006,21605,21199,20788,20371,19948,19520,19087,18648,18205,17757,17304,16846,
        16384,15917,15447,14972,14493,14010,13524,13033,12540,12043,11543,11039,10533,10024,9512,8998,
        8481,7962,7441,6918,6393,5866,5338,4808,4277,3745,3212,2678,2143,1608,1072,536,
        0,-536,-1072,-1608,-2143,-2678,-3212,-3745,-4277,-4808,-5338,-5866,-6393,-6918,-7441,-7962,
        -8481,-8998,-9512,-10024,-10533,-11039,-11543,-12043,-12540,-13033,-13524,-14010,-14493,-14972,-15447,-15917,
        -16384,-16846,-17304,-17757,-18205,-18648,-19087,-19520,-19948,-20371,-20788,-21199,-21605,-22006,-22400,-22788,
        -23170,-23546,-23916,-24279,-24636,-24986,-25330,-25667,-25997,-26320,-26635,-26944,-27246,-27540,-27827,-28106,
        -28378,-28642,-28899,-29148,-29389,-29622,-29847,-30064,-30274,-30475,-30668,-30853,-31029,-31197,-31357,-31508,
        -31651,-31786,-31912,-32029,-32138,-32239,-32330,-32413,-32488,-32553,-32610,-32658,-32698,-32729,-32750,-32764,
        -32768,-32764,-32750,-32729,-32698,-32658,-32610,-32553,-32488,-32413,-32330,-32239,-32138,-32029,-31912,-31786,
        -31651,-31508,-31357,-31197,-31029,-30853,-30668,-30475,-30274,-30064,-29847,-29622,-29389,-29148,-28899,-28642,
        -28378,-28106,-27827,-27540,-27246,-26944,-26635,-26320,-25997,-25667,-25330,-24986,-24636,-24279,-23916,-23546,
        -23170,-22788,-22400,-22006,-21605,-21199,-20788,-20371,-19948,-19520,-19087,-18648,-18205,-17757,-17304,-16846,
        -16384,-15917,-15447,-14972,-14493,-14010,-13524,-13033,-12540,-12043,-11543,-11039,-10533,-10024,-9512,-8998,
        -8481,-7962,-7441,-6918,-6393,-5866,-5338,-4808,-4277,-3745,-3212,-2678,-2143,-1608,-1072,-536
    };

#endif // MT_TONE_TEST

//////////////////////////////////////////////////////////////////////////////////
/* W15: writers-only registry lock. Serializes the (domain,rate) table mutators
 * — get_or_create_tic_entry / put_tic_entry and the chip->entry pointer swap —
 * against each other. Until W15 those ran on the netlink thread alone (serialized
 * for free); the in-place re-rate adds a SECOND mutator context (pcm_close, the
 * app's thread), so they now need an explicit lock. The softirq tick is a pure
 * reader (smp_load_acquire on entry->active + m_apChipEntry) and NEVER takes
 * this lock — readers stay wait-free. Deadlock rule: a holder must never block
 * on a PCM open/close while holding it (notably: teardown_card detaches every
 * chip — under this lock, via detach_alsa_driver — and only THEN calls
 * snd_card_free, which waits for closes; the lock is not held across that). */
static DEFINE_MUTEX(g_registry_lock);

//////////////////////////////////////////////////////////////////////////////////
/* W5 registry helpers (definitions follow Set* functions, which use them) */
static uint32_t tick_rate_for_sample_rate(uint32_t sample_rate);
static void tic_entry_refresh_base_period(struct tic_timer_entry* entry);
static void tic_entry_start(struct TManager* self, struct tic_timer_entry* entry, bool bResetPTPLock);
static void tic_entry_stop(struct TManager* self, struct tic_timer_entry* entry);
static struct tic_timer_entry* get_or_create_tic_entry(struct TManager* self, uint8_t domain, uint32_t sample_rate);
static void put_tic_entry(struct TManager* self, struct tic_timer_entry* entry);
static void manager_audio_frame_tic(struct TManager* self, struct tic_timer_entry* entry);

//////////////////////////////////////////////////////////////////////////////////
bool init(struct TManager* self, int* errorCode)
{
    bool theAnswer = true;
    int err = 0;
    int i = 0;

    self->m_Is_NIC_Active[0] = true;
    self->m_Is_NIC_Active[1] = false;
    
    /* W5: per-entry NIC selection — no domain-global selection state.
     * Registry slots and the chip->entry map start empty; chip 0's entry
     * is created when its chip attaches during card init below. */
    memset(self->m_TicTimers, 0, sizeof(self->m_TicTimers));
    memset(self->m_apChipEntry, 0, sizeof(self->m_apChipEntry));

    self->m_bIsStarted = false;
    self->m_bIORunning = false;
    memset(self->m_apALSAChip, 0, sizeof(self->m_apALSAChip));
    self->m_uPCMCount = 0;
    self->m_alsa_driver_frontend = NULL;
    self->m_bIsPlaybackIO = false;
    self->m_bIsRecordingIO = false;

    self->m_csIOState = kmalloc(sizeof(spinlock_t), GFP_KERNEL);
    if (!self->m_csIOState)
    {
        MTAL_DP("CManager::init: failed to allocate IO-state lock\n");
        if (errorCode)
            *errorCode = -ENOMEM;
        return false;
    }
    spin_lock_init((spinlock_t*)self->m_csIOState);

    memset(self->m_cInterfaceName, 0, MAX_INTERFACE_NAME);

    //  initialize the stuff tracked by the IORegistry
    self->m_SampleRate = DEFAULT_SAMPLERATE;
    SetSamplingRate(self, self->m_SampleRate);

    self->m_TICFrameSizeAt1FS = DEFAULT_NADAC_TICFRAMESIZE;
    self->m_MaxFrameSize = MAX_HORUS_SUPPORTED_FRAMESIZE_IN_SAMPLES;
    SetTICFrameSizeAt1FS(self, self->m_TICFrameSizeAt1FS);

    self->m_RingBufferFrameSize = RINGBUFFERSIZE;

    /* W9 #14: playout/capture delay moved off the manager onto each chip
     * (set per-pcm_id via Set{Playout,Capture}Delay, read at prepare()). */

    self->m_NumberOfInputs = DEFAULT_NUMBEROFINPUTS;
    SetNumberOfInputs(self, self->m_NumberOfInputs);

    self->m_NumberOfOutputs = DEFAULT_NUMBEROFOUTPUTS;
    SetNumberOfOutputs(self, self->m_NumberOfOutputs);

    init_alsa_callbacks(self);
    Init_C_Callbacks(self);

    // initialize the Ethernet Filter
    for (i = 0; i < _MAX_NICS; i++)
    {
        if (!InitEtherTube(&self->m_EthernetFilter[i], self, i))
        {
            MTAL_DP("CManager::init: self->m_EthernetFilter.Init() failed\n");
            theAnswer = false;
            err = -EINVAL;
            goto Failure;
        }
    }

    for (i = 0; i < _MAX_NICS; i++)
    {
        unsigned int d;
        for (d = 0; d < MAX_DOMAINS; d++)
        {
            /* W11: a servo per (NIC, domain), all sharing this NIC's filter.
             * Configured to its domain by SetPTPConfig; the feed-all dispatch
             * lets each self-filter incoming PTP by byDomainNumber. */
            if (!init_ptp(&self->m_PTP[i][d], &self->m_EthernetFilter[i], &self->m_c_audio_streamer_clock_PTP_callback))
            {
                DebugMsg("CManager::init: self->m_PTP.Init() failed");
                theAnswer = false;
                err = -EINVAL;
                goto Failure;
            }
        }
    }

    if (!init_(&self->m_RTP_streams_manager, Get_C_Callbacks(self), self->m_EthernetFilter))
    {
        MTAL_DP("CManager::init: self->m_RTP_streams_manager.init() failed\n");
        theAnswer = false;
        err = -EINVAL;
        goto Failure;
    }
    err = mr_alsa_audio_card_init(self, &self->m_alsa_callbacks);
    if (err != 0)
    {
        MTAL_DP("CManager::init: mr_alsa_audio_card_init() failed\n");
        theAnswer = false;
        goto Failure;
    }

    if (errorCode != nullptr)
        *errorCode = 0;
    return theAnswer;

Failure:
    MTAL_DP("CManager::init failed\n");
    if(errorCode != nullptr)
        *errorCode = err;
    return theAnswer;
}

//////////////////////////////////////////////////////////////////////////////////
void destroy(struct TManager* self)
{
    int i = 0;
    for (i = 0; i < _MAX_NICS; i++)
    {
        MTAL_DP("CManager::destroy : self->m_EthernetFilter.Stop() succeeded\n");
        if (!Stop(&self->m_EthernetFilter[i]))
        {
            MTAL_DP("CManager::destroy : self->m_EthernetFilter.Stop() failed\n");
        }
        else
        {
            MTAL_DP("CManager::destroy : self->m_EthernetFilter.Stop() succeeded\n");
        }
    }
    stop(self);

    mr_alsa_audio_card_exit();
    destroy_(&self->m_RTP_streams_manager);
    /* W5 safety sweep: card_exit should drop every chip's entry reference
     * via detach_alsa_driver, but the unload path is known-leaky (W10:
     * card_free doesn't clear m_apALSAChip[]). Force-free any entry still
     * active so engine SAC locks aren't leaked and its hrtimer is
     * cancelled before the servos are destroyed. */
    {
        unsigned int t;
        /* W15: hold the registry lock for the put_tic_entry invariant (module
         * unload, uncontended — no opens possible here). */
        mutex_lock(&g_registry_lock);
        for (t = 0; t < MAX_TIC_ENTRIES; t++)
        {
            struct tic_timer_entry* entry = &self->m_TicTimers[t];
            if (!entry->active)
                continue;
            entry->chip_refcount = 1;
            put_tic_entry(self, entry);
        }
        mutex_unlock(&g_registry_lock);
    }
    for (i = 0; i < _MAX_NICS; i++)
    {
        unsigned int d;
        for (d = 0; d < MAX_DOMAINS; d++)
            destroy_ptp(&self->m_PTP[i][d]);
    }
    for (i = 0; i < _MAX_NICS; i++)
    {
        DestroyEtherTube(&self->m_EthernetFilter[i]);
    }

    if (self->m_csIOState)
    {
        kfree(self->m_csIOState);
        self->m_csIOState = NULL;
    }
}

//////////////////////////////////////////////////////////////////////////////////
bool start(struct TManager* self)
{
    int i = 0;
    unsigned int t;
    /* W5: per-entry engines + hrtimer. Entry metadata (tick_rate,
     * frame_size) already tracks m_SampleRate via UpdateFrameSize;
     * bResetPTPLock=true preserves the legacy StartAudioFrameTICTimer
     * relock semantics on the start path. */
    for (t = 0; t < MAX_TIC_ENTRIES; t++)
    {
        struct tic_timer_entry* entry = &self->m_TicTimers[t];
        if (!entry->active)
            continue;
        tic_entry_start(self, entry, true);
    }
    for (i = 0; i < _MAX_NICS; i++)
    {
        EnableEtherTube(&self->m_EthernetFilter[i], 1);
    }
    self->m_bIsStarted = true;

    MTAL_DP("CManager::start()\n");

    return true;
}

//////////////////////////////////////////////////////////////////////////////////
bool stop(struct TManager* self)
{
    MTAL_DP("entering CManager::stop..\n");
    if(self->m_bIORunning)
    {
        /* Global stop: mute and stop every chip, not just chip 0. */
        uint32_t pcm_i, pcm_count = smp_load_acquire(&self->m_uPCMCount);
        for (pcm_i = 0; pcm_i < pcm_count; ++pcm_i)
        {
            void *chip = smp_load_acquire(&self->m_apALSAChip[pcm_i]);
            if (!chip)
                continue;
            stopIO(self, chip, false);
            stopIO(self, chip, true);
        }
    }

    int i;
    for (i = 0; i < _MAX_NICS; i++)
    {
        EnableEtherTube(&self->m_EthernetFilter[i], 0);
    }

    {
        unsigned int t;
        for (t = 0; t < MAX_TIC_ENTRIES; t++)
        {
            struct tic_timer_entry* entry = &self->m_TicTimers[t];
            if (!entry->active)
                continue;
            tic_entry_stop(self, entry);
        }
    }

    self->m_bIsStarted = false;
    MTAL_DP("leaving CManager::stop..\n");
    return true;
}

//////////////////////////////////////////////////////////////////////////////////
/*
 * Stage 1 multi-PCM: the global m_bIsPlaybackIO/m_bIsRecordingIO flags now
 * mean "at least one chip is doing playback/capture IO right now". They are
 * recomputed from the per-chip flags (set by start_interrupts/stop_interrupts
 * before startIO/stopIO run) instead of being toggled unconditionally — so a
 * stopIO call from chip 0 cannot stop the world while chip 1 is still active.
 */
static void recompute_global_io_flags(struct TManager* self)
{
    bool any_play = false;
    bool any_cap  = false;
    uint32_t i, count;
    unsigned long lock_flags = 0;
    bool locked = false;
    const struct ravenna_mgr_ops *frontend;

    /* 2026-06-09 review fix (lost-update race): ALSA triggers on different
     * chips run concurrently on different CPUs. Each caller updates its
     * chip's io state BEFORE calling us, so serializing the recompute
     * guarantees the LAST recompute under the lock reads every chip's
     * current flags — a stale m_bIORunning=false can no longer overwrite
     * a newer true (which silenced the TIC for all chips), nor vice versa.
     * irqsave: triggers may run with IRQs already disabled. */
    if (self->m_csIOState)
    {
        spin_lock_irqsave((spinlock_t*)self->m_csIOState, lock_flags);
        locked = true;
    }

    /* Acquire-load the frontend pointer first so the function-pointer
     * struct is fully visible before we dereference it through the chips
     * loop. Pairs with smp_store_release in attach_alsa_driver. */
    frontend = smp_load_acquire(&self->m_alsa_driver_frontend);
    if (frontend)
    {
        count = smp_load_acquire(&self->m_uPCMCount);
        for (i = 0; i < count; ++i)
        {
            void *chip = smp_load_acquire(&self->m_apALSAChip[i]);
            if (!chip)
                continue;
            if (frontend->get_io_state(chip, true))
                any_play = true;
            if (frontend->get_io_state(chip, false))
                any_cap = true;
        }
    }
    self->m_bIsPlaybackIO = any_play;
    self->m_bIsRecordingIO = any_cap;
    self->m_bIORunning = any_play || any_cap;

    if (locked)
        spin_unlock_irqrestore((spinlock_t*)self->m_csIOState, lock_flags);
}

bool startIO(struct TManager* self, void* alsa_chip_pointer, bool is_playback)
{
    if(!self->m_bIsStarted)
        return false;

    MTAL_DP("MergingRAVENNAAudioDriver::startIO\n");

    /* 2026-06-09 review fix (cross-chip mute wipe): mute only the chip
     * whose trigger is starting — previously this always muted chip 0's
     * rings, so starting PCM 1 wiped PCM 0's live audio. */
    if (!is_playback) {
        printk(KERN_DEBUG "starting capture I/O\n");
        MuteInputBuffer(self, alsa_chip_pointer);
    }
    else {
        printk(KERN_DEBUG "starting playback I/O\n");
        MuteOutputBuffer(self, alsa_chip_pointer);
    }

    #if defined(MT_TONE_TEST)
    self->m_tone_test_phase = 0;
    #elif defined(MT_RAMP_TEST)
    self->m_ramp_test_phase = -8388608; // -2^23
    #endif // MT_TONE_TEST

    // NAD-351: must be done after mute
    recompute_global_io_flags(self);

    return true;
}

//////////////////////////////////////////////////////////////////////////////////
bool stopIO(struct TManager* self, void* alsa_chip_pointer, bool is_playback)
{
    MTAL_DP("MergingRAVENNAAudioDriver::stopIO\n");

    /* 2026-06-09 review fix (cross-chip mute wipe): mute only the chip
     * whose trigger is stopping. Each chip now gets its own buffers muted
     * on its own stop (previously chips 1..N-1 were never muted and chip 0
     * was wiped by everyone). */
    if (!is_playback) {
        printk(KERN_DEBUG "stopping capture I/O\n");
        MuteInputBuffer(self, alsa_chip_pointer);
    } else {
        printk(KERN_DEBUG "stopping playback I/O\n");
        MuteOutputBuffer(self, alsa_chip_pointer);
    }

    recompute_global_io_flags(self);

    return true;
}

//////////////////////////////////////////////////////////////////////////////////
/*
 * Multi-rate Stage 2: compute frame_size (samples per TIC tick) for a
 * given sample rate. Extracted from UpdateFrameSize so per-chip code
 * paths (attach_alsa_driver, MT_ALSA_Msg_AddPCM handler, SetSamplingRate's
 * chip-0 propagation) can call it without modifying manager-wide state.
 *
 * The math here is the same as the original UpdateFrameSize: nFS scales
 * by power-of-two as the rate climbs the AES67 ladder, frame_size =
 * tic_frame_size_at_1fs * nFS, capped at max_frame_size.
 */
uint32_t compute_frame_size_for_rate(uint32_t sample_rate, uint64_t tic_frame_size_at_1fs, uint32_t max_frame_size)
{
    uint32_t ui32nFS;
    uint32_t frame_size;
    if(IsDSDRate(sample_rate))
    {
        ui32nFS = 8;
    }
    else
    {
        switch(sample_rate)
        {
            case 384000:
            case 352800:
                ui32nFS = 8;
                break;
            case 192000:
            case 176400:
                ui32nFS = 4;
                break;
            case 96000:
            case 88200:
                ui32nFS = 2;
                break;
            case 48000:
            case 44100:
            default:
                ui32nFS = 1;
                break;
        }
    }
    frame_size = (uint32_t)tic_frame_size_at_1fs * ui32nFS;
    if(frame_size > max_frame_size)
        frame_size = max_frame_size;
    return frame_size;
}

bool is_valid_pcm_rate(uint32_t sample_rate)
{
    switch (sample_rate)
    {
        case 44100:
        case 48000:
        case 88200:
        case 96000:
        case 176400:
        case 192000:
        case 352800:
        case 384000:
            return true;
        default:
            return false;
    }
}

void UpdateFrameSize(struct TManager* self)
{
    self->m_ui32FrameSize = compute_frame_size_for_rate(
        self->m_SampleRate,
        self->m_TICFrameSizeAt1FS,
        self->m_MaxFrameSize);

    MTAL_DP("CManager::UpdateFrameSize() new TIC Frame Size = %u\n", self->m_ui32FrameSize);

    /* W14: no chip-0 special case — every chip's rate is locked at AddPCM and
     * never follows a manager-wide rate. Refresh EVERY active entry's frame
     * size from its OWN tick rate, because m_TICFrameSizeAt1FS / m_MaxFrameSize
     * (which parameterize all cadences) may be what changed here. Engines pick
     * the new values up via tic_entry_start; callers guarantee IO is stopped.
     * (A DSD entry keys on 352800, which compute_frame_size_for_rate maps to
     * the same nFS=8 frame as the DSD bit rate.) */
    {
        unsigned int t;
        for (t = 0; t < MAX_TIC_ENTRIES; t++)
        {
            struct tic_timer_entry* entry = &self->m_TicTimers[t];
            if (!entry->active)
                continue;
            entry->frame_size = compute_frame_size_for_rate(
                entry->tick_rate,
                self->m_TICFrameSizeAt1FS,
                self->m_MaxFrameSize);
            tic_entry_refresh_base_period(entry);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
// the caller must call stop before calling
bool SetInterfaceName(struct TManager* self, const char* cInterfaceName, const int iEthFilterIndex)
{
    if (iEthFilterIndex > _MAX_NICS)
    {
        MTAL_DP("SetInterfaceName: Ethernet filter index out of range\n");
        return false;
    }

    if(!Stop(&self->m_EthernetFilter[iEthFilterIndex]))
    {
        MTAL_DP("SetInterfaceName: self->m_EthernetFilter[%d].Stop() failed\n", iEthFilterIndex);
        return false;
    }

    //MTAL_DP("SetInterfaceName(%s)\n", cInterfaceName);
    strscpy(self->m_cInterfaceName, cInterfaceName, MAX_INTERFACE_NAME);
    if(strlen(self->m_cInterfaceName) != strlen(cInterfaceName))
    {
        MTAL_DP("SetInterfaceName: Interface name too long\n");
        return false;
    }

    if(!Start(&self->m_EthernetFilter[iEthFilterIndex], self->m_cInterfaceName))
    {
        MTAL_DP("SetInterfaceName: self->m_EthernetFilter[%d].Attach() failed\n", iEthFilterIndex);
        return false;
    }

    self->m_Is_NIC_Active[iEthFilterIndex] = true;

    return true;
}

//////////////////////////////////////////////////////////////////////////////////
bool SetSamplingRate(struct TManager* self, uint32_t samplingRate)
{
    uint64_t nbloop = 0;
    //MTAL_DP("CManager::SetSamplingRate from %u to %u\n", self->m_SampleRate, samplingRate);

    if(self->m_SampleRate == samplingRate)
        return true;

    if(self->m_bIORunning)
    {
        MTAL_DP("CManager::SetSamplingRate(%u) not allowed when IO are running\n", samplingRate);
        return false; // not allowed. stop IO first
    }

    /* W5 step 3: chip 0's entry re-keys to the new rate in
     * UpdateFrameSize — refuse if another PCM group's entry already owns
     * that tick cadence (registry keys must stay unique: two timers at
     * one cadence would double-pump that rate's streams). W7's per-PCM
     * rate plumbing supersedes this global-rate path. */
    {
        uint32_t new_tick_rate = tick_rate_for_sample_rate(samplingRate);
        struct tic_timer_entry* entry0 = smp_load_acquire(&self->m_apChipEntry[0]);
        unsigned int t;
        /* 2026-06-11 review fix (shared-entry re-key): chip 0's entry is
         * shared by every same-rate chip; re-keying it in place would drag
         * their clocks to the new cadence while their published rates and
         * streams stay put — and a later AddPCM at the old rate would
         * split the registry and wedge this path permanently. Re-rating a
         * shared cadence is W10 SetPCMRate's job. */
        if (entry0 && entry0->chip_refcount > 1 && entry0->tick_rate != new_tick_rate)
        {
            MTAL_DP_ERR("CManager::SetSamplingRate(%u): refused — chip 0's (domain, rate) entry is shared by %u chips\n",
                        samplingRate, entry0->chip_refcount);
            return false;
        }
        for (t = 0; t < MAX_TIC_ENTRIES; t++)
        {
            struct tic_timer_entry* e = &self->m_TicTimers[t];
            if (e->active && e != entry0 && e->domain == 0 && e->tick_rate == new_tick_rate)
            {
                MTAL_DP_ERR("CManager::SetSamplingRate(%u): tick rate %u already owned by another PCM group's entry\n", samplingRate, new_tick_rate);
                return false;
            }
        }
    }


    self->m_SampleRate = samplingRate;
    UpdateFrameSize(self);
    /* Manager-wide rate propagates to chip 0 only (W5/W7 make rates
     * per-chip); mute chip 0's ring accordingly. */
    MuteOutputBuffer(self, smp_load_acquire(&self->m_apALSAChip[0]));

    if(self->m_bIsStarted)
    {
        struct tic_timer_entry* entry = smp_load_acquire(&self->m_apChipEntry[0]);
        if (entry)
            tic_entry_start(self, entry, true); /* re-key engines + relock (legacy semantics) */
        do
        {
            CW_msleep_interruptible(1);
            if(++nbloop >= 4000)
            {
                MTAL_DP("CManager::SetSamplingRate PTP lock timed out\n");
                return false;
            }
        }
        /* W11: wait on chip 0's domain servos (legacy global-rate path). Already
         * bounded by the nbloop cap above — a GM-less domain times out, not hangs. */
        while (GetLockStatus(&self->m_PTP[0][entry ? entry->domain : 0]) != PTPLS_LOCKED ||
               GetLockStatus(&self->m_PTP[1][entry ? entry->domain : 0]) != PTPLS_LOCKED);
        //MTAL_DP("CManager::SetSamplingRate(%u) Completed\n", samplingRate);
    }

    return true;
}

//////////////////////////////////////////////////////////////////////////////////
bool SetDSDSamplingRate(struct TManager* self, uint32_t samplingRate)
{
    uint64_t nbloop = 0;
    MTAL_DP("CManager::SetDSDSamplingRate(%u)\n", samplingRate);

    if(self->m_SampleRate == samplingRate)
        return true;

    if(self->m_bIORunning)
    {
        MTAL_DP("CManager::SetDSDSamplingRate(%u) not allowed when IO are running\n", samplingRate);
        return false; // not allowed. stop IO first
    }

    /* W5 step 3: chip 0's entry re-keys to the new rate in
     * UpdateFrameSize — refuse if another PCM group's entry already owns
     * that tick cadence (registry keys must stay unique: two timers at
     * one cadence would double-pump that rate's streams). W7's per-PCM
     * rate plumbing supersedes this global-rate path. */
    {
        uint32_t new_tick_rate = tick_rate_for_sample_rate(samplingRate);
        struct tic_timer_entry* entry0 = smp_load_acquire(&self->m_apChipEntry[0]);
        unsigned int t;
        /* 2026-06-11 review fix (shared-entry re-key): chip 0's entry is
         * shared by every same-rate chip; re-keying it in place would drag
         * their clocks to the new cadence while their published rates and
         * streams stay put — and a later AddPCM at the old rate would
         * split the registry and wedge this path permanently. Re-rating a
         * shared cadence is W10 SetPCMRate's job. */
        if (entry0 && entry0->chip_refcount > 1 && entry0->tick_rate != new_tick_rate)
        {
            MTAL_DP_ERR("CManager::SetSamplingRate(%u): refused — chip 0's (domain, rate) entry is shared by %u chips\n",
                        samplingRate, entry0->chip_refcount);
            return false;
        }
        for (t = 0; t < MAX_TIC_ENTRIES; t++)
        {
            struct tic_timer_entry* e = &self->m_TicTimers[t];
            if (e->active && e != entry0 && e->domain == 0 && e->tick_rate == new_tick_rate)
            {
                MTAL_DP_ERR("CManager::SetSamplingRate(%u): tick rate %u already owned by another PCM group's entry\n", samplingRate, new_tick_rate);
                return false;
            }
        }
    }


    self->m_SampleRate = samplingRate;
    UpdateFrameSize(self);
    /* Manager-wide rate propagates to chip 0 only (W5/W7 make rates
     * per-chip); mute chip 0's ring accordingly. */
    MuteOutputBuffer(self, smp_load_acquire(&self->m_apALSAChip[0]));

    if(self->m_bIsStarted)
    {
        struct tic_timer_entry* entry = smp_load_acquire(&self->m_apChipEntry[0]);
        if (entry)
            tic_entry_start(self, entry, true); /* re-key engines + relock (legacy semantics) */
        do
        {
            CW_msleep_interruptible(1);
            if(++nbloop >= 4000)
            {
                MTAL_DP("CManager::SetSamplingRate PTP lock timed out\n");
                return false;
            }
        }
        /* W11: wait on chip 0's domain servos; bounded by the nbloop cap above. */
        while (GetLockStatus(&self->m_PTP[0][entry ? entry->domain : 0]) != PTPLS_LOCKED ||
               GetLockStatus(&self->m_PTP[1][entry ? entry->domain : 0]) != PTPLS_LOCKED);
        //MTAL_DP("\n>>> CManager::SetSamplingRate completed () (self->m_PTP.GetLockStatus() == PTPLS_LOCKED)\n\n");
    }
    return true;
}

//////////////////////////////////////////////////////////////////////////////////
bool SetTICFrameSizeAt1FS(struct TManager* self, uint64_t TICFrameSize)
{
    bool bRestart = self->m_bIsStarted;

    MTAL_DP("CManager::SetTICFrameSizeAt1FS(%llu)\n", TICFrameSize);

    if(bRestart)
        stop(self);
    self->m_TICFrameSizeAt1FS = TICFrameSize;
    UpdateFrameSize(self);

    if(bRestart)
        start(self);
    return true;
}
//////////////////////////////////////////////////////////////////////////////////
bool SetMaxTICFrameSize(struct TManager* self, uint64_t max_frameSize)
{
    bool bRestart = self->m_bIsStarted;

    MTAL_DP("CManager::SetMAXTICFrameSize(%llu)\n", max_frameSize);

    if(bRestart)
        stop(self);
    self->m_MaxFrameSize = (uint32_t)(max_frameSize);
    UpdateFrameSize(self);

    if(bRestart)
        start(self);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////
bool SetNumberOfInputs(struct TManager* self, uint32_t NumberOfChannels)
{
    bool bRestart = self->m_bIsStarted;

    MTAL_DP("CManager::SetNumberOfInputs(%u)\n", NumberOfChannels);

    if(bRestart)
        stop(self);
    self->m_NumberOfInputs = NumberOfChannels;
    if(bRestart)
        start(self);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////
bool SetNumberOfOutputs(struct TManager* self, uint32_t NumberOfChannels)
{
    bool bRestart = self->m_bIsStarted;

    MTAL_DP("CManager::SetNumberOfOutputs(%u)\n", NumberOfChannels);

    if(bRestart)
        stop(self);
    self->m_NumberOfOutputs = NumberOfChannels;
    if(bRestart)
        start(self);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////
// W5: the (domain, tick_rate) timer registry
//////////////////////////////////////////////////////////////////////////////////

/* DSD chips publish their bit rate (e.g. 2,822,400) but tick in the
 * 352.8k clock domain — the registry keys on the TICK rate (the W5 DSD
 * unit fix; pre-W5 the SAC derivation wrongly scaled DSD by 8). */
static uint32_t tick_rate_for_sample_rate(uint32_t sample_rate)
{
    return IsDSDRate(sample_rate) ? 352800 : sample_rate;
}

static void tic_entry_refresh_base_period(struct tic_timer_entry* entry)
{
    /* W5 + upstream 367c166: frame*1e9/rate, via the shared per-ct helper. */
    update_base_period(&entry->timer, entry->frame_size, entry->tick_rate);
}

/* Start (or re-key) an entry's engines and arm its hrtimer.
 * bResetPTPLock=true reproduces the legacy StartAudioFrameTICTimer
 * semantics (manager start / global rate change: rate or frame size
 * changed, so computation made during PTP locking is no longer valid).
 * bResetPTPLock=false is the add-a-rate-to-a-live-domain path (W5
 * decoupling): the servo's PTP lock — and therefore every other running
 * engine on this domain — is left untouched; if the servo is already
 * locked the engine phase-inits directly from the live offset. */
static void tic_entry_start(struct TManager* self, struct tic_timer_entry* entry, bool bResetPTPLock)
{
    int i;
    for (i = 0; i < _MAX_NICS; i++)
    {
        TClock_PTP* pServo = &self->m_PTP[i][entry->domain];
        if (!self->m_Is_NIC_Active[i])
            continue;
        spin_lock_bh((spinlock_t*)pServo->m_csPTPTime);
        tic_engine_start(&entry->engine[i], entry->frame_size, entry->tick_rate);
        if (!bResetPTPLock)
        {
            /* 2026-06-11 review fix: ALWAYS arm the TIC-lock hysteresis on
             * the no-reset path — an engine started mid-PTP-acquisition
             * previously kept its zeroed counter and reported LOCKED with
             * zero convergence syncs. Phase-init (which arms it too)
             * additionally aligns the tick when the servo is already
             * locked; otherwise the prelock fan-out will. */
            if (pServo->m_usPTPLockCounter == 0)
                tic_engine_phase_init_from_locked(&entry->engine[i]);
            else
                tic_engine_reset(&entry->engine[i]);
        }
        spin_unlock_bh((spinlock_t*)pServo->m_csPTPTime);
        if (bResetPTPLock)
            ResetPTPLock(pServo, true);
    }
    tic_entry_refresh_base_period(entry);
    start_clock_timer(&entry->timer);
}

/* Stop an entry: cancel its hrtimer, stop its engines. The ResetPTPLock
 * mirrors the legacy StopAudioFrameTICTimer (note it resets ALL of the
 * servo's engines — acceptable because today's only callers are the
 * global stop/teardown paths; W10's selective RemovePCM quiesce will want
 * a gentler variant). */
static void tic_entry_stop(struct TManager* self, struct tic_timer_entry* entry)
{
    int i;
    stop_clock_timer(&entry->timer);
    for (i = 0; i < _MAX_NICS; i++)
    {
        tic_engine_stop(&entry->engine[i]);
        ResetPTPLock(&self->m_PTP[i][entry->domain], true);
    }
}

/* Find-or-create with chip refcounting. Netlink context only. */
static struct tic_timer_entry* get_or_create_tic_entry(struct TManager* self, uint8_t domain, uint32_t sample_rate)
{
    uint32_t tick_rate = tick_rate_for_sample_rate(sample_rate);
    struct tic_timer_entry* entry = NULL;
    unsigned int t, nic;

    for (t = 0; t < MAX_TIC_ENTRIES; t++)
    {
        struct tic_timer_entry* e = &self->m_TicTimers[t];
        if (e->active && e->domain == domain && e->tick_rate == tick_rate)
        {
            e->chip_refcount++;
            return e;
        }
    }

    /* Concurrent entries at distinct (domain, tick_rate) cadences are
     * fully supported as of step 3: the pump is rate-filtered (chip-map
     * identity + stream_on_tick), and a new entry created onto a running
     * domain phase-inits from the live offset without touching the
     * domain's PTP lock (running entries never glitch). */
    for (t = 0; t < MAX_TIC_ENTRIES; t++)
    {
        if (!self->m_TicTimers[t].active)
        {
            entry = &self->m_TicTimers[t];
            break;
        }
    }
    if (!entry)
        return NULL;

    entry->mgr = self;
    entry->domain = domain;
    entry->tick_rate = tick_rate;
    entry->frame_size = compute_frame_size_for_rate(sample_rate, self->m_TICFrameSizeAt1FS, self->m_MaxFrameSize);
    entry->chip_refcount = 1;
    entry->active_nic = 0;
    for (nic = 0; nic < _MAX_NICS; nic++)
    {
        /* W11: bind this entry's engine to its domain's servo on each NIC. */
        tic_engine_init(&entry->engine[nic], &self->m_PTP[nic][domain]);
        /* 2026-06-11 review fix: attach can fail (engine list full once
         * W11 multiplies the keyspace) — unwind, don't half-create. */
        if (!clock_ptp_attach_engine(&self->m_PTP[nic][domain], &entry->engine[nic]))
        {
            unsigned int u;
            tic_engine_destroy(&entry->engine[nic]);
            for (u = 0; u < nic; u++)
            {
                clock_ptp_detach_engine(&self->m_PTP[u][domain], &entry->engine[u]);
                tic_engine_destroy(&entry->engine[u]);
            }
            return NULL;
        }
    }
    /* review hardening: init_clock_timer can now reject a bad
     * audio_cpu_affinity module param (-EINVAL, 367c166's CPU-pin). Unwind
     * the attached engines and fail rather than arm an unvalidated timer. */
    if (init_clock_timer(&entry->timer, entry) != 0)
    {
        unsigned int u;
        for (u = 0; u < _MAX_NICS; u++)
        {
            clock_ptp_detach_engine(&self->m_PTP[u][domain], &entry->engine[u]);
            tic_engine_destroy(&entry->engine[u]);
        }
        return NULL;
    }
    tic_entry_refresh_base_period(entry);
    /* publish before any chip maps to it */
    smp_store_release(&entry->active, true);

    /* Created onto a running manager (AddPCM at a new rate): start it
     * WITHOUT resetting the domain's PTP lock — running engines on other
     * entries must not glitch. (Unreachable until step 3 lifts the gate
     * above; chip 0's entry is created before the manager starts.) */
    if (self->m_bIsStarted)
        tic_entry_start(self, entry, false);
    return entry;
}

/* Drop a chip's reference; on the last one, quiesce and free the entry.
 * Netlink/teardown context. NOTE: an RTP/tick-path reader that resolved
 * this entry just before the final put can briefly hold a pointer to it —
 * same exposure class as the known unload-with-streams UAF (W10 hardens
 * the lifecycle; until then the W4 hygiene rule "remove streams before
 * unload" covers the only caller paths). */
static void put_tic_entry(struct TManager* self, struct tic_timer_entry* entry)
{
    int i;
    if (!entry || entry->chip_refcount == 0)
        return;
    if (--entry->chip_refcount > 0)
        return;
    stop_clock_timer(&entry->timer);
    for (i = 0; i < _MAX_NICS; i++)
    {
        tic_engine_stop(&entry->engine[i]);
        clock_ptp_detach_engine(&self->m_PTP[i][entry->domain], &entry->engine[i]);
        tic_engine_destroy(&entry->engine[i]);
    }
    smp_store_release(&entry->active, false);
}

/* W15: re-key an IDLE chip's (domain,rate) timer entry to new_rate in place —
 * the in-place alternative to recreate-card. Move the chip from its current
 * entry to the (domain, new_rate) entry (creating it if first, freeing the old
 * if last), republishing the chip's (rate, frame_size). Bound into alsa_ops as
 * set_pcm_rate and called from pcm_close when an armed chip goes idle (and from
 * the SetPCMRate handler when the chip is already idle). `user` is the manager
 * (== ravenna_peer, as for detach_alsa_driver). */
static int manager_set_pcm_rate(void* user, int32_t pcm_id, uint32_t new_rate)
{
    struct TManager* self = (struct TManager*)user;
    struct tic_timer_entry *old_entry, *new_entry;
    void* chip;
    uint8_t domain;
    uint32_t fsize;
    int ret = 0;
    if (!self || pcm_id < 0 || pcm_id >= MAX_PCMS)
        return -EINVAL;

    mutex_lock(&g_registry_lock);
    chip      = smp_load_acquire(&self->m_apALSAChip[pcm_id]);
    old_entry = smp_load_acquire(&self->m_apChipEntry[pcm_id]);
    if (!chip || !old_entry)
    {
        /* chip detached underneath us (e.g. card teardown raced the close) —
         * nothing to re-rate. */
        ret = -ENODEV;
        goto out;
    }
    /* keep the chip on its own PTP domain; only the rate changes. */
    domain = old_entry->domain;
    new_entry = get_or_create_tic_entry(self, domain, new_rate);
    if (!new_entry)
    {
        MTAL_DP_ERR("manager_set_pcm_rate: pcm_id %d no (domain %u, rate %u) entry\n",
                    pcm_id, domain, new_rate);
        ret = -EINVAL;
        goto out;
    }
    /* publish the chip's new (rate, frame_size) — this also disarms the latch
     * (pending_rate == new_rate) and notifies the PCM Rate control. */
    fsize = compute_frame_size_for_rate(new_rate, self->m_TICFrameSizeAt1FS, self->m_MaxFrameSize);
    if (self->m_alsa_driver_frontend && self->m_alsa_driver_frontend->set_pcm_sample_rate)
        self->m_alsa_driver_frontend->set_pcm_sample_rate(chip, new_rate, fsize);
    /* swap the chip's entry pointer, then drop the old entry's ref (frees +
     * stops it if this was its last chip). For a same-tick-rate re-rate (DSD
     * family) new_entry == old_entry: get_or_create bumped the refcount and this
     * put restores it — net no-op, chip rate still updated. */
    smp_store_release(&self->m_apChipEntry[pcm_id], new_entry);
    put_tic_entry(self, old_entry);
    MTAL_DP_INFO("manager_set_pcm_rate: pcm_id %d re-rated in place to %u (domain %u)\n",
                 pcm_id, new_rate, domain);
out:
    mutex_unlock(&g_registry_lock);
    return ret;
}

/* W15: pcm_open barrier — take+release the registry lock so an open cannot run
 * concurrently with (and observe a half-applied) re-key from another
 * substream's last close. */
static void manager_registry_barrier(void* user)
{
    (void)user;
    mutex_lock(&g_registry_lock);
    mutex_unlock(&g_registry_lock);
}

/* Per-entry NIC selection — today's preference rule (priority-preferred
 * NIC 0, else NIC 1, else default 0) with per-entry eligibility:
 * eligible = servo PTP-locked AND this entry's engine on that NIC
 * TIC-locked (tic_engine_lock_status is exactly that composite). Selection
 * eligibility and the pump gate are the same predicate by construction.
 * Single-writer: only this entry's timer callback calls this. */
static void tic_entry_select_nic(struct TManager* self, struct tic_timer_entry* entry)
{
    bool elig0 = tic_engine_lock_status(&entry->engine[0]) == PTPLS_LOCKED;
    bool elig1 = tic_engine_lock_status(&entry->engine[1]) == PTPLS_LOCKED;

    if (elig0 && GetPTPPriority(&self->m_PTP[0][entry->domain]) <= GetPTPPriority(&self->m_PTP[1][entry->domain]))
    {
        WRITE_ONCE(entry->active_nic, 0);
    }
    else if (elig1)
    {
        WRITE_ONCE(entry->active_nic, 1);
    }
    else
    {
        WRITE_ONCE(entry->active_nic, 0);
    }
}

/* The per-(domain, rate) tick: today's t_clock_timer, parameterized by
 * entry. Order preserved from the legacy callback: selection first (on
 * lock state as of the previous tick), both engines advance, servo
 * checks, both engines schedule, hrtimer follows the active engine, the
 * standby engine is rephased to it (ST2022-7 slaving, per rate), then the
 * audio pump for this entry. */
void manager_entry_tick(struct tic_timer_entry* entry, uint64_t* pui64NextRTXClockTime, uint64_t ui64Now)
{
    struct TManager* self = entry->mgr;
    TTicEngineTickCtx ctx[_MAX_NICS];
    uint64_t cand[_MAX_NICS];
    unsigned short nic, active, standby;

    tic_entry_select_nic(self, entry);

    for (nic = 0; nic < _MAX_NICS; nic++)
        tic_engine_tick_advance(&entry->engine[nic], &ctx[nic]);

    for (nic = 0; nic < _MAX_NICS; nic++)
    {
        if (ctx[nic].bStarted)
            clock_ptp_periodic_checks(&self->m_PTP[nic][entry->domain], ctx[nic].ui64CurrentRTXClockTime);
    }

    for (nic = 0; nic < _MAX_NICS; nic++)
    {
        /* Default one base period out: schedule leaves the candidate
         * untouched for a not-started engine, and a zero/now value would
         * busy-spin the hrtimer callback if selection defaults to a NIC
         * whose engine isn't ticking (e.g. NIC-1-only configs before
         * lock). */
        cand[nic] = ui64Now + READ_ONCE(entry->timer.base_period_);
        tic_engine_tick_schedule(&entry->engine[nic], &ctx[nic], &cand[nic]);
    }

    active = entry->active_nic; /* plain read: we are the only writer */
    standby = (unsigned short)(active == 0 ? 1 : 0);

    *pui64NextRTXClockTime = cand[active];
    tic_engine_set_next_abs_time(&entry->engine[standby], cand[active]);

    manager_audio_frame_tic(self, entry);
}

/* Forward decl (defined with the per-PCM tick-path helpers below). */
static void* resolve_chip(struct TManager* self, uint32_t pcm_id, const struct ravenna_mgr_ops **out_frontend);

static TTicEngine* active_engine_of(struct tic_timer_entry* entry)
{
    return &entry->engine[READ_ONCE(entry->active_nic)];
}

//////////////////////////////////////////////////////////////////////////////////
/*
 * W5: the per-chip clock handle — chip -> (domain, tick_rate) entry ->
 * that entry's active-NIC engine. The SINGLE chokepoint that maps a
 * pcm_id to the media clock that anchors it (Decision 9: W11 makes the
 * entry lookup domain-aware here and nowhere else).
 *
 * Safe-fail mirrors the legacy get_clock_for_pcm: an unmapped pcm_id
 * resolves to chip 0's entry (the legacy manager-wide clock).
 */
static TTicEngine* get_engine_for_pcm(struct TManager* self, uint32_t pcm_id)
{
    struct tic_timer_entry* entry = NULL;
    if (pcm_id < MAX_PCMS)
        entry = smp_load_acquire(&self->m_apChipEntry[pcm_id]);
    if (!entry)
        entry = smp_load_acquire(&self->m_apChipEntry[0]);
    if (!entry)
        return NULL;
    return active_engine_of(entry);
}

/* Daemon-visible PTP status routes via chip 0's entry (W5 decision 5). */
static unsigned short manager_status_nic(struct TManager* self)
{
    struct tic_timer_entry* entry = smp_load_acquire(&self->m_apChipEntry[0]);
    return entry ? READ_ONCE(entry->active_nic) : 0;
}

//////////////////////////////////////////////////////////////////////////////////
bool IsStarted(struct TManager* self)
{
    return self->m_bIsStarted;
}

//////////////////////////////////////////////////////////////////////////////////
bool IsIOStarted(struct TManager* self)
{
    return self->m_bIORunning;
}

//////////////////////////////////////////////////////////////////////////////////
int EtherTubeRxPacket(struct TManager* self, void* packet, int packet_size, const char* ifname, int mac_header)
{
    int i = 0;
    int ret = 1;
    for (i = 0; i < _MAX_NICS; i++)
    {
        ret &= rx_packet(&self->m_EthernetFilter[i], packet, packet_size, ifname, mac_header);
    }
    return ret;
}

//////////////////////////////////////////////////////////////////////////////////
void EtherTubeHookFct(struct TManager* self, void* hook_fct, void* hook_struct)
{
    int i = 0;
    for (i = 0; i < _MAX_NICS; i++)
    {
        netfilter_hook_fct(&self->m_EthernetFilter[i], hook_fct, hook_struct);
    }
}

//////////////////////////////////////////////////////////////////////////////////
// Messaging with userland (use netlink)
void OnNewMessage(struct TManager* self, struct MT_ALSA_msg* msg_rcv)
{
    int i = 0;
    uint32_t ravenna_rate = IsDSDRate(self->m_SampleRate)? 352800 : self->m_SampleRate;
    uint32_t ravenna_audiomode = GetAudioModeFromRate(self->m_SampleRate);

    struct MT_ALSA_msg msg_reply;

    msg_reply.id = msg_rcv->id;
    msg_reply.errCode = -404;
    msg_reply.dataSize = 0;
    msg_reply.data = NULL;

    if (msg_rcv == NULL)
        return;

    switch (msg_rcv->id)
    {
        case MT_ALSA_Msg_GetRTPStreamStatus:
        {
            if (msg_rcv->dataSize != sizeof(uint64_t))
            {
                MTAL_DP_ERR("Get stream status invalid data size\n");
                msg_reply.errCode = -315;
            }
            else
            {
                TRTP_stream_status stream_status;
                uint64_t* rtp_stream_handle_ptr = (uint64_t*)msg_rcv->data;
                if (!get_RTPStream_status_(&self->m_RTP_streams_manager, *rtp_stream_handle_ptr, &stream_status))
                    msg_reply.errCode = -401;
                else
                    msg_reply.errCode = 0;
                
                msg_reply.errCode = 0;
                msg_reply.dataSize = sizeof(TRTP_stream_status);
                msg_reply.data = &stream_status;

                CW_netlink_send_reply_to_user_land(&msg_reply);
                return; // because stream_status is out of the scope if send reply at the end of the function
            }
            break;
        }
        case MT_ALSA_Msg_Reset:
        {
            /* Payload: int32_t pcm_id. W9 convention: pcm_id < 0 is the
             * init-time clean slate the daemon sends at startup — drains ALL
             * streams AND tears down ALL cards (W10), so a restart redeclares
             * its cards onto an empty module instead of colliding with the
             * previous session's cards (AddCard -EEXIST). pcm_id in
             * [0, MAX_PCMS) drains only that PCM's streams via the W8 per-PCM
             * index, leaving every other PCM's streams and all cards intact. */
            MTAL_DP("CManager::OnNewMessage MT_ALSA_Msg_Reset..\n");
            if (msg_rcv->dataSize != sizeof(int32_t))
            {
                MTAL_DP_ERR("MT_ALSA_Msg_Reset invalid data size (got %d, expected %zu)\n",
                            msg_rcv->dataSize, sizeof(int32_t));
                msg_reply.errCode = -315;
            }
            else
            {
                int32_t pcm_id = *(int32_t*)msg_rcv->data;
                if (pcm_id < 0)
                {
                    /* Streams first, then cards, so no stream outlives the
                     * chip it referenced (teardown_card additionally detaches
                     * each chip from the manager before freeing it). */
                    remove_all_RTP_streams(&self->m_RTP_streams_manager);
                    mr_alsa_audio_remove_all_cards();
                    msg_reply.errCode = 0;
                }
                else if (pcm_id < MAX_PCMS)
                {
                    unsigned int removed = remove_RTP_streams_for_pcm(
                        &self->m_RTP_streams_manager, (uint32_t)pcm_id);
                    printk("W9 reset: drained %u stream(s) of pcm %d\n",
                           removed, pcm_id);
                    msg_reply.errCode = 0;
                }
                else
                {
                    MTAL_DP_ERR("MT_ALSA_Msg_Reset: pcm_id %d out of range [0,%d)\n",
                                pcm_id, MAX_PCMS);
                    msg_reply.errCode = -315;
                }
            }
            break;
        }
        case MT_ALSA_Msg_Start:
        {
            MTAL_DP("CManager::OnNewMessage MT_ALSA_Msg_Start..\n");
            if (!start(self))
            {
                MTAL_DP("CManager::OnNewMessage MT_ALSA_Msg_Start.. failed\n");
                msg_reply.errCode = -401;
            }
            else
            {
                MTAL_DP("CManager::OnNewMessage MT_ALSA_Msg_Start.. succeeded\n");
                msg_reply.errCode = 0;
            }
            break;
        }
        case MT_ALSA_Msg_Stop:
        {
            MTAL_DP("CManager::OnNewMessage MT_ALSA_Msg_Stop\n");
            if (!stop(self))
            {
                MTAL_DP("CManager::OnNewMessage MT_ALSA_Msg_Stop.. failed\n");
                msg_reply.errCode = -401;
            }
            else
            {
                MTAL_DP("CManager::OnNewMessage MT_ALSA_Msg_Stop.. succeeded\n");
                msg_reply.errCode = 0;
            }
            break;
        }
        case MT_ALSA_Msg_StartIO:
        {
            MTAL_DP("CManager::OnNewMessage MT_ALSA_Msg_StartIO..\n");
            /*
            if (!startIO(self) )
            {
                MTAL_DP("CManager::OnNewMessage MT_ALSA_Msg_StartIO.. failed\n");
                msg_reply.errCode = -401;
            }
            else
            {
                MTAL_DP("CManager::OnNewMessage MT_ALSA_Msg_StartIO.. succeeded\n");
                msg_reply.errCode = 0;
            }
            */
            msg_reply.errCode = -401;
            break;
        }
        case MT_ALSA_Msg_StopIO:
        {
            MTAL_DP("CManager::OnNewMessage MT_ALSA_Msg_StopIO..\n");
            /*
            if (!stopIO(self))
            {
                MTAL_DP("CManager::OnNewMessage MT_ALSA_Msg_StopIO.. failed\n");
                msg_reply.errCode = -401;
            }
            else
            {
                MTAL_DP("CManager::OnNewMessage MT_ALSA_Msg_StopIO.. succeeded\n");
                msg_reply.errCode = 0;
            }
            */
            msg_reply.errCode = -401;
            break;
        }
        case MT_ALSA_Msg_SetSampleRate:
        {
            //MTAL_DP(">>>> CManager::OnNewMessage MT_ALSA_Msg_SetSampleRate...\n");
            if (msg_rcv->dataSize != sizeof(unsigned int))
            {
                MTAL_DP_ERR("Set sampling rate invalid data size\n");
                msg_reply.errCode = -315;
            }
            else
            {
                unsigned int* samplingrate_ptr = (unsigned int*)msg_rcv->data;
                if(*samplingrate_ptr == 0)
                {
                    MTAL_DP_INFO("Set sampling rate invalid value (%p)\n", samplingrate_ptr);
                    msg_reply.errCode = -805;
                }
                //MTAL_DP_INFO(">>>> CManager::OnNewMessage: Set sampling rate to %u\n", *samplingrate_ptr);
                msg_reply.errCode = 0;
                SetSamplingRate(self, *samplingrate_ptr);
            }
            break;
        }
        case MT_ALSA_Msg_GetSampleRate:
        {
            //MTAL_DP(">>>> CManager::OnNewMessage MT_ALSA_Msg_GetSampleRate... return %u\n", self->m_SampleRate);
            msg_reply.errCode = 0;
            msg_reply.dataSize = sizeof(ravenna_rate);

            msg_reply.data = &ravenna_rate;
            break;
        }

        case MT_ALSA_Msg_GetAudioMode:
        {
            msg_reply.errCode = 0;
            msg_reply.dataSize = sizeof(ravenna_audiomode);
            msg_reply.data = &ravenna_audiomode;
            break;
        }

        case MT_ALSA_Msg_SetDSDAudioMode:
        {
            if (msg_rcv->dataSize != sizeof(unsigned int))
            {
                MTAL_DP_ERR("Set DSD Audio Mode invalid data size\n");
                msg_reply.errCode = -315;
            }
            else
            {
                unsigned int* audiomode_ptr = (unsigned int*)msg_rcv->data;
                uint32_t newDSDSamplingRate = 0;
                switch(*audiomode_ptr)
                {
                    case AM_DSD64:
                        newDSDSamplingRate = 2822400;
                        break;
                    case AM_DSD128:
                        newDSDSamplingRate = 5644800;
                        break;
                    case AM_DSD256:
                        newDSDSamplingRate = 11289600;
                        break;
                    default:
                        MTAL_DP_INFO("Set DSD Audio Mode invalid value (%u)\n", *audiomode_ptr);
                        msg_reply.errCode = -805;
                        break;
                }
                if(newDSDSamplingRate != 0)
                {
                    MTAL_DP_INFO(">>>> CManager::OnNewMessage: Set DSD sampling rate to %u\n", newDSDSamplingRate);
                    msg_reply.errCode = 0;
                    SetDSDSamplingRate(self, newDSDSamplingRate);
                }
            }
            break;
        }
        case MT_ALSA_Msg_SetTICFrameSizeAt1FS:
        {
            if (msg_rcv->dataSize != sizeof(uint64_t))
            {
                MTAL_DP_ERR("Set 1 FS TIC frame size invalid data size\n");
                msg_reply.errCode = -315;
            }
            else
            {
                uint64_t* framesize_ptr = (uint64_t*)msg_rcv->data;
                MTAL_DP_INFO("Set 1 FS TIC frame size to %llu\n", *framesize_ptr);
                if (!SetTICFrameSizeAt1FS(self, *framesize_ptr))
                    msg_reply.errCode = -401;
                else
                    msg_reply.errCode = 0;
            }
            break;
        }
        case MT_ALSA_Msg_SetMaxTICFrameSize:
        {
            if (msg_rcv->dataSize != sizeof(uint64_t))
            {
                MTAL_DP_ERR("Set Max TIC frame size invalid data size\n");
                msg_reply.errCode = -315;
            }
            else
            {
                uint64_t* framesize_ptr = (uint64_t*)msg_rcv->data;
                MTAL_DP_INFO("Set Max TIC frame size to %llu\n", *framesize_ptr);
                if (!SetMaxTICFrameSize(self, *framesize_ptr))
                    msg_reply.errCode = -401;
                else
                    msg_reply.errCode = 0;
            }
            break;
        }

        case MT_ALSA_Msg_SetNumberOfInputs:
            /* Payload (Stage 1+): {int32_t pcm_id, uint32_t count}. Channel
             * count is still applied manager-wide in Stage 1 (all PCMs share
             * the same count); pcm_id is parsed for protocol-compat with the
             * future per-PCM split in Stage 2. */
            if (msg_rcv->dataSize != sizeof(int32_t) + sizeof(uint32_t))
            {
                MTAL_DP_ERR("Set Nb Inputs invalid data size\n");
                msg_reply.errCode = -315;
            }
            else
            {
                int32_t pcm_id = *(int32_t*)msg_rcv->data;
                uint32_t nbch = *(uint32_t*)((char*)msg_rcv->data + sizeof(int32_t));
                MTAL_DP_INFO("Set Nb Inputs pcm_id=%d to %u\n", pcm_id, nbch);
                /* Stage 1: refuse non-zero pcm_id so the per-PCM split
                 * gap is visible at the wire level. Today we apply the
                 * value manager-wide; a daemon trying to set per-PCM
                 * channel counts will see -EINVAL and know it's hit the
                 * Stage 2 boundary, instead of silently getting all
                 * chips re-counted whenever the last writer wins. */
                if (pcm_id != 0)
                {
                    MTAL_DP_ERR("Set Nb Inputs: per-pcm_id (%d) not supported in Stage 1\n", pcm_id);
                    msg_reply.errCode = -EINVAL;
                }
                else if (!SetNumberOfInputs(self, nbch))
                    msg_reply.errCode = -401;
                else
                    msg_reply.errCode = 0;
            }
            break;
        case MT_ALSA_Msg_SetNumberOfOutputs:
            if (msg_rcv->dataSize != sizeof(int32_t) + sizeof(uint32_t))
            {
                MTAL_DP_ERR("Set Nb Outputs invalid data size\n");
                msg_reply.errCode = -315;
            }
            else
            {
                int32_t pcm_id = *(int32_t*)msg_rcv->data;
                uint32_t nbch = *(uint32_t*)((char*)msg_rcv->data + sizeof(int32_t));
                MTAL_DP_INFO("Set Nb Outputs pcm_id=%d to %u\n", pcm_id, nbch);
                if (pcm_id != 0)
                {
                    MTAL_DP_ERR("Set Nb Outputs: per-pcm_id (%d) not supported in Stage 1\n", pcm_id);
                    msg_reply.errCode = -EINVAL;
                }
                else if (!SetNumberOfOutputs(self, nbch))
                    msg_reply.errCode = -401;
                else
                    msg_reply.errCode = 0;
            }
            break;
        case MT_ALSA_Msg_GetNumberOfInputs:
        {
            /* Payload: int32_t pcm_id. Reply unchanged (uint32_t count). */
            if (msg_rcv->dataSize != sizeof(int32_t))
            {
                MTAL_DP_ERR("Get Nb Inputs invalid data size\n");
                msg_reply.errCode = -315;
            }
            else
            {
                msg_reply.errCode = 0;
                msg_reply.dataSize = sizeof(self->m_NumberOfInputs);
                msg_reply.data = &self->m_NumberOfInputs;
            }
            break;
        }
        case MT_ALSA_Msg_GetNumberOfOutputs:
        {
            if (msg_rcv->dataSize != sizeof(int32_t))
            {
                MTAL_DP_ERR("Get Nb Outputs invalid data size\n");
                msg_reply.errCode = -315;
            }
            else
            {
                msg_reply.errCode = 0;
                msg_reply.dataSize = sizeof(self->m_NumberOfOutputs);
                msg_reply.data = &self->m_NumberOfOutputs;
            }
            break;
        }
        case MT_ALSA_Msg_SetInterfaceName:
        {
            if (msg_rcv->dataSize > 32)
            {
                MTAL_DP_ERR("Set interface name name length to long\n");
                msg_reply.errCode = -315;
            }
            else
            {
                int ifindex = 0;
                char* ifnames_ptr = (char*)msg_rcv->data;
                char* start = ifnames_ptr;
                char* end;

                printk("Set interface name: %s\n", ifnames_ptr);

                // Reset here and re-enabled in SetInterfaceName
                for (i = 0; i < _MAX_NICS; i++)
                {
                    self->m_Is_NIC_Active[i] = false;
                }

                while ((end = strchr(start, ',')) != NULL) 
                {
                    *end = '\0';
                    printk(KERN_INFO "Set interface name [%d] %s\n", ifindex, start);
                    if (!SetInterfaceName(self, start, ifindex))
                        msg_reply.errCode = -401;
                    else
                        msg_reply.errCode = 0;
                    start = end + 1;
                    ifindex += 1;
                }

                if (*start != '\0')
                {
                    printk(KERN_INFO "Set interface name [%d] %s\n", ifindex, start);
                    if (!SetInterfaceName(self, start, ifindex))
                        msg_reply.errCode = -401;
                    else
                        msg_reply.errCode = 0;
                }
            }
            break;
        }
        case MT_ALSA_Msg_Add_RTPStream:
        {
            /* Payload (Stage 1+): {int32_t pcm_id, TRTP_stream_info}.
             * pcm_id is parsed here; routing the stream's buffer access to
             * the right chip is Task 7 (TRTP_stream_info gains m_uiPCMId,
             * and get_live_{in,out}_jitter_buffer reads from that chip). */
            if (msg_rcv->dataSize != sizeof(int32_t) + sizeof(TRTP_stream_info))
            {
                MTAL_DP_ERR("Add RTP stream invalid data size\n");
                msg_reply.errCode = -315;
            }
            else
            {
                uint64_t stream_handle;
                int32_t pcm_id = *(int32_t*)msg_rcv->data;
                TRTP_stream_info* rtp_stream_info_ptr =
                    (TRTP_stream_info*)((char*)msg_rcv->data + sizeof(int32_t));
                MTAL_DP_INFO("Add RTP stream pcm_id=%d, channels=%d\n",
                             pcm_id, rtp_stream_info_ptr->m_byNbOfChannels);
                /* W5: fail-loud if the message's pcm_id prefix and the
                 * embedded stream binding disagree (also makes the prefix
                 * load-bearing — it was MTAL_DP_INFO-only, an unused
                 * variable in release builds). */
                if (pcm_id < 0 || (uint32_t)pcm_id != rtp_stream_info_ptr->m_uiPCMId)
                {
                    MTAL_DP_ERR("Add RTP stream: prefix pcm_id %d != stream m_uiPCMId %u\n",
                                pcm_id, rtp_stream_info_ptr->m_uiPCMId);
                    msg_reply.errCode = -EINVAL;
                }
                else
                {
                    /* W6: fail-loud stream-rate vs chip-rate validation.
                     * A mismatched stream used to produce silently garbled
                     * audio (the 64-bit RTP stitch drifts immediately).
                     * Compared in the tick-rate clock domain so DSD wire
                     * streams (352.8k container) match a DSD chip
                     * (2,822,400 published). A pcm with no live chip is
                     * rejected too — binding to an empty slot was a
                     * silently dead stream + slot leak (W3-C2 note).
                     * Daemon-side mirror validation lands with the W7
                     * per-group rates. */
                    const struct ravenna_mgr_ops *frontend = smp_load_acquire(&self->m_alsa_driver_frontend);
                    void* stream_chip = get_chip_by_pcm_id(self, pcm_id);
                    uint32_t chip_rate = (stream_chip && frontend && frontend->get_pcm_sample_rate)
                                       ? frontend->get_pcm_sample_rate(stream_chip) : 0;
                    if (chip_rate == 0
                        || tick_rate_for_sample_rate(rtp_stream_info_ptr->m_ui32SamplingRate)
                           != tick_rate_for_sample_rate(chip_rate))
                    {
                        MTAL_DP_ERR("Add RTP stream: stream rate %u does not match pcm %d's configured rate %u (chip %s)\n",
                                    rtp_stream_info_ptr->m_ui32SamplingRate, pcm_id, chip_rate,
                                    stream_chip ? "live" : "absent");
                        msg_reply.errCode = -EINVAL;
                    }
                    else if (add_RTP_stream_(&self->m_RTP_streams_manager, rtp_stream_info_ptr, &stream_handle))
                    {
                        MTAL_DP_INFO("self->m_RTP_streams_manager stream_handle = %llu\n", stream_handle);

                        msg_reply.errCode = 0;
                        msg_reply.dataSize = sizeof(uint64_t);
                        msg_reply.data = &stream_handle;
                        CW_netlink_send_reply_to_user_land(&msg_reply);
                        return; // because stream_handle is outof the scope if send reply at the end of the function
                    }
                    else
                    {
                        /* W5 follow-up fix: this used to sit after the
                         * whole if/else chain and OVERWROTE the -EINVAL
                         * set by the validation branches with -401. */
                        msg_reply.errCode = -401;
                    }
                }
            }
            break;
        }
        case MT_ALSA_Msg_Remove_RTPStream:
        {
            if (msg_rcv->dataSize != sizeof(uint64_t))
            {
                MTAL_DP_ERR("Remove RTP stream invalid data size\n");
                msg_reply.errCode = -315;
            }
            else
            {
                uint64_t* rtp_stream_handle_ptr = (uint64_t*)msg_rcv->data;
                MTAL_DP_INFO("remove_RTP_stream stream_handle = %llu\n", *rtp_stream_handle_ptr);
                if (!remove_RTP_stream_(&self->m_RTP_streams_manager, *rtp_stream_handle_ptr, true))
                    msg_reply.errCode = -401;
                else
                    msg_reply.errCode = 0;
            }
            break;
        }
        case MT_ALSA_Msg_Update_RTPStream_Name:
            break;
        case MT_ALSA_Msg_SetPTPConfig:
        {
            if (msg_rcv->dataSize != sizeof(TPTPConfig))
            {
                MTAL_DP_ERR("Set PTP config invalid data size\n");
                msg_reply.errCode = -315;
            }
            else
            {
                TPTPConfig* ptpConfig = (TPTPConfig*)msg_rcv->data;
                /* W11: configure every (NIC, domain) servo to ITS domain (slot ==
                 * domain number) sharing the daemon's global DSCP. The daemon's
                 * ptpConfig->ui8Domain is not used — domains come from the cards. */
                for (i = 0; i < _MAX_NICS; i++)
                {
                    unsigned int d;
                    for (d = 0; d < MAX_DOMAINS; d++)
                    {
                        TPTPConfig cfg = *ptpConfig;
                        cfg.ui8Domain = (uint8_t)d;
                        SetPTPConfig(&self->m_PTP[i][d], &cfg);
                    }
                }
                msg_reply.errCode = 0;
            }
            break;
        }
        case MT_ALSA_Msg_GetPTPConfig:
        {
            //MTAL_DP_INFO("Get PTP Config\n");

            TPTPConfig ptpConfig;
            /* W11: per-domain GetPTPConfig is the next sub-slice; report domain 0. */
            GetPTPConfig(&self->m_PTP[manager_status_nic(self)][0], &ptpConfig);

            msg_reply.errCode = 0;
            msg_reply.dataSize = sizeof(TPTPConfig);
            msg_reply.data = &ptpConfig;

            CW_netlink_send_reply_to_user_land(&msg_reply);
            return; // because ptpConfig is out of the scope if send reply at the end of the function
        }
        case MT_ALSA_Msg_GetPTPStatus:
        {
            //MTAL_DP_INFO("Get PTP Status\n");

            TPTPStatus ptpStatus;
            /* W11: per-domain status — the request carries the wanted domain as a
             * single byte (a legacy no-data request defaults to domain 0). */
            uint8_t dom = (msg_rcv->dataSize >= 1) ? *(uint8_t*)msg_rcv->data : 0;
            if (dom >= MAX_DOMAINS)
                dom = 0;
            GetPTPStatus(&self->m_PTP[manager_status_nic(self)][dom], &ptpStatus);

            msg_reply.errCode = 0;
            msg_reply.dataSize = sizeof(TPTPStatus);
            msg_reply.data = &ptpStatus;

            CW_netlink_send_reply_to_user_land(&msg_reply);
            return; // because ptpStatus is out of the scope if send reply at the end of the function
        }
        case MT_ALSA_Msg_GetPCMStatus:
        {
            /* #22: per-PCM TIC-engine lock — the chip's (domain, rate) entry's
             * active-NIC engine. An unattached/unknown pcm reports unlocked. */
            struct TPCMStatus pcmStatus;
            int32_t pcm_id = (msg_rcv->dataSize >= (int)sizeof(int32_t))
                                 ? *(int32_t*)msg_rcv->data
                                 : -1;
            pcmStatus.nTICLockStatus = PTPLS_UNLOCKED;
            if (pcm_id >= 0 && pcm_id < MAX_PCMS)
            {
                struct tic_timer_entry* entry =
                    smp_load_acquire(&self->m_apChipEntry[pcm_id]);
                if (entry)
                    pcmStatus.nTICLockStatus =
                        tic_engine_lock_status(active_engine_of(entry));
            }
            msg_reply.errCode = 0;
            msg_reply.dataSize = sizeof(struct TPCMStatus);
            msg_reply.data = &pcmStatus;
            CW_netlink_send_reply_to_user_land(&msg_reply);
            return;
        }
        case MT_ALSA_Msg_SetPCMRate:
        {
            /* W15: in-place per-PCM re-rate. Payload {int32_t pcm_id, uint32_t
             * sample_rate}. Idle chip -> re-key now; busy chip -> ARM (the PCM
             * Rate kcontrol advertises the target + notifies) and return -EBUSY
             * so the daemon retries once the client has released the device. */
            if (msg_rcv->dataSize != (int)(sizeof(int32_t) * 2))
            {
                MTAL_DP_ERR("MT_ALSA_Msg_SetPCMRate invalid data size (got %d, expected %zu)\n",
                            msg_rcv->dataSize, sizeof(int32_t) * 2);
                msg_reply.errCode = -315;
            }
            else
            {
                int32_t  pcm_id   = *(int32_t*)msg_rcv->data;
                uint32_t new_rate = *(uint32_t*)((char*)msg_rcv->data + sizeof(int32_t));
                const struct ravenna_mgr_ops* fe = self->m_alsa_driver_frontend;
                void* chip = (pcm_id >= 0 && pcm_id < MAX_PCMS)
                                 ? get_chip_by_pcm_id(self, pcm_id) : NULL;
                if (!chip || !fe)
                {
                    MTAL_DP_ERR("MT_ALSA_Msg_SetPCMRate: no chip for pcm_id %d\n", pcm_id);
                    msg_reply.errCode = -ENODEV;
                }
                else if (!is_valid_pcm_rate(new_rate))
                {
                    MTAL_DP_ERR("MT_ALSA_Msg_SetPCMRate: pcm_id %d invalid rate %u\n",
                                pcm_id, new_rate);
                    msg_reply.errCode = -EINVAL;
                }
                else if (fe->get_pcm_sample_rate && fe->get_pcm_sample_rate(chip) == new_rate)
                {
                    /* already at the target — clear any stale arm, success. */
                    if (fe->arm_pcm_rate)
                        fe->arm_pcm_rate(chip, 0);
                    msg_reply.errCode = 0;
                }
                else if (fe->pcm_is_idle && fe->pcm_is_idle(chip))
                {
                    msg_reply.errCode = manager_set_pcm_rate(self, pcm_id, new_rate);
                    MTAL_DP_INFO("MT_ALSA_Msg_SetPCMRate pcm_id=%d -> %u (idle, applied err=%d)\n",
                                 pcm_id, new_rate, msg_reply.errCode);
                }
                else
                {
                    if (fe->arm_pcm_rate)
                        fe->arm_pcm_rate(chip, new_rate);
                    msg_reply.errCode = -EBUSY;
                    MTAL_DP_INFO("MT_ALSA_Msg_SetPCMRate pcm_id=%d -> %u (busy, armed; daemon should retry)\n",
                                 pcm_id, new_rate);
                }
            }
            break;
        }
        case MT_ALSA_Msg_SetMasterOutputVolume:
            if (msg_rcv->dataSize != sizeof(int32_t))
            {
                MTAL_DP_ERR("MT_ALSA_Msg_SetMasterOutputVolume invalid data size\n");
                msg_reply.errCode = -315;
            }
            else
            {
                int32_t* value_ptr = (int32_t*)msg_rcv->data;
                ///TODO
                if(self->m_apALSAChip[0] && value_ptr != nullptr)
                    self->m_alsa_driver_frontend->notify_master_volume_change(self->m_apALSAChip[0], 0, *value_ptr);

                msg_reply.errCode = 0;
            }
            break;
        case MT_ALSA_Msg_SetMasterOutputSwitch:
            if (msg_rcv->dataSize != sizeof(int32_t))
            {
                MTAL_DP_ERR("MT_ALSA_Msg_SetMasterOutputSwitch invalid data size\n");
                msg_reply.errCode = -315;
            }
            else
            {
                int32_t* value_ptr = (int32_t*)msg_rcv->data;
                ///TODO
                if(self->m_apALSAChip[0] && value_ptr != nullptr)
                    self->m_alsa_driver_frontend->notify_master_switch_change(self->m_apALSAChip[0], 0, *value_ptr);

                msg_reply.errCode = 0;
            }
            break;
        case MT_ALSA_Msg_SetPlayoutDelay:
        case MT_ALSA_Msg_SetCaptureDelay:
            /* W9 #14: payload {int32_t pcm_id, int32_t delay_in_samples}. The
             * delay is now a per-chip advisory latency (no manager-wide
             * m_n*Delay): resolve the chip for pcm_id and store on it; read at
             * prepare() into runtime->delay. Any in-range pcm_id is accepted. */
            if (msg_rcv->dataSize != sizeof(int32_t) * 2)
            {
                MTAL_DP_ERR("MT_ALSA_Msg_Set%sDelay invalid data size\n",
                            msg_rcv->id == MT_ALSA_Msg_SetPlayoutDelay ? "Playout" : "Capture");
                msg_reply.errCode = -315;
            }
            else
            {
                int32_t pcm_id = *(int32_t*)msg_rcv->data;
                int32_t delay  = *(int32_t*)((char*)msg_rcv->data + sizeof(int32_t));
                bool is_playout = (msg_rcv->id == MT_ALSA_Msg_SetPlayoutDelay);
                void* chip = (pcm_id >= 0 && pcm_id < MAX_PCMS)
                                 ? get_chip_by_pcm_id(self, pcm_id) : NULL;
                if (!chip)
                {
                    MTAL_DP_ERR("MT_ALSA_Msg_Set%sDelay: no chip for pcm_id %d\n",
                                is_playout ? "Playout" : "Capture", pcm_id);
                    msg_reply.errCode = -ENODEV;
                }
                else
                {
                    msg_reply.errCode = is_playout
                        ? mr_alsa_audio_set_playout_delay(chip, delay)
                        : mr_alsa_audio_set_capture_delay(chip, delay);
                    MTAL_DP_INFO("MT_ALSA_Msg_Set%sDelay pcm_id=%d set to %d\n",
                                 is_playout ? "Playout" : "Capture", pcm_id, delay);
                }
            }
            break;
        case MT_ALSA_Msg_AddPCM:
            if (msg_rcv->dataSize != sizeof(struct MT_ALSA_AddPCM_args))
            {
                MTAL_DP_ERR("MT_ALSA_Msg_AddPCM invalid data size (got %d, expected %zu)\n",
                            msg_rcv->dataSize, sizeof(struct MT_ALSA_AddPCM_args));
                msg_reply.errCode = -315;
            }
            else
            {
                struct MT_ALSA_AddPCM_args* args = (struct MT_ALSA_AddPCM_args*)msg_rcv->data;
                MTAL_DP_INFO("MT_ALSA_Msg_AddPCM pcm_id=%d rate=%u in=%u out=%u\n",
                             args->pcm_id, args->sample_rate,
                             args->num_inputs, args->num_outputs);
                /* W14: every PCM carries its OWN rate, set here at AddPCM. There
                 * is no manager-wide m_SampleRate to fall back to — a missing or
                 * unsupported rate is a hard error (the daemon always sends an
                 * explicit, validated rate). is_valid_pcm_rate(0) is false, so
                 * this also rejects rate 0. */
                if (!is_valid_pcm_rate(args->sample_rate))
                {
                    MTAL_DP_ERR("MT_ALSA_Msg_AddPCM rate %u invalid (a per-PCM rate is required)\n",
                                args->sample_rate);
                    msg_reply.errCode = -EINVAL;
                }
                else
                {
                    int err;
                    /* W7: ensure the name is NUL-terminated before use —
                     * it crossed the netlink boundary as a fixed array. */
                    args->name[sizeof(args->name) - 1] = '\0';
                    err = mr_alsa_audio_add_pcm_to_card(args->card_handle, args->pcm_id, args->sample_rate, args->name);
                    msg_reply.errCode = err;
                }
            }
            break;
        case MT_ALSA_Msg_RemovePCM:
            /* Legacy per-PCM remove; superseded by RemoveCard (W10 multi-card). */
            MTAL_DP_ERR("MT_ALSA_Msg_RemovePCM not supported (use RemoveCard)\n");
            msg_reply.errCode = -ENOSYS;
            break;
        case MT_ALSA_Msg_AddCard:
        {
            if (msg_rcv->dataSize != sizeof(struct MT_ALSA_AddCard_args))
            {
                MTAL_DP_ERR("MT_ALSA_Msg_AddCard invalid data size (got %d, expected %zu)\n",
                            msg_rcv->dataSize, sizeof(struct MT_ALSA_AddCard_args));
                msg_reply.errCode = -315;
            }
            else
            {
                struct MT_ALSA_AddCard_args* a = (struct MT_ALSA_AddCard_args*)msg_rcv->data;
                a->id[sizeof(a->id) - 1] = '\0';
                msg_reply.errCode = mr_alsa_audio_add_card(a->card_handle, a->id, a->domain);
            }
            break;
        }
        case MT_ALSA_Msg_RegisterCard:
        {
            if (msg_rcv->dataSize != sizeof(int32_t))
                msg_reply.errCode = -315;
            else
                msg_reply.errCode = mr_alsa_audio_register_card(*(int32_t*)msg_rcv->data);
            break;
        }
        case MT_ALSA_Msg_RemoveCard:
        {
            if (msg_rcv->dataSize != sizeof(int32_t))
                msg_reply.errCode = -315;
            else
                msg_reply.errCode = mr_alsa_audio_remove_card(*(int32_t*)msg_rcv->data);
            break;
        }
        default:
            msg_reply.errCode = -314;
    }

    CW_netlink_send_reply_to_user_land(&msg_reply);
}

//////////////////////////////////////////////////////////////////////////////////
// Statistics
bool GetHALToTICDelta(struct TManager* self, THALToTICDelta* pHALToTICDelta)
{
    if(!pHALToTICDelta)
    {
        return false;
    }

    /*CMTAL_SingleLock nLock(&self->m_csStats, true);
    pHALToTICDelta->i32MinHALToTICDelta = self->m_pmmmHALToTICDelta.GetMin();
    pHALToTICDelta->i32MaxHALToTICDelta = self->m_pmmmHALToTICDelta.GetMax();
    self->m_pmmmHALToTICDelta.ResetAtNextPoint();
    return true;*/

    return false;
}


//////////////////////////////////////////////////////////////////////////////////
/* Bound the _bh lock-hold when wiping a ring. Zeroing a multi-MB buffer under a
 * single lock keeps softirqs disabled for the whole memset and stalls the tick
 * (the latency residue of the 2c98043 deadlock fix). Wipe in 64 KiB chunks,
 * dropping the lock between them. */
#define MR_ALSA_MUTE_CHUNK_INTS 16384u

void MuteInputBuffer(struct TManager* self, void* alsa_chip_pointer)
{
    int32_t* inputBuffer = nullptr;
    uint32_t bufferLength, total_ints, done, chunk_ints;
    int mute_pattern;

    if (!alsa_chip_pointer || !self->m_alsa_driver_frontend)
        return;

    bufferLength = self->m_alsa_driver_frontend->get_capture_buffer_size_in_frames(alsa_chip_pointer);
    if(bufferLength == 0)
    {
        MTAL_DP("CManager::MuteInputBuffer failed: ALSA capture buffer size is 0\n");
        return;
    }

    /* m_NumberOfInputs is the manager-wide channel count; per-chip counts
     * land in W9. Every chip's ring shares the same per-channel layout and
     * MR_ALSA_NB_CHANNELS_MAX capacity, so this bound is safe for all
     * chips and mirrors the legacy chip-0 coverage. */
    total_ints = bufferLength * self->m_NumberOfInputs;
    mute_pattern = get_live_in_mute_pattern(self, 0);

    /* Drop the lock between chunks ONLY when this chip's capture IO is stopped
     * (stop / rate-change): then the tick isn't writing the ring concurrently,
     * so chunking can't clobber live audio and the end state (a fully zeroed
     * ring) is identical. On the start path the IO flag is already set, so the
     * tick may write live audio in parallel -> wipe atomically (one lock hold)
     * to preserve exact behaviour. The full fix that removes the process-context
     * ring writer entirely is the mute-as-flag reshaping (R1). */
    chunk_ints = self->m_alsa_driver_frontend->get_io_state(alsa_chip_pointer, false)
                 ? total_ints : MR_ALSA_MUTE_CHUNK_INTS;

    done = 0;
    while (done < total_ints)
    {
        uint32_t n = (total_ints - done < chunk_ints) ? (total_ints - done) : chunk_ints;
        self->m_alsa_driver_frontend->lock_capture_buffer(alsa_chip_pointer);
        inputBuffer = (int32_t*)(self->m_alsa_driver_frontend->get_capture_buffer(alsa_chip_pointer));
        if(inputBuffer == nullptr)
        {
            MTAL_DP("CManager::MuteInputBuffer failed: No ALSA capture buffer available\n");
            self->m_alsa_driver_frontend->unlock_capture_buffer(alsa_chip_pointer);
            return;
        }
        memset(inputBuffer + done, mute_pattern, sizeof(int32_t) * n);
        self->m_alsa_driver_frontend->unlock_capture_buffer(alsa_chip_pointer);
        done += n;
    }
}

//////////////////////////////////////////////////////////////////////////////////
void MuteOutputBuffer(struct TManager* self, void* alsa_chip_pointer)
{
    int32_t* outputBuffer = nullptr;
    uint32_t bufferLength, total_ints, done, chunk_ints;
    int mute_pattern;

    if (!alsa_chip_pointer || !self->m_alsa_driver_frontend)
        return;

    bufferLength = self->m_alsa_driver_frontend->get_playback_buffer_size_in_frames(alsa_chip_pointer);
    if(bufferLength == 0)
    {
        MTAL_DP("CManager::MuteOutputBuffer failed: ALSA playback buffer size is 0\n");
        return;
    }

    /* See MuteInputBuffer: manager-wide channel count is safe for all
     * chips until per-chip counts land in W9. */
    total_ints = bufferLength * self->m_NumberOfOutputs;
    mute_pattern = get_live_out_mute_pattern(self, 0);

    /* See MuteInputBuffer: chunk only on the stop/quiesced path (playback IO
     * off -> no concurrent tick writer); wipe atomically on the start path. */
    chunk_ints = self->m_alsa_driver_frontend->get_io_state(alsa_chip_pointer, true)
                 ? total_ints : MR_ALSA_MUTE_CHUNK_INTS;

    done = 0;
    while (done < total_ints)
    {
        uint32_t n = (total_ints - done < chunk_ints) ? (total_ints - done) : chunk_ints;
        self->m_alsa_driver_frontend->lock_playback_buffer(alsa_chip_pointer);
        outputBuffer = (int32_t*)(self->m_alsa_driver_frontend->get_playback_buffer(alsa_chip_pointer));
        if(outputBuffer == nullptr)
        {
            MTAL_DP("CManager::MuteOutputBuffer failed: No ALSA playback buffer available\n");
            self->m_alsa_driver_frontend->unlock_playback_buffer(alsa_chip_pointer);
            return;
        }
        memset(outputBuffer + done, mute_pattern, sizeof(int32_t) * n);
        self->m_alsa_driver_frontend->unlock_playback_buffer(alsa_chip_pointer);
        done += n;
    }
}

//////////////////////////////////////////////////////////////////////////////////
uint32_t GetTICFrameSizeAt1FS(struct TManager* self)
{
    return (uint32_t)(self->m_TICFrameSizeAt1FS);
}

//////////////////////////////////////////////////////////////////////////////////
uint32_t GetMaxTICFrameSize(struct TManager* self)
{
    return self->m_MaxFrameSize;
}

//////////////////////////////////////////////////////////////////////////////////
uint32_t GetIPAddress(void* user)
{
    return 0; // TODO
}

//////////////////////////////////////////////////////////////////////////////////
// CEtherTubeAdviseSink
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
EDispatchResult DispatchPacket(struct TManager* self, void* pBuffer, uint32_t packetsize, int mac_header, unsigned char nicId)
{
    EDispatchResult nDispatchResult = DR_PACKET_NOT_USED;
    TUDPPacketBase* pUDPPacketBase = (TUDPPacketBase*)pBuffer;

    if(packetsize < sizeof(TUDPPacketBase) || (mac_header && pUDPPacketBase->EthernetHeader.usType != MTAL_SWAP16(MTAL_ETH_PROTO_IPV4)) || pUDPPacketBase->IPV4Header.byProtocol != IP_PROTO_UDP)
    { // can not be for us
        if((mac_header && pUDPPacketBase->EthernetHeader.usType == MTAL_SWAP16(MTAL_ETH_MAC_CONTROL)) && packetsize >= sizeof(TMACControlFrame))
        {
            TMACControlFrame* pMACControlFrame = (TMACControlFrame*)pBuffer;
            MTAL_DP("receive a MAC CONTROL: could be a PAUSE packet which could mean there is a too slow device (10/100Mb) on the network\n");
            MTAL_DumpMACControlFrame(pMACControlFrame);
        }
        return DR_PACKET_NOT_USED;
    }
    // OK, it's an UDP packet

    //MTAL_DumpIPV4Header(&pUDPPacketBase->IPV4Header);
    //MTAL_DumpUDPHeader(&pUDPPacketBase->UDPHeader);
    //MTAL_DP("packetsize %u\n", packetsize);

    if (nicId < _MAX_NICS)
    {
        if (self->m_Is_NIC_Active[nicId])
        {
            /* W11 feed-all: hand the packet to every domain's servo on this NIC;
             * each self-filters by byDomainNumber (PTP.c), so exactly the matching
             * domain acts. No break — correctness doesn't lean on the per-domain
             * return value, and a non-PTP packet bails cheaply in each on the port
             * check before we fall through to the RTP path. */
            unsigned int d;
            for (d = 0; d < MAX_DOMAINS; d++)
            {
                EDispatchResult rd = process_PTP_packet(&self->m_PTP[nicId][d], pUDPPacketBase, packetsize);
                if (rd != DR_PACKET_NOT_USED)
                    nDispatchResult = rd;
            }
            if (nDispatchResult == DR_PACKET_NOT_USED)
            {
                nDispatchResult = process_UDP_packet(&self->m_RTP_streams_manager, nicId, pUDPPacketBase, packetsize);
            }
        }
    }
    return nDispatchResult;
}

//////////////////////////////////////////////////////////////////////////////////
uint64_t get_global_SAC(void* user)
{
    struct TManager* self = (struct TManager*)user;
    struct tic_timer_entry* entry = smp_load_acquire(&self->m_apChipEntry[0]);
    if (!entry)
        return 0;
    return tic_engine_get_sac(active_engine_of(entry));
}

//////////////////////////////////////////////////////////////////////////////////
/*
 * Multi-rate W5 (step 3): per-PCM SAC — the real per-rate media clock.
 *
 * Each chip's media clock IS its (domain, rate) entry's active engine:
 * frame-quantized at the chip's tick rate, advancing by the chip's own
 * frame size on its own tick. This replaces the W2 stopgap that ratio-
 * scaled the manager clock's SAC — exact only at equal rates; at
 * divergent ones it stepped 44/45-style, never by the chip's frame,
 * breaking SendRTPAudioPackets' one-frame-per-call assumption and the
 * mute path's frame-alignment invariant. Decision 1 ("no rate-conversion
 * math on the hot path") holds again, in full.
 *
 * DSD: tick-rate identity. A DSD chip publishes its bit rate
 * (2,822,400) but its entry — and therefore its SAC — lives in the
 * 352.8k tick clock domain, so there is no x8 scaling (the W2
 * stopgap's DSD bug, fixed by construction).
 *
 * Safe-fail: an unmapped pcm_id resolves to chip 0's entry (the legacy
 * manager-wide clock), 0 if no entry exists at all.
 */
uint64_t get_global_SAC_for_pcm(void* user, uint32_t pcm_id)
{
    struct TManager* self = (struct TManager*)user;
    TTicEngine* engine = get_engine_for_pcm(self, pcm_id);
    if (!engine)
        return 0;
    return tic_engine_get_sac(engine);
}
//////////////////////////////////////////////////////////////////////////////////
/* W5 step 3: tick-rate (registry-key) resolution for the streams
 * manager's pump filter. 0 = unmapped pcm (pumped by no cadence). */
uint32_t get_tick_rate_for_pcm(void* user, uint32_t pcm_id)
{
    struct TManager* self = (struct TManager*)user;
    struct tic_timer_entry* entry = NULL;
    if (pcm_id < MAX_PCMS)
        entry = smp_load_acquire(&self->m_apChipEntry[pcm_id]);
    if (!entry)
        return 0;
    return entry->tick_rate;
}

/* W11 fix: the pcm's PTP domain (the other half of its (domain, rate) registry
 * key), so the pump filter can match domain AND rate. Same entry resolution as
 * get_tick_rate_for_pcm; 0 when the pcm resolves to no entry. */
uint8_t get_domain_for_pcm(void* user, uint32_t pcm_id)
{
    struct TManager* self = (struct TManager*)user;
    struct tic_timer_entry* entry = NULL;
    if (pcm_id < MAX_PCMS)
        entry = smp_load_acquire(&self->m_apChipEntry[pcm_id]);
    if (!entry)
        return 0;
    return entry->domain;
}

//////////////////////////////////////////////////////////////////////////////////
uint64_t get_global_time(void* user)
{
    struct TManager* self = (struct TManager*)user;
    struct tic_timer_entry* entry = smp_load_acquire(&self->m_apChipEntry[0]);
    if (!entry)
        return 0;
    return tic_engine_get_time(active_engine_of(entry));
}
//////////////////////////////////////////////////////////////////////////////////
void get_global_times(void* user, uint64_t* pui64GlobalSAC, uint64_t* pui64GlobalTime, uint64_t* pui64GlobalPerformanceCounter)
{
    struct TManager* self = (struct TManager*)user;
    struct tic_timer_entry* entry = smp_load_acquire(&self->m_apChipEntry[0]);
    if (!entry)
    {
        *pui64GlobalSAC = 0;
        *pui64GlobalTime = 0;
        *pui64GlobalPerformanceCounter = 0;
        return;
    }
    tic_engine_get_times(active_engine_of(entry), pui64GlobalSAC, pui64GlobalTime, pui64GlobalPerformanceCounter);
} // return the time when the audio frame TIC occured

//////////////////////////////////////////////////////////////////////////////////
uint32_t get_frame_size(void* user)
{
    struct TManager* self = (struct TManager*)user;
    return self->m_ui32FrameSize;
}

//////////////////////////////////////////////////////////////////////////////////
// CEthernetFilter_callback
//////////////////////////////////////////////////////////////////////////////////
void get_audio_engine_sample_format(void* user, enum EAudioEngineSampleFormat* pnSampleFormat)
{
    struct TManager* self = (struct TManager*)user;
    switch(GetAudioModeFromRate(self->m_SampleRate))
    {
        case AM_DSD64:
        case AM_DSD128:
        case AM_DSD256:
        case AM_PCM:
        default:
            *pnSampleFormat = AESF_L32;
    }
}

//////////////////////////////////////////////////////////////////////////////////
char get_audio_engine_sample_bytelength(void* user)
{
    struct TManager* self = (struct TManager*)user;
    enum EAudioEngineSampleFormat nSampleFormat;
    get_audio_engine_sample_format(self, &nSampleFormat);
    switch (nSampleFormat)
    {
        case AESF_FLOAT32: return 4;
        case AESF_L32: return 4;
        case AESF_L24: return 3;
        case AESF_L16: return 2;
        case AESF_DSDInt8MSB1: return 1;
        case AESF_DSDInt16MSB1: return 2;
        case AESF_DSDInt32MSB1: return 4;
        default:
        {
            MTAL_DP("get_audio_engine_sample_bytelength UNKNOWN sample format !");
            return 4;
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
// Note: buffer type is retrieved through get_audio_engine_sample_format
void* get_live_in_jitter_buffer(void* user, uint32_t ulChannelId)
{
    struct TManager* self = (struct TManager*)user;
    unsigned char* inputBuffer = nullptr;
    uint32_t bufferLength = RINGBUFFERSIZE;

    inputBuffer = (unsigned char*)(self->m_alsa_driver_frontend->get_capture_buffer(self->m_apALSAChip[0]));
    if(inputBuffer == nullptr || ulChannelId >= self->m_NumberOfInputs)
    {
        MTAL_DP_ERR("CManager::get_live_in_jitter_buffer() failed: retrieving channel #%u buffer jitter buffer \n", ulChannelId + 1);
        return NULL;
    }
    inputBuffer += ulChannelId * bufferLength * get_audio_engine_sample_bytelength(self);

    return inputBuffer;
}

//////////////////////////////////////////////////////////////////////////////////
// Note: buffer type is retrieved through get_audio_engine_sample_format
void* get_live_out_jitter_buffer(void* user, uint32_t ulChannelId)
{
    struct TManager* self = (struct TManager*)user;
    unsigned char* outputBuffer = nullptr;
    uint32_t bufferLength = RINGBUFFERSIZE;

    outputBuffer = (unsigned char*)(self->m_alsa_driver_frontend->get_playback_buffer(self->m_apALSAChip[0]));
    if(outputBuffer == nullptr || ulChannelId >= self->m_NumberOfOutputs)
    {
        MTAL_DP_ERR("CManager::get_live_out_jitter_buffer() failed: retrieving channel #%u buffer jitter buffer \n", ulChannelId + 1);
        return NULL;
    }
    outputBuffer += ulChannelId * bufferLength * get_audio_engine_sample_bytelength(self);

    return outputBuffer;
}

//////////////////////////////////////////////////////////////////////////////////
// Stage 1 multi-PCM: resolve the right chip's playback/capture buffer for a
// stream tagged with pcm_id (from TRTP_stream_info::m_uiPCMId). Called once
// per channel at stream Init time; the resulting pointer is then cached on
// the stream (m_pvLives{In,Out}CircularBuffer[us]). pcm_id is validated; out
// of range or unattached PCMs return NULL.
void* get_live_buffer_for_pcm(void* user, uint32_t pcm_id, uint32_t ulChannelId, int is_capture)
{
    struct TManager* self = (struct TManager*)user;
    unsigned char* buf = nullptr;
    uint32_t bufferLength = RINGBUFFERSIZE;
    void* chip = NULL;
    uint32_t nb_channels;
    const struct ravenna_mgr_ops *frontend;

    if (pcm_id >= self->m_uPCMCount)
    {
        MTAL_DP_ERR("get_live_buffer_for_pcm: pcm_id %u out of range (count=%u)\n",
                    pcm_id, self->m_uPCMCount);
        return NULL;
    }
    /* Acquire-load frontend and chip slot, pairing with smp_store_release
     * in attach_alsa_driver. */
    frontend = smp_load_acquire(&self->m_alsa_driver_frontend);
    chip = smp_load_acquire(&self->m_apALSAChip[pcm_id]);
    if (!chip || !frontend)
        return NULL;

    /* Stage 1: per-PCM channel counts not yet stored; reuse manager-wide. */
    nb_channels = is_capture ? self->m_NumberOfInputs : self->m_NumberOfOutputs;
    if (ulChannelId >= nb_channels)
    {
        MTAL_DP_ERR("get_live_buffer_for_pcm: ch %u >= nb %u (pcm_id=%u, %s)\n",
                    ulChannelId, nb_channels, pcm_id,
                    is_capture ? "capture" : "playback");
        return NULL;
    }

    buf = (unsigned char*)(is_capture
        ? frontend->get_capture_buffer(chip)
        : frontend->get_playback_buffer(chip));
    if (!buf)
        return NULL;
    buf += ulChannelId * bufferLength * get_audio_engine_sample_bytelength(self);
    return buf;
}

//////////////////////////////////////////////////////////////////////////////////
uint32_t get_live_in_jitter_buffer_length(void* user)
{
    struct TManager* self = (struct TManager*)user;
    return self->m_alsa_driver_frontend->get_capture_buffer_size_in_frames(self->m_apALSAChip[0]);
}

//////////////////////////////////////////////////////////////////////////////////
uint32_t get_live_out_jitter_buffer_length(void* user)
{
    struct TManager* self = (struct TManager*)user;
    return self->m_alsa_driver_frontend->get_playback_buffer_size_in_frames(self->m_apALSAChip[0]);
}

//////////////////////////////////////////////////////////////////////////////////
uint32_t get_live_in_jitter_buffer_offset(void* user, const uint64_t ui64CurrentSAC)
{
    struct TManager* self = (struct TManager*)user;

    #if defined(MT_TONE_TEST) || defined (MT_RAMP_TEST) || defined (MTLOOPBACK) || defined (MTTRANSPARENCY_CHECK)
        return (uint32_t)CW_ll_modulo(ui64CurrentSAC, get_live_in_jitter_buffer_length(self));
    #else
        uint32_t live_in_jitter_buffer_length = self->m_alsa_driver_frontend->get_capture_buffer_size_in_frames(self->m_apALSAChip[0]);
        return (uint32_t)CW_ll_modulo(ui64CurrentSAC, live_in_jitter_buffer_length);
    #endif
}

//////////////////////////////////////////////////////////////////////////////////
unsigned char get_live_in_mute_pattern(void* user, uint32_t ulChannelId)
{
    struct TManager* self = (struct TManager*)user;
    return get_live_out_mute_pattern(self, ulChannelId);
}

//////////////////////////////////////////////////////////////////////////////////
uint32_t get_live_out_jitter_buffer_offset(void* user, const uint64_t ui64CurrentSAC)
{
    struct TManager* self = (struct TManager*)user;

    #if defined(MT_TONE_TEST) || defined (MT_RAMP_TEST) || defined (MTLOOPBACK) || defined (MTTRANSPARENCY_CHECK)
        return (uint32_t)CW_ll_modulo(ui64CurrentSAC, get_live_out_jitter_buffer_length(self));
    #else
        uint32_t offset = self->m_alsa_driver_frontend->get_playback_buffer_offset(self->m_apALSAChip[0]);
        const uint32_t sacOffset = (uint32_t)(get_global_SAC(self) - get_frame_size(self) - ui64CurrentSAC);

        if(ui64CurrentSAC > get_global_SAC(self))
        {
            MTAL_DP("CManager::get_live_out_jitter_buffer_offset() wrong SAC request (SAC = %llu)\n", ui64CurrentSAC);
            return 0;
        }
        if(sacOffset > 0) // f10b In reallity this var is always equal to zero.
        {
            MTAL_DP("get_global_SAC(self)=%llu get_frame_size(self)=%llu ui64CurrentSAC=%llu\n", get_global_SAC(self), get_frame_size(self), ui64CurrentSAC);
            MTAL_DP("CManager::get_live_out_jitter_buffer_offset() not the SAC of previous TIC (sacOffset = %u)\n", sacOffset);
            if(sacOffset <= offset)
                offset -= sacOffset;
            else
                offset += get_live_out_jitter_buffer_length(self) - sacOffset;
        }
        //MTAL_DP("CManager::get_live_out_jitter_buffer_offset() returned %u (sacOffset = %u)\n", offset, sacOffset);
        return offset;
    #endif // MT_TONE_TEST
}
//////////////////////////////////////////////////////////////////////////////////
int update_live_in_audio_data_format(void* user, uint32_t ulChannelId, char const * pszCodec)
{
    return 1;
}

//////////////////////////////////////////////////////////////////////////////////
unsigned char get_live_out_mute_pattern(void* user, uint32_t ulChannelId)
{
    struct TManager* self = (struct TManager*)user;
    switch(GetAudioModeFromRate(self->m_SampleRate))
    {
        case AM_PCM:
            return 0;
        case AM_DSD64:
        case AM_DSD128:
        case AM_DSD256:
            return 0x55;
    }
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////
// Multi-rate Stage 2: per-PCM tick-path callbacks.
//
// These resolve the owning chip from pcm_id (m_uiPCMId on the stream),
// acquire-load the chip slot, and route to the chip's own rate / buffers.
// Used by RTP_audio_stream.c hot paths in place of the manager-wide
// variants so streams at different rates compute frame size and buffer
// offsets correctly for their own rate.
//
// Safe-fail on bad pcm_id or empty slot: length=0, offset=0, pattern=0.
// Returning 0 lengths short-circuits hot-path loops (min(length-offset,N)
// becomes 0; the loop body then iterates zero times).
//////////////////////////////////////////////////////////////////////////////////
static void* resolve_chip(struct TManager* self, uint32_t pcm_id, const struct ravenna_mgr_ops **out_frontend)
{
    void *chip;
    const struct ravenna_mgr_ops *frontend;
    if (pcm_id >= smp_load_acquire(&self->m_uPCMCount))
    {
        if (out_frontend)
            *out_frontend = NULL;
        return NULL;
    }
    frontend = smp_load_acquire(&self->m_alsa_driver_frontend);
    chip = smp_load_acquire(&self->m_apALSAChip[pcm_id]);
    if (!chip || !frontend)
    {
        if (out_frontend)
            *out_frontend = NULL;
        return NULL;
    }
    if (out_frontend)
        *out_frontend = frontend;
    return chip;
}

uint32_t get_frame_size_for_pcm(void* user, uint32_t pcm_id)
{
    struct TManager* self = (struct TManager*)user;
    const struct ravenna_mgr_ops *frontend = NULL;
    void *chip = resolve_chip(self, pcm_id, &frontend);
    if (!chip || !frontend || !frontend->get_pcm_frame_size)
        return 0;
    return frontend->get_pcm_frame_size(chip);
}

uint32_t get_live_in_jitter_buffer_length_for_pcm(void* user, uint32_t pcm_id)
{
    struct TManager* self = (struct TManager*)user;
    const struct ravenna_mgr_ops *frontend = NULL;
    void *chip = resolve_chip(self, pcm_id, &frontend);
    if (!chip || !frontend)
        return 0;
    return frontend->get_capture_buffer_size_in_frames(chip);
}

uint32_t get_live_out_jitter_buffer_length_for_pcm(void* user, uint32_t pcm_id)
{
    struct TManager* self = (struct TManager*)user;
    const struct ravenna_mgr_ops *frontend = NULL;
    void *chip = resolve_chip(self, pcm_id, &frontend);
    if (!chip || !frontend)
        return 0;
    return frontend->get_playback_buffer_size_in_frames(chip);
}

uint32_t get_live_in_jitter_buffer_offset_for_pcm(void* user, uint32_t pcm_id, const uint64_t ui64CurrentSAC)
{
    uint32_t len = get_live_in_jitter_buffer_length_for_pcm(user, pcm_id);
    if (len == 0)
        return 0;
    return (uint32_t)CW_ll_modulo(ui64CurrentSAC, len);
}

uint32_t get_live_out_jitter_buffer_offset_for_pcm(void* user, uint32_t pcm_id, const uint64_t ui64CurrentSAC)
{
    struct TManager* self = (struct TManager*)user;
    const struct ravenna_mgr_ops *frontend = NULL;
    void *chip = resolve_chip(self, pcm_id, &frontend);
    if (!chip || !frontend)
        return 0;
    #if defined(MT_TONE_TEST) || defined(MT_RAMP_TEST) || defined(MTLOOPBACK) || defined(MTTRANSPARENCY_CHECK)
    {
        /* Test/loopback paths: simple modulo against the ring length. */
        uint32_t test_len = frontend->get_playback_buffer_size_in_frames(chip);
        if (test_len == 0)
            return 0;
        return (uint32_t)CW_ll_modulo(ui64CurrentSAC, test_len);
    }
    #else
    {
        /* Production path mirrors the manager-wide
         * get_live_out_jitter_buffer_offset (chip 0) but reads everything
         * per-PCM via the chip slot we just resolved. fsize falls back to
         * the manager-wide value if pcm_frame_size hasn't been published
         * yet (shouldn't happen — attach_alsa_driver publishes before the
         * chip slot — but the safe-fail keeps a misordering bug from
         * producing garbage arithmetic). */
        uint32_t offset = frontend->get_playback_buffer_offset(chip);
        uint32_t len = frontend->get_playback_buffer_size_in_frames(chip);
        uint32_t fsize = frontend->get_pcm_frame_size ? frontend->get_pcm_frame_size(chip) : 0;
        uint64_t global_sac;
        uint32_t sac_offset;
        if (fsize == 0)
            fsize = self->m_ui32FrameSize;
        /* Stage 3: per-chip SAC (was manager-wide get_global_SAC in the
         * Stage 2 first cut — correct only for chips at the manager rate). */
        global_sac = get_global_SAC_for_pcm(self, pcm_id);
        if (ui64CurrentSAC > global_sac)
        {
            MTAL_DP("get_live_out_jitter_buffer_offset_for_pcm(pcm=%u): bad SAC request (%llu > global %llu)\n",
                    pcm_id, ui64CurrentSAC, global_sac);
            return 0;
        }
        sac_offset = (uint32_t)(global_sac - fsize - ui64CurrentSAC);
        if (sac_offset > 0 && len > 0)
        {
            if (sac_offset <= offset)
                offset -= sac_offset;
            else
                offset += len - sac_offset;
        }
        return offset;
    }
    #endif
}

unsigned char get_live_in_mute_pattern_for_pcm(void* user, uint32_t pcm_id, uint32_t ulChannelId)
{
    /* Mute pattern is rate-mode-dependent (PCM vs DSD); per chip's own
     * rate, not manager-wide. */
    struct TManager* self = (struct TManager*)user;
    const struct ravenna_mgr_ops *frontend = NULL;
    void *chip = resolve_chip(self, pcm_id, &frontend);
    uint32_t rate;
    if (!chip || !frontend || !frontend->get_pcm_sample_rate)
        return 0;
    rate = frontend->get_pcm_sample_rate(chip);
    switch (GetAudioModeFromRate(rate))
    {
        case AM_PCM:
            return 0;
        case AM_DSD64:
        case AM_DSD128:
        case AM_DSD256:
            return 0x55;
    }
    return 0;
}

unsigned char get_live_out_mute_pattern_for_pcm(void* user, uint32_t pcm_id, uint32_t ulChannelId)
{
    /* Same pattern logic as in; share the implementation. */
    return get_live_in_mute_pattern_for_pcm(user, pcm_id, ulChannelId);
}

//////////////////////////////////////////////////////////////////////////////////
// Caudio_streamer_clock_PTP_callback
//////////////////////////////////////////////////////////////////////////////////
/* W5: the audio pump, per (domain, rate) entry — services ONLY this
 * entry's chips (chip-map identity) and streams (stream_on_tick via
 * get_tick_rate_for_pcm), at this entry's cadence. Gated on the
 * per-entry predicate — this entry's active engine fully locked — which
 * is the same condition as NIC-selection eligibility by construction. */
static void manager_audio_frame_tic(struct TManager* self, struct tic_timer_entry* entry)
{
    prepare_buffer_lives(&self->m_RTP_streams_manager, entry->domain, entry->tick_rate);
    
    if (self->m_bIORunning && tic_engine_lock_status(&entry->engine[entry->active_nic]) == PTPLS_LOCKED)
    {
        #ifdef MTTRANSPARENCY_CHECK
        {
            uint32_t ui32Channel = MTTRANSPARENCY_CHECK_CHANNEL_IDX;
            uint32_t ui32Offset = get_live_out_jitter_buffer_offset(self, get_global_SAC(self));

            float *pfInputBuffer = NULL;
            float *pfOutputBuffer = (float*)get_live_out_jitter_buffer(ui32Channel) + ui32Offset;

            bool bStatusChanged = false;
            self->m_Transparencycheck.ProcessFloat(get_global_SAC(self), pfInputBuffer, pfOutputBuffer, get_frame_size(self), &bStatusChanged);
        }
        #endif

        #ifdef MTLOOPBACK
        {
            uint32_t ui32Channel = MTLOOPBACK_CHANNEL_IDX;
            uint32_t ui32Offset = get_live_out_jitter_buffer_offset(self, get_global_SAC(self));

            void *pfInputBuffer = (char*)get_live_in_jitter_buffer(self, ui32Channel) + ui32Offset * get_audio_engine_sample_bytelength(self);
            void *pfOutputBuffer = (char*)get_live_out_jitter_buffer(self, ui32Channel) + ui32Offset * get_audio_engine_sample_bytelength(self);

            memcpy(pfOutputBuffer, pfInputBuffer, get_audio_engine_sample_bytelength(self) * get_frame_size(self));
            //memset(pfOutputBuffer, 0x58, sizeof(int32_t) * get_frame_size(self)); // 4 byte because streams are padded to word of 32bits

            /// write live outputs
            frame_process_begin(&self->m_RTP_streams_manager, entry->domain, entry->tick_rate);
            frame_process_end(&self->m_RTP_streams_manager);
        }
        #elif defined(MT_TONE_TEST) || defined (MT_RAMP_TEST)
        {
            uint32_t ui32Offset = get_live_out_jitter_buffer_offset(self, get_global_SAC(self));
            uint32_t stepOut = get_audio_engine_sample_bytelength(self);

            frame_process_begin(&self->m_RTP_streams_manager, entry->domain, entry->tick_rate);

            #if defined(MT_TONE_TEST)
            int* LUT = &sinebuf[0];
            unsigned int LUTnbPoints = 48;
            unsigned int LUTSampleRate = 48000;

            switch(self->m_SampleRate)
            {
                case 96000:
                    LUT = &sinebuf_96k[0];
                    LUTnbPoints = 96;
                    LUTSampleRate = 96000;
                    break;
                case 192000:
                    LUT = &sinebuf_192k[0];
                    LUTnbPoints = 192;
                    LUTSampleRate = 192000;
                    break;
                case 384000:
                    LUT = &sinebuf_384k[0];
                    LUTnbPoints = 384;
                    LUTSampleRate = 384000;
                    break;
            }
            #endif // MT_TONE_TEST

            for(uint32_t chIdx = 0; chIdx < self->m_NumberOfOutputs; ++chIdx)
            {
                unsigned char* buf = (unsigned char*)get_live_out_jitter_buffer(chIdx) + ui32Offset * get_audio_engine_sample_bytelength(self);
                if(chIdx == 0)
                {
                    for(uint32_t ui32SampleIdx = 0; ui32SampleIdx < get_frame_size(self); ui32SampleIdx++)
                    {
                        #if defined(MT_TONE_TEST)
                        unsigned long p = (self->m_tone_test_phase * self->m_SampleRate) / LUTSampleRate;
                        int16_t val16 = LUT[CW_ll_modulo((p + 4 * chIdx), LUTnbPoints)]/* >> 1*/;
                        int32_t val24 = val16 << 8;
                        self->m_tone_test_phase = CW_ll_modulo((self->m_tone_test_phase + 1), (LUTnbPoints * 100));
                        #elif defined(MT_RAMP_TEST)
                        int32_t val24 = self->m_ramp_test_phase;
                        if(val24 >= 8388608) // 2^23
                            val24 = -8388608;
                        self->m_ramp_test_phase += 256;
                        if(self->m_ramp_test_phase >= 8388608) // 2^23
                            self->m_ramp_test_phase = -8388608;

                        #endif // MT_RAMP_TEST

                        /// 32  bit output
                        buf[0] = 0;
                        buf[1] = ((unsigned char*)&val24)[0];
                        buf[2] = ((unsigned char*)&val24)[1];
                        buf[3] = ((unsigned char*)&val24)[2];
                        buf += stepOut;
                    }
                }
                else
                {
                    unsigned char* bufSrc = (unsigned char*)get_live_out_jitter_buffer(0) + ui32Offset * get_audio_engine_sample_bytelength(self);
                    memcpy(buf, bufSrc, get_frame_size() * get_audio_engine_sample_bytelength(self));
                }
            }
            frame_process_end(&self->m_RTP_streams_manager);
        }
        #else
            /// write live outputs
            frame_process_begin(&self->m_RTP_streams_manager, entry->domain, entry->tick_rate);
            {
                /* Acquire-load frontend and count, then each slot, so we
                 * pair with smp_store_release in attach_alsa_driver and
                 * detach_alsa_driver. The loop body runs in softirq
                 * context; we must not see a non-NULL chip pointer or a
                 * non-NULL frontend whose contents aren't published. */
                const struct ravenna_mgr_ops *frontend = smp_load_acquire(&self->m_alsa_driver_frontend);
                if (frontend)
                {
                    uint32_t count = smp_load_acquire(&self->m_uPCMCount);
                    uint32_t i;
                    for (i = 0; i < count; ++i)
                    {
                        void *chip = smp_load_acquire(&self->m_apALSAChip[i]);
                        if (!chip)
                            continue;
                        /* W5 step 3: a chip ticks only at its own entry's
                         * cadence. Entry identity via the chip map — exact
                         * even if two entries were ever to share a tick
                         * rate momentarily. */
                        if (smp_load_acquire(&self->m_apChipEntry[i]) != entry)
                            continue;
                        if (frontend->get_io_state(chip, false))
                            frontend->pcm_interrupt(chip, 1);
                        if (frontend->get_io_state(chip, true))
                            frontend->pcm_interrupt(chip, 0);
                    }
                }
            }
            frame_process_end(&self->m_RTP_streams_manager);
        #endif
    }
}

/* clock_ptp_ops-compatible legacy entry point. The ops-table member has
 * been unused since the Linux port routed the pump through module_main
 * (now through manager_entry_tick), but keep the contract honest: route
 * via chip 0's entry. */
void AudioFrameTIC(void* user)
{
    struct TManager* self = (struct TManager*)user;
    struct tic_timer_entry* entry = smp_load_acquire(&self->m_apChipEntry[0]);
    if (entry)
        manager_audio_frame_tic(self, entry);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Streams <> Ravenna Manager communication
void Init_C_Callbacks(struct TManager* self)
{
    self->m_c_callbacks.user = self;
    self->m_c_callbacks.get_global_SAC = &get_global_SAC;
    self->m_c_callbacks.get_global_time = &get_global_time;
    self->m_c_callbacks.get_global_times = &get_global_times;
    self->m_c_callbacks.get_frame_size = &get_frame_size;
    self->m_c_callbacks.get_audio_engine_sample_format = &get_audio_engine_sample_format;
    self->m_c_callbacks.get_live_in_jitter_buffer = &get_live_in_jitter_buffer;
    self->m_c_callbacks.get_live_out_jitter_buffer = &get_live_out_jitter_buffer;
    self->m_c_callbacks.get_live_in_jitter_buffer_length = &get_live_in_jitter_buffer_length;
    self->m_c_callbacks.get_live_out_jitter_buffer_length = &get_live_out_jitter_buffer_length;
    self->m_c_callbacks.get_live_in_jitter_buffer_offset = &get_live_in_jitter_buffer_offset;
    self->m_c_callbacks.get_live_out_jitter_buffer_offset = &get_live_out_jitter_buffer_offset;
    self->m_c_callbacks.update_live_in_audio_data_format = &update_live_in_audio_data_format;
    self->m_c_callbacks.get_live_in_mute_pattern = &get_live_in_mute_pattern;
    self->m_c_callbacks.get_live_out_mute_pattern = &get_live_out_mute_pattern;
    self->m_c_callbacks.get_live_buffer_for_pcm = &get_live_buffer_for_pcm;
    /* Multi-rate Stage 2: per-PCM tick-path variants used by RTP streams
     * on the hot path. RTP_audio_stream.c calls these with the stream's
     * pcm_id (TRTP_stream_info::m_uiPCMId) so streams at different rates
     * compute frame size / buffer offsets against their own chip's state. */
    self->m_c_callbacks.get_global_SAC_for_pcm = &get_global_SAC_for_pcm;
    self->m_c_callbacks.get_frame_size_for_pcm = &get_frame_size_for_pcm;
    self->m_c_callbacks.get_live_in_jitter_buffer_length_for_pcm = &get_live_in_jitter_buffer_length_for_pcm;
    self->m_c_callbacks.get_live_out_jitter_buffer_length_for_pcm = &get_live_out_jitter_buffer_length_for_pcm;
    self->m_c_callbacks.get_live_in_jitter_buffer_offset_for_pcm = &get_live_in_jitter_buffer_offset_for_pcm;
    self->m_c_callbacks.get_live_out_jitter_buffer_offset_for_pcm = &get_live_out_jitter_buffer_offset_for_pcm;
    self->m_c_callbacks.get_live_in_mute_pattern_for_pcm = &get_live_in_mute_pattern_for_pcm;
    self->m_c_callbacks.get_live_out_mute_pattern_for_pcm = &get_live_out_mute_pattern_for_pcm;
    /* W5 step 3: per-(domain, rate) pump filter. */
    self->m_c_callbacks.get_tick_rate_for_pcm = &get_tick_rate_for_pcm;
    self->m_c_callbacks.get_domain_for_pcm = &get_domain_for_pcm;  /* W11 fix */
    //m_c_dispatch_callbacks.user = this;
    //m_c_dispatch_callbacks.DispatchPacket = &DispatchPacket;
    self->m_c_audio_streamer_clock_PTP_callback.user = self;
    self->m_c_audio_streamer_clock_PTP_callback.GetIPAddress = &GetIPAddress;
    self->m_c_audio_streamer_clock_PTP_callback.AudioFrameTIC = &AudioFrameTIC;
}
rtp_audio_stream_ops* Get_C_Callbacks(struct TManager* self)
{
    return &self->m_c_callbacks;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// ALSA <> Ravenna Manager communication
/*
 * Stage 1 multi-PCM: chips are indexed in m_apALSAChip[] by their pcm_id
 * (matches the snd_pcm_new device argument). m_uPCMCount is the high-water
 * mark — the TIC loop iterates [0, m_uPCMCount) and skips NULL slots, so
 * sparse layouts (e.g. ids 1 and 3 without id 2) work correctly. Earlier
 * code indexed by insertion order which broke when ids arrived out of
 * order.
 *
 * Publish ordering: the hrtimer TIC reads these from softirq context
 * concurrent with this netlink-context add path. The chip slot is
 * written with smp_store_release() so a reader using READ_ONCE() sees a
 * fully-initialised chip (its fields published by the
 * mr_alsa_audio_chip_create path that ran before this call) before the
 * slot pointer becomes visible.
 */
int attach_alsa_driver(void* user, const struct ravenna_mgr_ops *ops, void *alsa_chip_pointer, int pcm_id)
{
    struct TManager* self = (struct TManager*)user;
    uint32_t new_count;
    if (!ops || !alsa_chip_pointer)
        return -EINVAL;
    if (pcm_id < 0 || pcm_id >= MAX_PCMS)
    {
        MTAL_DP("attach_alsa_driver: pcm_id %d out of range [0..%d)\n", pcm_id, MAX_PCMS);
        return -EINVAL;
    }
    /* Refuse re-attach of the same slot or duplicate chip pointer. */
    if (self->m_apALSAChip[pcm_id])
    {
        MTAL_DP("attach_alsa_driver: pcm_id %d already attached\n", pcm_id);
        return -EEXIST;
    }
    /* Frontend ops are identical across chips; set once on first attach.
     * Use release ordering so a concurrent reader (typically the hrtimer
     * AudioFrameTIC, which also acquire-loads the chip slot below) sees
     * a non-NULL frontend pointer with all the function pointers in the
     * struct visible. */
    if (!READ_ONCE(self->m_alsa_driver_frontend))
        smp_store_release(&self->m_alsa_driver_frontend, ops);
    /*
     * Multi-rate Stage 2: establish the chip's pcm_sample_rate /
     * pcm_frame_size BEFORE the slot publish below. Two cases:
     *
     *  (a) Chip 0 at probe: mr_alsa_audio_chip_probe didn't pre-set a
     *      rate, so the chip's pcm_sample_rate is 0 (from kzalloc) when
     *      it reaches attach. Inherit the manager's current m_SampleRate
     *      (= DEFAULT_SAMPLERATE at probe time, then updated when the
     *      daemon sends MT_ALSA_Msg_SetSampleRate via SetSamplingRate
     *      → UpdateFrameSize → set_pcm_sample_rate on chip 0).
     *
     *  (b) Chips 1+ via mr_alsa_audio_add_pcm: the caller pre-stashed a
     *      rate on chip->pcm_sample_rate before chip_create ran. Honor
     *      it. (The AddPCM handler is what decides the effective rate
     *      from args->sample_rate or m_SampleRate.)
     *
     * In both cases we compute frame_size from the chosen rate and
     * publish both fields via the ops vtable's smp_store_release before
     * publishing the slot pointer. The publish-before-slot ordering
     * ensures that any reader who acquire-loads the chip slot and then
     * the chip's rate/frame_size sees a coherent pair.
     */
    {
        uint32_t effective_rate = self->m_SampleRate;
        struct tic_timer_entry* entry;
        if (ops->set_pcm_sample_rate && ops->get_pcm_sample_rate)
        {
            uint32_t chip_rate = ops->get_pcm_sample_rate(alsa_chip_pointer);
            uint32_t fsize;
            if (chip_rate)
                effective_rate = chip_rate;
            fsize = compute_frame_size_for_rate(
                effective_rate,
                self->m_TICFrameSizeAt1FS,
                self->m_MaxFrameSize);
            ops->set_pcm_sample_rate(alsa_chip_pointer, effective_rate, fsize);
            MTAL_DP("attach_alsa_driver: pcm_id %d rate=%u frame_size=%u (chip-prestashed=%u)\n",
                    pcm_id, effective_rate, fsize, chip_rate);
        }
        /* W5/W11: bind the chip to its (domain, tick_rate) timer entry — the
         * per-chip clock handle. The domain is the chip's owning card's PTP
         * domain (W11; was pinned 0). The map is published before the chip slot
         * so a tick-path reader that acquires the chip pointer always finds the
         * entry. */
        uint8_t chip_domain = ops->get_pcm_domain ? ops->get_pcm_domain(alsa_chip_pointer) : 0;
        if (chip_domain >= MAX_DOMAINS)
        {
            MTAL_DP("attach_alsa_driver: pcm_id %d: domain %u >= MAX_DOMAINS %u — using 0\n",
                    pcm_id, chip_domain, MAX_DOMAINS);
            chip_domain = 0;
        }
        /* W15: registry lock — serialize the entry get-or-create + the
         * chip->entry + chip-slot publishes against an in-place re-key from
         * pcm_close (the only other table mutator context). */
        mutex_lock(&g_registry_lock);
        entry = get_or_create_tic_entry(self, chip_domain, effective_rate);
        if (!entry)
        {
            mutex_unlock(&g_registry_lock);
            MTAL_DP("attach_alsa_driver: pcm_id %d: no (domain, rate) timer entry for rate %u\n",
                    pcm_id, effective_rate);
            return -EINVAL;
        }
        smp_store_release(&self->m_apChipEntry[pcm_id], entry);
        /* Publish the slot with release semantics so the hrtimer reader sees
         * the chip pointer only after the chip itself is fully constructed. */
        smp_store_release(&self->m_apALSAChip[pcm_id], alsa_chip_pointer);
        new_count = (uint32_t)pcm_id + 1;
        if (new_count > self->m_uPCMCount)
            smp_store_release(&self->m_uPCMCount, new_count);
        mutex_unlock(&g_registry_lock);
    }
    return 0;
}

void detach_alsa_driver(void* user, void *alsa_chip_pointer)
{
    struct TManager* self = (struct TManager*)user;
    uint32_t i;
    if (!self || !alsa_chip_pointer)
        return;
    /* W15: registry lock — serialize the slot/entry clear + put against an
     * in-place re-key (pcm_close). Held only around the table mutation; the
     * caller (teardown_card) runs detach for every chip and THEN calls
     * snd_card_free without this lock, so the lock is never held across a
     * close-wait (no deadlock). */
    mutex_lock(&g_registry_lock);
    for (i = 0; i < self->m_uPCMCount; ++i)
    {
        if (self->m_apALSAChip[i] != alsa_chip_pointer)
            continue;
        /* Clear with release so any concurrent reader sees NULL atomically
         * (no half-cleared pointer). Don't shrink m_uPCMCount: the slot is
         * skipped by the NULL check, and not shrinking avoids a TOCTOU
         * window where the reader's bound check passes against a stale
         * count but the slot has just been cleared. */
        smp_store_release(&self->m_apALSAChip[i], NULL);
        /* W5: drop the chip's timer-entry reference (last chip at the
         * rate frees the entry). Map cleared after the chip slot so no
         * reader resolves a live chip to a vanished entry. */
        {
            struct tic_timer_entry* entry = self->m_apChipEntry[i];
            smp_store_release(&self->m_apChipEntry[i], NULL);
            put_tic_entry(self, entry);
        }
        mutex_unlock(&g_registry_lock);
        return;
    }
    mutex_unlock(&g_registry_lock);
}

void* get_chip_by_pcm_id(struct TManager* self, int32_t pcm_id)
{
    void* chip;
    if (pcm_id < 0 || pcm_id >= MAX_PCMS)
        return NULL;
    /* Read paired with smp_store_release in attach_alsa_driver. */
    chip = smp_load_acquire(&self->m_apALSAChip[pcm_id]);
    return chip;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
/// called from alsa driver
/// effective change will be done by SetSampleRate once manger has received MT_ALSA_Msg_SetSampleRate message from daemon
////////////////////////////////////////////////////////////////////////////////////////////////////////
int set_sample_rate(void* user, uint32_t rate)
{
    struct TManager* self = (struct TManager*)user;
    int err = 0;
    int nbloop = 0;

    struct MT_ALSA_msg msgSent;
    MTAL_DP("CManager::set_sample_rate from %u to %u\n", self->m_SampleRate, rate);
    msgSent.id = MT_ALSA_Msg_SetSampleRate;
    msgSent.errCode = 0;
    msgSent.dataSize = sizeof(rate);
    msgSent.data = (void*)&rate;
    err = CW_netlink_send_msg_to_user_land(&msgSent, NULL);
    if(err != 0)
        return -EPIPE;
    else
    {
        if(IsStarted(self))
        {
            /* W11: legacy global-rate path — wait on chip 0's domain servos,
             * bounded by the nbloop cap (a GM-less domain times out, never hangs). */
            struct tic_timer_entry* e0 = smp_load_acquire(&self->m_apChipEntry[0]);
            uint8_t dom = e0 ? e0->domain : 0;
            do
            {
                CW_msleep_interruptible(1);
                if(++nbloop >= 4000)
                {
                    MTAL_DP("CManager::set_sample_rate PTP lock timed out\n");
                    return false;
                }
            }
            while (GetLockStatus(&self->m_PTP[0][dom]) != PTPLS_LOCKED || GetLockStatus(&self->m_PTP[1][dom]) != PTPLS_LOCKED);
            MTAL_DP("CManager::set_sample_rate completed\n");
        }
        return 0;
    }
}

int get_sample_rate(void* user, uint32_t *rate)
{
    struct TManager* self = (struct TManager*)user;
    if(rate)
    {
        int err = 0;
        *rate = self->m_SampleRate;
        return err;
    }
    return -EINVAL;
}

int get_jitter_buffer_sample_bytelength(void* user, char *byte_len)
{
    struct TManager* self = (struct TManager*)user;
    if(byte_len)
    {
        int err = 0;
        *byte_len = get_audio_engine_sample_bytelength(self);
        return err;
    }
    return -EINVAL;
}

int get_nb_inputs(void* user, uint32_t *nb_Channels)
{
    struct TManager* self = (struct TManager*)user;
    if(nb_Channels)
    {
        *nb_Channels = self->m_NumberOfInputs;
        return 0;
    }
    return -EINVAL;
}

int get_nb_outputs(void* user, uint32_t *nb_Channels)
{
    struct TManager* self = (struct TManager*)user;
    if(nb_Channels)
    {
        *nb_Channels = self->m_NumberOfOutputs;
        return 0;
    }
    return -EINVAL;
}

/* W9 #14: get_playout_delay / get_capture_delay removed — the delay is now a
 * per-chip property read directly at prepare() (see audio_driver.c), not a
 * manager-wide value fetched via callback. */

int get_output_jitter_buffer_offset(void* user, uint32_t *offset)
{
    struct TManager* self = (struct TManager*)user;
    if (offset)
    {
        *offset = get_live_out_jitter_buffer_offset(self, get_global_SAC(self));
    }
    return -EINVAL;
}

int get_input_jitter_buffer_offset(void* user, uint32_t *offset)
{
    struct TManager* self = (struct TManager*)user;
    if (offset)
    {
        *offset = get_live_in_jitter_buffer_offset(self, get_global_SAC(self));
    }
    return -EINVAL;
}

/*
 * 2026-06-09 review fix (chip-N capture misalignment): capture prepare on
 * chip N must compute its start offset against chip N's own ring length
 * and chip N's own SAC. The legacy callback above is chip-0 only — using
 * it for chip N phase-misaligned the capture read pointer against the
 * per-PCM RTP write pointer (the ring lengths differ whenever chip 0's
 * substream open/closed state differs from chip N's). Identity for
 * pcm_id 0: same SAC, same ring as the legacy path.
 */
int get_input_jitter_buffer_offset_for_pcm(void* user, uint32_t pcm_id, uint32_t *offset)
{
    struct TManager* self = (struct TManager*)user;
    if (offset)
    {
        *offset = get_live_in_jitter_buffer_offset_for_pcm(self, pcm_id, get_global_SAC_for_pcm(self, pcm_id));
        return 0;
    }
    return -EINVAL;
}

int get_min_interrupts_frame_size(void* user, uint32_t *framesize)
{
    struct TManager* self = (struct TManager*)user;
    if(framesize)
    {
        *framesize = GetTICFrameSizeAt1FS(self);
        return 0;
    }
    return -EINVAL;
}

int get_max_interrupts_frame_size(void* user, uint32_t *framesize)
{
    struct TManager* self = (struct TManager*)user;
    if(framesize)
    {
        *framesize = min(GetMaxTICFrameSize(self), GetTICFrameSizeAt1FS(self) * 8); // TODO: increase factor to 16 if we do 16Fs
        return 0;
    }
    return -EINVAL;
}

int get_interrupts_frame_size(void* user, uint32_t *framesize)
{
    struct TManager* self = (struct TManager*)user;
    if(framesize)
    {
        *framesize = get_frame_size(self);
        return 0;
    }
    return -EINVAL;
}

int start_interrupts(void* user, void* alsa_chip_pointer, bool is_playback)
{
    struct TManager* self = (struct TManager*)user;

    MTAL_DP("entering CManager::start_interrupts..\n");
    if (alsa_chip_pointer && self->m_alsa_driver_frontend)
        self->m_alsa_driver_frontend->set_io_state(alsa_chip_pointer, is_playback, true);
    if(startIO(self, alsa_chip_pointer, is_playback)) {
        return 0;
    }
    return -1;
}

int stop_interrupts(void* user, void* alsa_chip_pointer, bool is_playback)
{
    struct TManager* self = (struct TManager*)user;

	MTAL_DP("entering CManager::stop_interrupts..\n");
    if (alsa_chip_pointer && self->m_alsa_driver_frontend)
        self->m_alsa_driver_frontend->set_io_state(alsa_chip_pointer, is_playback, false);
    if (stopIO(self, alsa_chip_pointer, is_playback)) {
        return 0;
    }
    return -1;
}

int notify_master_volume_change(void* user, int direction, int32_t value)
{
    if(direction == 0)
    {
        int err = 0;
        struct MT_ALSA_msg msgSent, msgAnswer;
        msgSent.id = MT_ALSA_Msg_SetMasterOutputVolume;
        msgSent.errCode = 0;
        msgSent.dataSize = sizeof(value);
        msgSent.data = (void*)&value;

        err = CW_netlink_send_msg_to_user_land(&msgSent, &msgAnswer);
        if(err != 0)
            return -EPIPE;
        else
            return 0;
    }
    return -EINVAL;
}

int notify_master_switch_change(void* user, int direction, int32_t value)
{
    if(direction == 0)
    {
        int err = 0;
        struct MT_ALSA_msg msgSent, msgAnswer;
        msgSent.id = MT_ALSA_Msg_SetMasterOutputSwitch;
        msgSent.errCode = 0;
        msgSent.dataSize = sizeof(value);
        msgSent.data = (void*)&value;

        err = CW_netlink_send_msg_to_user_land(&msgSent, &msgAnswer);
        if(err != 0)
            return -EPIPE;
        else
            return 0;
    }
    return -EINVAL;
}

int get_master_volume_value(void* user, int direction, int32_t* value)
{
    if(value && direction == 0)
    {
        int err = 0;
        struct MT_ALSA_msg msgAnswer;
        struct MT_ALSA_msg msgSent;
        msgAnswer.data = value;
        msgAnswer.dataSize = sizeof(*value);
        msgSent.id = MT_ALSA_Msg_GetMasterOutputVolume;
        msgSent.errCode = 0;
        msgSent.dataSize = 0;
        msgSent.data = nullptr;

        err = CW_netlink_send_msg_to_user_land(&msgSent, &msgAnswer);
        if(err != 0)
        {
            return -EPIPE;
        }
        else
        {
            if (msgAnswer.errCode == 0 && msgAnswer.dataSize == sizeof(int32_t))
            {
            }
            else
            {
                MTAL_DP("CManager::get_master_volume_value FAILED\n");
            }
            return msgAnswer.errCode;
        }
    }
    return -EINVAL;
}

int get_master_switch_value(void* user, int direction, int32_t* value)
{
    if(value && direction == 0)
    {
        int err = 0;
        struct MT_ALSA_msg msgAnswer;
        struct MT_ALSA_msg msgSent;
        msgAnswer.data = value;
        msgAnswer.dataSize = sizeof(*value);
        msgSent.id = MT_ALSA_Msg_GetMasterOutputSwitch;
        msgSent.errCode = 0;
        msgSent.dataSize = 0;
        msgSent.data = nullptr;

        err = CW_netlink_send_msg_to_user_land(&msgSent, &msgAnswer);
        if(err != 0)
        {
            return -EPIPE;
        }
        else
        {
            if (msgAnswer.errCode == 0 && msgAnswer.dataSize == sizeof(int32_t))
            {
            }
            else
            {
                MTAL_DP("CManager::get_master_switch_value FAILED\n");
            }
            return msgAnswer.errCode;
        }
    }
    return -EINVAL;
}

bool IsDSDRate(uint32_t sample_rate)
{
    switch(sample_rate)
    {
        case 2822400:
        case 5644800:
        case 11289600:
            return true;
    }
    return false;
}

enum eAudioMode GetAudioModeFromRate(uint32_t sample_rate)
{
    switch(sample_rate)
    {
        case 2822400:
            return AM_DSD64;
        case 5644800:
            return AM_DSD128;
        case 11289600:
            return AM_DSD256;
        default:
            return AM_PCM;
    }
}

void init_alsa_callbacks(struct TManager* self)
{
    self->m_alsa_callbacks.register_alsa_driver = &attach_alsa_driver;
    self->m_alsa_callbacks.unregister_alsa_driver = &detach_alsa_driver;
    self->m_alsa_callbacks.get_input_jitter_buffer_offset = &get_input_jitter_buffer_offset;
    self->m_alsa_callbacks.get_input_jitter_buffer_offset_for_pcm = &get_input_jitter_buffer_offset_for_pcm;
    self->m_alsa_callbacks.get_output_jitter_buffer_offset = &get_output_jitter_buffer_offset;
    self->m_alsa_callbacks.get_min_interrupts_frame_size = &get_min_interrupts_frame_size;
    self->m_alsa_callbacks.get_max_interrupts_frame_size = &get_max_interrupts_frame_size;
    self->m_alsa_callbacks.get_interrupts_frame_size = &get_interrupts_frame_size;
    self->m_alsa_callbacks.set_sample_rate = &set_sample_rate;
    self->m_alsa_callbacks.get_sample_rate = &get_sample_rate;
    self->m_alsa_callbacks.get_jitter_buffer_sample_bytelength = &get_jitter_buffer_sample_bytelength;
    //self->m_alsa_callbacks.set_nb_inputs = &set_nb_inputs;
    //self->m_alsa_callbacks.set_nb_outputs = &set_nb_outputs;
    self->m_alsa_callbacks.get_nb_inputs = &get_nb_inputs;
    self->m_alsa_callbacks.get_nb_outputs = &get_nb_outputs;
    self->m_alsa_callbacks.start_interrupts = &start_interrupts;
    self->m_alsa_callbacks.stop_interrupts = &stop_interrupts;
    self->m_alsa_callbacks.notify_master_volume_change = &notify_master_volume_change;
    self->m_alsa_callbacks.notify_master_switch_change = &notify_master_switch_change;
    self->m_alsa_callbacks.get_master_volume_value = &get_master_volume_value;
    self->m_alsa_callbacks.get_master_switch_value = &get_master_switch_value;
    self->m_alsa_callbacks.set_pcm_rate = &manager_set_pcm_rate;       /* W15 in-place re-rate */
    self->m_alsa_callbacks.registry_barrier = &manager_registry_barrier; /* W15 open barrier */

}
