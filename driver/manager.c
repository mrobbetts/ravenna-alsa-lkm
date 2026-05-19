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
bool init(struct TManager* self, int* errorCode)
{
    bool theAnswer = true;
    int err = 0;
    int i = 0;

    self->m_Is_NIC_Active[0] = true;
    self->m_Is_NIC_Active[1] = false;
    
    self->m_Active_PTP_NIC_Idx = 0;
    for (i = 0; i < _MAX_NICS; i++)
    {
        self->m_lastLockStatus[i] = PTPLS_UNLOCKED;
    }    

    self->m_bIsStarted = false;
    self->m_bIORunning = false;
    memset(self->m_apALSAChip, 0, sizeof(self->m_apALSAChip));
    self->m_uPCMCount = 0;
    self->m_alsa_driver_frontend = NULL;
    self->m_bIsPlaybackIO = false;
    self->m_bIsRecordingIO = false;

    memset(self->m_cInterfaceName, 0, MAX_INTERFACE_NAME);

    //  initialize the stuff tracked by the IORegistry
    self->m_SampleRate = DEFAULT_SAMPLERATE;
    SetSamplingRate(self, self->m_SampleRate);

    self->m_TICFrameSizeAt1FS = DEFAULT_NADAC_TICFRAMESIZE;
    self->m_MaxFrameSize = MAX_HORUS_SUPPORTED_FRAMESIZE_IN_SAMPLES;
    SetTICFrameSizeAt1FS(self, self->m_TICFrameSizeAt1FS);

    self->m_RingBufferFrameSize = RINGBUFFERSIZE;

    self->m_nPlayoutDelay = 0;
    self->m_nCaptureDelay = 0;

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
        if (!init_ptp(&self->m_PTP[i], &self->m_EthernetFilter[i], &self->m_c_audio_streamer_clock_PTP_callback))
        {
            DebugMsg("CManager::init: self->m_PTP.Init() failed");
            theAnswer = false;
            err = -EINVAL;
            goto Failure;
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
    for (i = 0; i < _MAX_NICS; i++)
    {
        destroy_ptp(&self->m_PTP[i]);
    }
    for (i = 0; i < _MAX_NICS; i++)
    {
        DestroyEtherTube(&self->m_EthernetFilter[i]);
    }
}

//////////////////////////////////////////////////////////////////////////////////
bool start(struct TManager* self)
{
    int i = 0;
    for (i = 0; i < _MAX_NICS; i++)
    {
        if (self->m_Is_NIC_Active[i])
        {
            StartAudioFrameTICTimer(&self->m_PTP[i], get_frame_size(self), (IsDSDRate(self->m_SampleRate) ? 352800 : self->m_SampleRate));
        }
    }
    start_clock_timer();
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
        stopIO(self, false);
        stopIO(self, true);
    }

    int i;
    for (i = 0; i < _MAX_NICS; i++)
    {
        EnableEtherTube(&self->m_EthernetFilter[i], 0);
    }

    stop_clock_timer();
    
    for (i = 0; i < _MAX_NICS; i++)
    {
        StopAudioFrameTICTimer(&self->m_PTP[i]);
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
    /* Acquire-load the frontend pointer first so the function-pointer
     * struct is fully visible before we dereference it through the chips
     * loop. Pairs with smp_store_release in attach_alsa_driver. */
    const struct ravenna_mgr_ops *frontend = smp_load_acquire(&self->m_alsa_driver_frontend);
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
}

bool startIO(struct TManager* self, bool is_playback)
{
    if(!self->m_bIsStarted)
        return false;

    MTAL_DP("MergingRAVENNAAudioDriver::startIO\n");

    if (!is_playback) {
        printk(KERN_DEBUG "starting capture I/O\n");
        MuteInputBuffer(self);
    }
    else {
        printk(KERN_DEBUG "starting playback I/O\n");
        MuteOutputBuffer(self);
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
bool stopIO(struct TManager* self, bool is_playback)
{
    MTAL_DP("MergingRAVENNAAudioDriver::stopIO\n");

    if (!is_playback) {
        printk(KERN_DEBUG "stopping capture I/O\n");
        /* Stage 1 caveat: MuteInputBuffer/MuteOutputBuffer mute chip 0's
         * buffer only. With multiple capturing PCMs the per-chip buffers
         * for chips 1..N-1 don't get muted on a stopIO; benign because
         * the trigger STOP path on each chip will also halt that chip's
         * RTP source consumption. True per-chip mute requires per-PCM
         * channel counts in kernel storage (Stage 2). */
        MuteInputBuffer(self);
    } else {
        printk(KERN_DEBUG "stopping playback I/O\n");
        MuteOutputBuffer(self);
    }

    recompute_global_io_flags(self);

    return true;
}

//////////////////////////////////////////////////////////////////////////////////
void UpdateFrameSize(struct TManager* self)
{
    uint32_t ui32nFS;
    if(IsDSDRate(self->m_SampleRate))
    {
        ui32nFS = 8;
    }
    else
    {
        switch(self->m_SampleRate)
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

    self->m_ui32FrameSize = (uint32_t)self->m_TICFrameSizeAt1FS * ui32nFS;

    if(self->m_ui32FrameSize > self->m_MaxFrameSize)
        self->m_ui32FrameSize = self->m_MaxFrameSize;

    MTAL_DP("CManager::UpdateFrameSize() new TIC Frame Size = %u\n", self->m_ui32FrameSize);
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
    int i;
    uint64_t nbloop = 0;
    //MTAL_DP("CManager::SetSamplingRate from %u to %u\n", self->m_SampleRate, samplingRate);

    if(self->m_SampleRate == samplingRate)
        return true;

    if(self->m_bIORunning)
    {
        MTAL_DP("CManager::SetSamplingRate(%u) not allowed when IO are running\n", samplingRate);
        return false; // not allowed. stop IO first
    }


    self->m_SampleRate = samplingRate;
    UpdateFrameSize(self);
    MuteOutputBuffer(self);

    if(self->m_bIsStarted)
    {
        for (i = 0; i < _MAX_NICS; i++)
        {
            if (self->m_Is_NIC_Active[i])
            {
                StartAudioFrameTICTimer(&self->m_PTP[i], get_frame_size(self), (IsDSDRate(self->m_SampleRate) ? 352800 : self->m_SampleRate));
            }
        }
        start_clock_timer();
        do
        {
            CW_msleep_interruptible(1);
            if(++nbloop >= 4000)
            {
                MTAL_DP("CManager::SetSamplingRate PTP lock timed out\n");
                return false;
            }
        }
        while (GetLockStatus(&self->m_PTP[0]) != PTPLS_LOCKED || GetLockStatus(&self->m_PTP[1]) != PTPLS_LOCKED);
        //MTAL_DP("CManager::SetSamplingRate(%u) Completed\n", samplingRate);
    }

    return true;
}

//////////////////////////////////////////////////////////////////////////////////
bool SetDSDSamplingRate(struct TManager* self, uint32_t samplingRate)
{
    int i = 0;
    uint64_t nbloop = 0;
    MTAL_DP("CManager::SetDSDSamplingRate(%u)\n", samplingRate);

    if(self->m_SampleRate == samplingRate)
        return true;

    if(self->m_bIORunning)
    {
        MTAL_DP("CManager::SetDSDSamplingRate(%u) not allowed when IO are running\n", samplingRate);
        return false; // not allowed. stop IO first
    }


    self->m_SampleRate = samplingRate;
    UpdateFrameSize(self);
    MuteOutputBuffer(self);

    if(self->m_bIsStarted)
    {
        for (i = 0; i < _MAX_NICS; i++)
        {
            if (self->m_Is_NIC_Active[i])
            {
                StartAudioFrameTICTimer(&self->m_PTP[i], get_frame_size(self), (IsDSDRate(self->m_SampleRate) ? 352800 : self->m_SampleRate));
            }
        }
        start_clock_timer();
        do
        {
            CW_msleep_interruptible(1);
            if(++nbloop >= 4000)
            {
                MTAL_DP("CManager::SetSamplingRate PTP lock timed out\n");
                return false;
            }
        }
        while (GetLockStatus(&self->m_PTP[0]) != PTPLS_LOCKED || GetLockStatus(&self->m_PTP[1]) != PTPLS_LOCKED);
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
TClock_PTP* GetPTP(struct TManager* self, unsigned short ptp_idx)
{
    if (ptp_idx < _MAX_NICS)
    {
        return &self->m_PTP[ptp_idx];
    }
    return NULL;
}

//////////////////////////////////////////////////////////////////////////////////
void Select_PTP_NIC(struct TManager* self)
{
    int i = 0;
    for (i = 0; i < _MAX_NICS; i++)
    {
        EPTPLockStatus lockStatus = GetLockStatus(&self->m_PTP[i]);
        if (self->m_lastLockStatus[i] != lockStatus)
        {
            printk("PTP lock [%d] status changed from %d to %d\n", i, self->m_lastLockStatus[i], lockStatus);
            self->m_lastLockStatus[i] = lockStatus;
        }
    }

    if (self->m_lastLockStatus[0] == PTPLS_LOCKED && GetPTPPriority(&self->m_PTP[0]) <= GetPTPPriority(&self->m_PTP[1]))
    {
        self->m_Active_PTP_NIC_Idx = 0;
    }
    else if (self->m_lastLockStatus[1] == PTPLS_LOCKED)
    {
        self->m_Active_PTP_NIC_Idx = 1;
    }
    else
    {
        self->m_Active_PTP_NIC_Idx = 0;
    }
}

//////////////////////////////////////////////////////////////////////////////////
unsigned short GetSelected_PTP_NIC(struct TManager* self)
{
    return self->m_Active_PTP_NIC_Idx;
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
            /* Payload (Stage 1+): int32_t pcm_id. Streams ARE tagged with
             * m_uiPCMId now (see RTP_stream_info.h), but the kernel-side
             * stream container doesn't yet have a per-pcm_id iteration
             * helper, so this handler still wipes all streams regardless
             * of pcm_id. Per-pcm_id reset is a Stage 2/3 follow-up. */
            MTAL_DP("CManager::OnNewMessage MT_ALSA_Msg_Reset..\n");
            if (msg_rcv->dataSize != sizeof(int32_t))
            {
                MTAL_DP_ERR("MT_ALSA_Msg_Reset invalid data size (got %d, expected %zu)\n",
                            msg_rcv->dataSize, sizeof(int32_t));
                msg_reply.errCode = -315;
            }
            else
            {
                remove_all_RTP_streams(&self->m_RTP_streams_manager);
                msg_reply.errCode = 0;
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
                if (add_RTP_stream_(&self->m_RTP_streams_manager, rtp_stream_info_ptr, &stream_handle))
                {
                    MTAL_DP_INFO("self->m_RTP_streams_manager stream_handle = %llu\n", stream_handle);

                    msg_reply.errCode = 0;
                    msg_reply.dataSize = sizeof(uint64_t);
                    msg_reply.data = &stream_handle;
                    CW_netlink_send_reply_to_user_land(&msg_reply);
                    return; // because stream_handle is outof the scope if send reply at the end of the function
                }
                msg_reply.errCode = -401;
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
                if (!remove_RTP_stream_(&self->m_RTP_streams_manager, *rtp_stream_handle_ptr))
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
                for (i = 0; i < _MAX_NICS; i++)
                {
                    SetPTPConfig(&self->m_PTP[i], ptpConfig);
                }
                msg_reply.errCode = 0;
            }
            break;
        }
        case MT_ALSA_Msg_GetPTPConfig:
        {
            //MTAL_DP_INFO("Get PTP Config\n");

            TPTPConfig ptpConfig;
            GetPTPConfig(&self->m_PTP[self->m_Active_PTP_NIC_Idx], &ptpConfig);

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
             GetPTPStatus(&self->m_PTP[self->m_Active_PTP_NIC_Idx], &ptpStatus);

            msg_reply.errCode = 0;
            msg_reply.dataSize = sizeof(TPTPStatus);
            msg_reply.data = &ptpStatus;

            CW_netlink_send_reply_to_user_land(&msg_reply);
            return; // because ptpStatus is out of the scope if send reply at the end of the function
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
            /* Payload (Stage 1+): {int32_t pcm_id, int32_t delay_in_samples}.
             * Stage 1 stores delay manager-wide and refuses non-zero
             * pcm_id, so the per-PCM gap is visible to the daemon (which
             * already restricts itself to pcm_id=0 in init). Per-PCM
             * delay storage lands in Stage 2 when m_nPlayoutDelay moves
             * onto the chip. */
            if (msg_rcv->dataSize != sizeof(int32_t) * 2)
            {
                MTAL_DP_ERR("MT_ALSA_Msg_SetPlayoutDelay invalid data size\n");
                msg_reply.errCode = -315;
            }
            else
            {
                int32_t pcm_id = *(int32_t*)msg_rcv->data;
                int32_t delay  = *(int32_t*)((char*)msg_rcv->data + sizeof(int32_t));
                if (pcm_id != 0)
                {
                    MTAL_DP_ERR("MT_ALSA_Msg_SetPlayoutDelay: per-pcm_id (%d) not supported in Stage 1\n", pcm_id);
                    msg_reply.errCode = -EINVAL;
                }
                else
                {
                    self->m_nPlayoutDelay = delay;
                    MTAL_DP_INFO("MT_ALSA_Msg_SetPlayoutDelay pcm_id=0 set to %d\n",
                                 self->m_nPlayoutDelay);
                    msg_reply.errCode = 0;
                }
            }
            break;
        case MT_ALSA_Msg_SetCaptureDelay:
            if (msg_rcv->dataSize != sizeof(int32_t) * 2)
            {
                MTAL_DP_ERR("MT_ALSA_Msg_SetCaptureDelay invalid data size\n");
                msg_reply.errCode = -315;
            }
            else
            {
                int32_t pcm_id = *(int32_t*)msg_rcv->data;
                int32_t delay  = *(int32_t*)((char*)msg_rcv->data + sizeof(int32_t));
                if (pcm_id != 0)
                {
                    MTAL_DP_ERR("MT_ALSA_Msg_SetCaptureDelay: per-pcm_id (%d) not supported in Stage 1\n", pcm_id);
                    msg_reply.errCode = -EINVAL;
                }
                else
                {
                    self->m_nCaptureDelay = delay;
                    MTAL_DP_INFO("MT_ALSA_Msg_SetCaptureDelay pcm_id=0 set to %d\n",
                                 self->m_nCaptureDelay);
                    msg_reply.errCode = 0;
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
                /* Stage 1: shared rate. Refuse mismatched per-PCM rates so
                 * the daemon learns about the Stage 2 dependency early. */
                if (args->sample_rate != 0 && args->sample_rate != self->m_SampleRate)
                {
                    MTAL_DP_ERR("MT_ALSA_Msg_AddPCM rate %u != manager rate %u (Stage 1: shared rate only)\n",
                                args->sample_rate, self->m_SampleRate);
                    msg_reply.errCode = -EINVAL;
                }
                else
                {
                    int err = mr_alsa_audio_add_pcm(args->pcm_id);
                    msg_reply.errCode = err;
                }
            }
            break;
        case MT_ALSA_Msg_RemovePCM:
            /* Stage 3 will wire this up via REST. Stage 1 declines. */
            MTAL_DP_ERR("MT_ALSA_Msg_RemovePCM not supported in Stage 1\n");
            msg_reply.errCode = -ENOSYS;
            break;
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
void MuteInputBuffer(struct TManager* self)
{
    int32_t* inputBuffer = nullptr;
    uint32_t bufferLength = self->m_alsa_driver_frontend->get_capture_buffer_size_in_frames(self->m_apALSAChip[0]);
    if(bufferLength == 0)
    {
        MTAL_DP("CManager::start() failed: ALSA capture buffer size is 0\n");
        return;
    }

    self->m_alsa_driver_frontend->lock_capture_buffer(self->m_apALSAChip[0]);
    inputBuffer = (int32_t*)(self->m_alsa_driver_frontend->get_capture_buffer(self->m_apALSAChip[0]));
    if(inputBuffer == nullptr)
    {
        MTAL_DP("CManager::start() failed: No ALSA capture buffer available\n");
        self->m_alsa_driver_frontend->unlock_capture_buffer(self->m_apALSAChip[0]);
        return;
    }

    //MTAL_DP("Input Buffer muted with 0x%x\n", get_live_in_mute_pattern(0));
    memset(inputBuffer, get_live_in_mute_pattern(self, 0), sizeof(int32_t) * bufferLength * self->m_NumberOfInputs);

    self->m_alsa_driver_frontend->unlock_capture_buffer(self->m_apALSAChip[0]);
}

//////////////////////////////////////////////////////////////////////////////////
void MuteOutputBuffer(struct TManager* self)
{
    int32_t* outputBuffer = nullptr;
    uint32_t bufferLength = self->m_alsa_driver_frontend->get_playback_buffer_size_in_frames(self->m_apALSAChip[0]);
    if(bufferLength == 0)
    {
        MTAL_DP("CManager::start() failed: ALSA playback buffer size is 0\n");
        return;
    }

    self->m_alsa_driver_frontend->lock_playback_buffer(self->m_apALSAChip[0]);
    outputBuffer = (int32_t*)(self->m_alsa_driver_frontend->get_playback_buffer(self->m_apALSAChip[0]));
    if(outputBuffer == nullptr)
    {
        MTAL_DP("CManager::start() failed: No ALSA playback buffer available\n");
        self->m_alsa_driver_frontend->unlock_playback_buffer(self->m_apALSAChip[0]);
        return;
    }

    //MTAL_DP("Output Buffer muted with 0x%x\n", get_live_out_mute_pattern(self, 0));
    memset(outputBuffer, get_live_out_mute_pattern(self, 0), sizeof(int32_t) * bufferLength * self->m_NumberOfOutputs);

    self->m_alsa_driver_frontend->unlock_playback_buffer(self->m_apALSAChip[0]);
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
            nDispatchResult = process_PTP_packet(&self->m_PTP[nicId], pUDPPacketBase, packetsize);
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
    return get_ptp_global_SAC(&self->m_PTP[self->m_Active_PTP_NIC_Idx]);
}
//////////////////////////////////////////////////////////////////////////////////
uint64_t get_global_time(void* user)
{
    struct TManager* self = (struct TManager*)user;
    return get_ptp_global_time(&self->m_PTP[self->m_Active_PTP_NIC_Idx]);
}
//////////////////////////////////////////////////////////////////////////////////
void get_global_times(void* user, uint64_t* pui64GlobalSAC, uint64_t* pui64GlobalTime, uint64_t* pui64GlobalPerformanceCounter)
{
    struct TManager* self = (struct TManager*)user;
    return get_ptp_global_times(&self->m_PTP[self->m_Active_PTP_NIC_Idx], pui64GlobalSAC, pui64GlobalTime, pui64GlobalPerformanceCounter);
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
// Caudio_streamer_clock_PTP_callback
//////////////////////////////////////////////////////////////////////////////////
void AudioFrameTIC(void* user)
{
    struct TManager* self = (struct TManager*)user;

    prepare_buffer_lives(&self->m_RTP_streams_manager);
    
    if (self->m_bIORunning && self->m_lastLockStatus[self->m_Active_PTP_NIC_Idx] == PTPLS_LOCKED)
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
            frame_process_begin(&self->m_RTP_streams_manager);
            frame_process_end(&self->m_RTP_streams_manager);
        }
        #elif defined(MT_TONE_TEST) || defined (MT_RAMP_TEST)
        {
            uint32_t ui32Offset = get_live_out_jitter_buffer_offset(self, get_global_SAC(self));
            uint32_t stepOut = get_audio_engine_sample_bytelength(self);

            frame_process_begin(&self->m_RTP_streams_manager);

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
            frame_process_begin(&self->m_RTP_streams_manager);
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
    /* Publish the slot with release semantics so the hrtimer reader sees
     * the chip pointer only after the chip itself is fully constructed. */
    smp_store_release(&self->m_apALSAChip[pcm_id], alsa_chip_pointer);
    new_count = (uint32_t)pcm_id + 1;
    if (new_count > self->m_uPCMCount)
        smp_store_release(&self->m_uPCMCount, new_count);
    return 0;
}

void detach_alsa_driver(void* user, void *alsa_chip_pointer)
{
    struct TManager* self = (struct TManager*)user;
    uint32_t i;
    if (!self || !alsa_chip_pointer)
        return;
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
        return;
    }
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
            do
            {
                CW_msleep_interruptible(1);
                if(++nbloop >= 4000)
                {
                    MTAL_DP("CManager::set_sample_rate PTP lock timed out\n");
                    return false;
                }
            }
            while (GetLockStatus(&self->m_PTP[0]) != PTPLS_LOCKED || GetLockStatus(&self->m_PTP[1]) != PTPLS_LOCKED);
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

int get_playout_delay(void* user, snd_pcm_sframes_t *delay_in_sample)
{
    struct TManager* self = (struct TManager*)user;
    if (delay_in_sample)
    {
        *delay_in_sample = self->m_nPlayoutDelay;
        return 0;
    }
    return -EINVAL;
}

int get_capture_delay(void* user, snd_pcm_sframes_t *delay_in_sample)
{
    struct TManager* self = (struct TManager*)user;
    if (delay_in_sample)
    {
        *delay_in_sample = self->m_nCaptureDelay;
        return 0;
    }
    return -EINVAL;
}

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
    if(startIO(self, is_playback)) {
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
    if (stopIO(self, is_playback)) {
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
    self->m_alsa_callbacks.get_playout_delay = &get_playout_delay;
    self->m_alsa_callbacks.get_capture_delay = &get_capture_delay;
    self->m_alsa_callbacks.start_interrupts = &start_interrupts;
    self->m_alsa_callbacks.stop_interrupts = &stop_interrupts;
    self->m_alsa_callbacks.notify_master_volume_change = &notify_master_volume_change;
    self->m_alsa_callbacks.notify_master_switch_change = &notify_master_switch_change;
    self->m_alsa_callbacks.get_master_volume_value = &get_master_volume_value;
    self->m_alsa_callbacks.get_master_switch_value = &get_master_switch_value;

}
