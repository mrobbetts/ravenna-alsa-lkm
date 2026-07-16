/****************************************************************************
*
*  Module Name    : MT_ALSA_message_defs.h
*  Version        : 
*
*  Abstract       : RAVENNA/AES67 ALSA LKM
*
*  Written by     : Baume Florian
*  Date           : 24/03/2016
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

#ifndef MT_ALSA_MESSAGE_DEFINES_INCLUDED
#define MT_ALSA_MESSAGE_DEFINES_INCLUDED

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#define NETLINK_U2K_ID 31
#define NETLINK_K2U_ID 29

#define MAX_PAYLOAD 1024

/*
 * Multi-PCM payload convention (multi-rate project, Stage 1):
 *
 * The driver supports N ALSA PCM devices (`hw:RAVENNA,0..N-1`). For messages
 * that act on a specific PCM, the payload is prefixed with `int32_t pcm_id`
 * followed by the legacy payload. PCM 0 always exists (created at module
 * load). Additional PCMs are created on demand via MT_ALSA_Msg_AddPCM.
 *
 * Per-PCM messages (payload begins with int32_t pcm_id):
 *   MT_ALSA_Msg_Reset, MT_ALSA_Msg_SetNumberOfInputs,
 *   MT_ALSA_Msg_SetNumberOfOutputs, MT_ALSA_Msg_GetNumberOfInputs,
 *   MT_ALSA_Msg_GetNumberOfOutputs, MT_ALSA_Msg_SetPlayoutDelay,
 *   MT_ALSA_Msg_SetCaptureDelay, MT_ALSA_Msg_Add_RTPStream.
 *
 * Manager-wide messages (no pcm_id prefix): everything else, including
 * Start/Stop, SetSampleRate (Stage 1: shared rate), PTP/NIC config,
 * Remove_RTPStream and GetRTPStreamStatus (handle is globally unique),
 * and master volume/switch (Stage 1: shared, registered on PCM 0).
 */
enum MT_ALSA_msg_id
{

    MT_ALSA_Msg_Start,                    //    U2K No arguments
    MT_ALSA_Msg_Stop,                     //    U2K No arguments
    MT_ALSA_Msg_Reset,                    //    U2K One input: int32_t pcm_id
    MT_ALSA_Msg_StartIO,                  //    U2K No arguments
    MT_ALSA_Msg_StopIO,                   //    U2K No arguments
    MT_ALSA_Msg_SetSampleRate,            //    U2K K2U One input: the new sample rate as a 32 bit integer
    MT_ALSA_Msg_GetSampleRate,            //    U2K One input: the new sample rate as a 32 bit integer
    MT_ALSA_Msg_GetAudioMode,             //    U2K One input: the audio mode as a 32 bit integer
    MT_ALSA_Msg_SetDSDAudioMode,          //    U2K One input: the new DSD audio mode as a 32 bit integer
    MT_ALSA_Msg_SetTICFrameSizeAt1FS,     //    U2K One input: the new 1 Fs TIC Frame size as a 64 bit integer
    MT_ALSA_Msg_SetMaxTICFrameSize,       //    U2K One input: the new Max TIC Frame size as a 64 bit integer
    MT_ALSA_Msg_SetNumberOfInputs,        //    U2K Inputs: int32_t pcm_id, uint32_t number of inputs
    MT_ALSA_Msg_SetNumberOfOutputs,       //    U2K Inputs: int32_t pcm_id, uint32_t number of outputs
    MT_ALSA_Msg_GetNumberOfInputs,        //    U2K Input: int32_t pcm_id. Output: uint32_t number of inputs
    MT_ALSA_Msg_GetNumberOfOutputs,       //    U2K Input: int32_t pcm_id. Output: uint32_t number of outputs
    MT_ALSA_Msg_SetInterfaceName,         //    U2K One input: Struct_SetInterfaceName
    MT_ALSA_Msg_Add_RTPStream,            //    U2K Inputs: int32_t pcm_id, RTPStreamInfo. Output: hHandle
    MT_ALSA_Msg_Remove_RTPStream,         //    U2K One input: hHandle
    MT_ALSA_Msg_Update_RTPStream_Name,    //    U2K One input: CRTP_stream_update_name
    MT_ALSA_Msg_GetPTPInfo,               //    U2K One output: TPTPInfo (obsolete)
    MT_ALSA_Msg_Hello,                    //    U2K K2U No arguments
    MT_ALSA_Msg_Bye,                      //    U2K K2U No arguments
    MT_ALSA_Msg_Ping,                     //    U2K K2U No arguments
    MT_ALSA_Msg_SetMasterOutputVolume,    //    U2K K2U NADAC only : one input: int32_t value (-99 to 0)
    MT_ALSA_Msg_SetMasterOutputSwitch,    //    U2K K2U NADAC only : one input: int32_t value (0 or 1)
    MT_ALSA_Msg_GetMasterOutputVolume,    //    K2U NADAC only : one output: int32_t value (-99 to 0)
    MT_ALSA_Msg_GetMasterOutputSwitch,    //    K2U NADAC only : one output: int32_t value (0 or 1)
    MT_ALSA_Msg_SetPlayoutDelay,          //    U2K Inputs: int32_t pcm_id, int32_t delay in samples
    MT_ALSA_Msg_SetCaptureDelay,          //    U2K Inputs: int32_t pcm_id, int32_t delay in samples
    MT_ALSA_Msg_GetRTPStreamStatus,       //    U2K One input: hHandle, one output: the RTP stream status struct
    MT_ALSA_Msg_SetPTPConfig,             //    U2K One input: TPTPConfig
    MT_ALSA_Msg_GetPTPConfig,             //    U2K One output: TPTPConfig
    MT_ALSA_Msg_GetPTPStatus,             //    U2K One output: TPTPStatus
    /* multi-rate Stage 1 additions; appended to preserve wire-protocol values */
    MT_ALSA_Msg_AddPCM,                   //    U2K Input: struct MT_ALSA_AddPCM_args. Adds a PCM device to a card
    MT_ALSA_Msg_RemovePCM,                //    U2K Input: int32_t pcm_id (legacy; superseded by RemoveCard)
    /* W10 multi-card: cards are created/destroyed live (snd-usb-audio style).
     * AddCard makes an UNregistered snd_card; AddPCM adds its device(s);
     * RegisterCard commits (snd_card_register) so all of a card's PCMs appear
     * together (PA/PW enumerate a card's PCMs once); RemoveCard tears one card
     * down. Appended to preserve existing wire-protocol values. */
    MT_ALSA_Msg_AddCard,                  //    U2K Input: struct MT_ALSA_AddCard_args
    MT_ALSA_Msg_RegisterCard,             //    U2K Input: int32_t card_handle
    MT_ALSA_Msg_RemoveCard,               //    U2K Input: int32_t card_handle
    /* #22: per-PCM TIC-engine status (the media-clock lock). Appended to
     * preserve wire-protocol values. */
    MT_ALSA_Msg_GetPCMStatus,             //    U2K Input: int32_t pcm_id, output: struct TPCMStatus
    /* W15: in-place per-PCM re-rate (the alternative to recreate-card). Inputs:
     * int32_t pcm_id, uint32_t sample_rate. If the chip is idle the kernel
     * re-keys it to the new rate immediately; if a client holds it open the
     * kernel ARMS the chip (advertises the target on the "PCM Rate" kcontrol +
     * notifies) and returns -EBUSY, applying the re-rate when the chip next goes
     * idle (last close). Appended to preserve wire-protocol values. */
    MT_ALSA_Msg_SetPCMRate,               //    U2K Inputs: int32_t pcm_id, uint32_t sample_rate
    /* W15: K2U notification that an ARMED in-place re-rate has now been applied
     * autonomously (the holding client closed the PCM, so close() fired the
     * latch). Lets the daemon re-attach the sink at the new rate the instant it
     * happens, instead of polling. Inputs: int32_t pcm_id, uint32_t sample_rate.
     * Appended to preserve wire-protocol values. */
    MT_ALSA_Msg_PCMRateApplied,           //    K2U Inputs: int32_t pcm_id, uint32_t sample_rate
    /* W28 (intent-in/truth-out): retract an ARMED in-place re-rate. The latch is
     * cleared (chip stays at its live rate; nothing is applied) so a re-rate
     * issued in error — or made stale by a source that reverted before the client
     * released the device — never fires later. Idempotent (no-op if not armed).
     * Input: int32_t pcm_id. Appended to preserve wire-protocol values. */
    MT_ALSA_Msg_CancelPCMRate             //    U2K Input: int32_t pcm_id
};

/*
 * Argument struct for MT_ALSA_Msg_AddPCM.
 * sample_rate is the per-PCM rate (W2/W5; honoured per-chip since W5).
 * W7: `name` is the ALSA device name for this PCM (aplay -l); empty ⇒ the
 * kernel default (CARD_NAME). Fixed-size to keep the struct POD and the
 * daemon/kernel sizeof() size-check valid across the netlink boundary.
 */
#define MT_ALSA_PCM_NAME_MAXLEN 32
struct MT_ALSA_AddPCM_args
{
    int32_t  card_handle;   /* W10 multi-card: which card this PCM device joins */
    int32_t  pcm_id;        /* the GLOBAL pcm_id (manager m_apALSAChip[] slot) */
    uint32_t sample_rate;
    uint32_t num_inputs;
    uint32_t num_outputs;
    char     name[MT_ALSA_PCM_NAME_MAXLEN];
};

/* W10 multi-card: argument struct for MT_ALSA_Msg_AddCard. Creates an
 * UNregistered snd_card; `id` becomes the ALSA card id (hw:<id>); `domain` is
 * the card's PTP clock domain (pinned 0 until W11). The PCMs are added with
 * MT_ALSA_Msg_AddPCM (carrying this card_handle), then MT_ALSA_Msg_RegisterCard
 * commits the card. */
struct MT_ALSA_AddCard_args
{
    int32_t  card_handle;   /* daemon-assigned card index [0, MR_ALSA_MAX_CARDS) */
    uint8_t  domain;
    char     id[MT_ALSA_PCM_NAME_MAXLEN];
};

/* #22: per-PCM clock status (output of MT_ALSA_Msg_GetPCMStatus) — the TIC
 * engine's composite lock on the chip's active NIC: is this PCM's media clock
 * actually tracking? (Jitter/scheduling metrics are a later addition.) */
struct TPCMStatus
{
    int32_t nTICLockStatus;   /* EPTPLockStatus: 0 unlocked / 1 locking / 2 locked */
    /* W28 (intent-in/truth-out): the kernel is the source of truth for the live
     * chip rate. live_rate = the rate the chip is actually keyed to right now;
     * pending_rate = an ARMED in-place re-rate target (0 = not armed). The daemon
     * reads these rather than trusting its own cached/commanded rate. Appended to
     * preserve wire layout — older readers that stop at nTICLockStatus still work. */
    uint32_t live_rate;
    uint32_t pending_rate;
    /* W16 slice 3 (appended): EClockState — the media-clock state WITH the
     * reason for unlockedness: 0 stopped / 1 no-signal / 2 acquiring / 3 locked
     * / 4 saturated (GM beyond steering range, untrackable). Supersedes
     * nTICLockStatus (kept for wire compat), which cannot distinguish
     * acquiring from saturated. NOTE (slice 3b): clock-source health is a
     * domain-level fact — the DOMAIN composite in TPTPStatus is what UIs
     * should colour by; this per-PCM copy is the engine's own state, useful
     * as detail (tooltips, per-rate acquiring transients). */
    int32_t clock_state;
    /* W16 slice 3b (appended): engine-local EXECUTION health — is this PCM's
     * tick engine meeting its own schedule? Meaningful regardless of clock
     * state (a free-wheeling engine still ticks and services clients, W17).
     * tick_period_us = the engine's nominal tick period; us_since_last_tick =
     * elapsed since the last hrtimer tick ran. ticking ⇔ since ≲ a few
     * periods. Both 0 when the engine is stopped. */
    uint32_t tick_period_us;
    uint32_t us_since_last_tick;
};

struct MT_ALSA_msg
{
    enum MT_ALSA_msg_id id;
    int errCode;
    int dataSize;
    void* data;
    // data are right here when sending through netlink
};

#endif // MT_ALSA_MESSAGE_DEFINES_INCLUDED
