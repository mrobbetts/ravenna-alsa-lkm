/****************************************************************************
*
*  Module Name    : audio_driver.h
*  Version        : 
*
*  Abstract       : RAVENNA/AES67 ALSA LKM
*
*  Written by     : Beguec Frederic
*  Date           : 27/04/2016
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

#if !defined(__audio_driver_h__)
#define __audio_driver_h__

#include <sound/asound.h>

#if defined(__cplusplus)
extern "C"
{
#endif // defined(__cplusplus)

/// Put functions to be called by Manager to ALSA driver
struct ravenna_mgr_ops
{
    void* (*get_playback_buffer)(void *mr_alsa_audio_chip);/// returns pointer to the playback (output) Ravenna Ring Buffer
    uint32_t (*get_playback_buffer_size_in_frames)(void *mr_alsa_audio_chip); /// returns the size of the playback (output) Ravenna Ring Buffer in samples (channel independent)
    void* (*get_capture_buffer)(void *mr_alsa_audio_chip); /// returns pointer to the capture (input) Ravenna Ring Buffer
    uint32_t (*get_capture_buffer_size_in_frames)(void *mr_alsa_audio_chip);/// returns the size of the capture (input) Ravenna Ring Buffer in samples (channel independent)
    void (*lock_playback_buffer)(void *mr_alsa_audio_chip);
    void (*unlock_playback_buffer)(void *mr_alsa_audio_chip);
    void (*lock_capture_buffer)(void *mr_alsa_audio_chip);
    void (*unlock_capture_buffer)(void *mr_alsa_audio_chip);
    int (*pcm_interrupt)(void *mr_alsa_audio_chip, int direction);/// direction: 0 for playback, 1 for capture. One interrupt per Ravenna TIC
    //uint32_t (*get_capture_buffer_offset)(void *mr_alsa_audio_chip);/// returns current offset in samples (channel independent) for Ravenna Ring Buffer
    uint32_t (*get_playback_buffer_offset)(void *mr_alsa_audio_chip);/// returns current offset (channel independent) in samples for Ravenna Ring Buffer
    int (*notify_master_volume_change)(void* mr_alsa_audio_chip, int direction, int32_t value); /// direction: 0 for playback, 1 for capture. value: from -99 to 0
    int (*notify_master_switch_change)(void* mr_alsa_audio_chip, int direction, int32_t value); /// direction: 0 for playback, 1 for capture. value: 0 for mute, 1 for enable
    /* multi-rate Stage 1: per-chip IO state accessors, used by the manager's tick loop */
    void (*set_io_state)(void *mr_alsa_audio_chip, bool is_playback, bool running);
    bool (*get_io_state)(void *mr_alsa_audio_chip, bool is_playback);
    /*
     * Multi-rate Stage 2: per-chip sample-rate + frame-size storage.
     * Each chip carries its own (rate, frame_size) configured at AddPCM
     * time and visible to the manager's tick path (via the
     * *_for_pcm callbacks) without going through the manager-wide
     * m_SampleRate / m_ui32FrameSize.
     *
     * Writes happen in netlink context (AddPCM, SetSamplingRate); reads
     * happen in softirq context (TIC tick, RTP stream hot paths). The
     * implementation uses smp_store_release on writes and smp_load_acquire
     * on reads so a reader never sees a torn pair (rate from one
     * generation paired with frame_size from another).
     *
     * frame_size is the (rate * nFS scaled) value; the manager owns the
     * rate->frame_size derivation and passes both fields atomically.
     */
    void (*set_pcm_sample_rate)(void *mr_alsa_audio_chip, uint32_t rate, uint32_t frame_size);
    uint32_t (*get_pcm_sample_rate)(void *mr_alsa_audio_chip);
    uint32_t (*get_pcm_frame_size)(void *mr_alsa_audio_chip);
    uint8_t (*get_pcm_domain)(void *mr_alsa_audio_chip); /* W11: the chip's card's PTP domain */
};

/// Put functions to be called by ALSA driver (C ALSA to CPP Ravenna wrapper/owner object)
struct alsa_ops
{
    /* multi-rate Stage 1: pcm_id is the per-card device index (matches the
     * snd_pcm_new device argument). Manager indexes its chip array by
     * pcm_id directly, so out-of-order group ids work. */
    int (*register_alsa_driver)(void* ravenna_peer, const struct ravenna_mgr_ops *ops, void *alsa_chip_pointer, int pcm_id);/// to be called at driver init to allow communication between driver and Ravenna context
    /* multi-rate Stage 1: clear the manager's chip-array slot for this
     * chip. Called from mr_alsa_audio_add_pcm's failure path so a failed
     * AddPCM doesn't poison the slot with a soon-to-be-freed chip
     * pointer. Stage 3 will also call this from a future RemovePCM path. */
    void (*unregister_alsa_driver)(void* ravenna_peer, void *alsa_chip_pointer);
    int (*get_input_jitter_buffer_offset)(void* ravenna_peer, uint32_t *offset);
    /* 2026-06-09 review fix: per-PCM variant — capture prepare on chip N
     * must align to chip N's ring length and SAC, not chip 0's. pcm_id is
     * the per-card device index (chip->pcm->device). */
    int (*get_input_jitter_buffer_offset_for_pcm)(void* ravenna_peer, uint32_t pcm_id, uint32_t *offset);
    int (*get_output_jitter_buffer_offset)(void* ravenna_peer, uint32_t *offset);
    int (*get_min_interrupts_frame_size)(void* ravenna_peer, uint32_t *framesize); /// returns min Ravenna Frame Size in samples (channel independent)
    int (*get_max_interrupts_frame_size)(void* ravenna_peer, uint32_t *framesize); /// returns max Ravenna Frame Size (hardware dependent) in samples (channel independent)
    int (*get_interrupts_frame_size)(void* ravenna_peer, uint32_t *framesize); /// returns current Ravenna Frame Size in samples (channel independent)
    int (*set_sample_rate)(void* ravenna_peer, uint32_t rate);  /// rate: use PCM rates values or raw DSD sample rates values. stop_interrupts() should be called prior sample rate changes.
                                                                ///  this function is not atomic and caller must be schedulable
    int (*get_sample_rate)(void* ravenna_peer, uint32_t *rate); /// returns current Ravenna sample rate (actual PCM rate or actual DSD rate)
    int (*get_jitter_buffer_sample_bytelength)(void* ravenna_peer, char *byte_len); /// returns current Ravenna sample rate (actual PCM rate or actual DSD rate)
    int (*get_nb_inputs)(void* ravenna_peer, uint32_t *nb_channels);
    int (*get_nb_outputs)(void* ravenna_peer, uint32_t *nb_channels);
    /* W9 #14: playout/capture delay is now a per-chip property (see
     * mr_alsa_audio_set_{playout,capture}_delay), read directly at prepare();
     * the former manager-callback round-trip (get_*_delay) is gone. */
    int (*set_jitter_buffer_depth)(void* ravenna_peer, uint32_t depth_in_frames); /// must not sleep (called under spinlock)
    int (*start_interrupts)(void* ravenna_peer, void *mr_alsa_audio_chip, bool is_playback); /// starts IO on the given chip
    int (*stop_interrupts)(void* ravenna_peer, void *mr_alsa_audio_chip, bool is_playback); /// stops IO on the given chip

    int (*notify_master_volume_change)(void* ravenna_peer, int direction, int32_t value); /// direction: 0 for playback, 1 for capture. value: from -99 to 0
    int (*notify_master_switch_change)(void* ravenna_peer, int direction, int32_t value); /// direction: 0 for playback, 1 for capture. value: 0 for mute, 1 for enable
    int (*get_master_volume_value)(void* ravenna_peer, int direction, int32_t* value); /// direction: 0 for playback, 1 for capture. value: from -99 to 0
    int (*get_master_switch_value)(void* ravenna_peer, int direction, int32_t* value); /// direction: 0 for playback, 1 for capture. value: 0 for mute, 1
};

/// Put ALSA driver functions which needs to be used by CPP code here:
extern int mr_alsa_audio_card_init(void* ravennaPeer, struct alsa_ops *callbacks);
extern void mr_alsa_audio_card_exit(void);
/* Extra PCMs created via mr_alsa_audio_add_pcm have ids
 * 1..MR_ALSA_MAX_EXTRA_PCMS. Must equal MAX_PCMS - 1 (manager.h) —
 * enforced by a _Static_assert next to MAX_PCMS so the two can never
 * drift again (the 8→16 bump originally missed this constant, capping
 * AddPCM at id 7 while everything else accepted 15). */
#define MR_ALSA_MAX_EXTRA_PCMS 15
/* W10 multi-card: the module owns up to MR_ALSA_MAX_CARDS independent ALSA
 * cards (~ one per logical device / PTP clock domain), created/destroyed live.
 * Each card costs one system-wide SNDRV_CARDS slot (cap 32). The GLOBAL pcm_id
 * space (manager m_apALSAChip[]) stays MAX_PCMS, shared across all cards. */
#define MR_ALSA_MAX_CARDS 4
/* add_pcm_to_card's sample_rate is the per-chip rate (W2/W5). Pass a non-zero
 * value from the supported rate set to lock this chip at that rate; pass 0 to
 * inherit the manager-wide rate. The chip's (pcm_sample_rate, pcm_frame_size)
 * pair is published before register_alsa_driver runs, so the moment the
 * manager's chip slot becomes visible to tick-path readers, the chip's rate is
 * too. The chip auto-attaches into the manager via the register_alsa_driver
 * callback (attach_alsa_driver stores it at m_apALSAChip[global_pcm_id]). */
/* W10 multi-card lifecycle. The daemon drives bringup/teardown as:
 *   add_card(handle, id, domain)                    -> snd_card_new (UNregistered)
 *   add_pcm_to_card(handle, global_pcm_id, rate, name) x N   (deferred-register)
 *   register_card(handle)                           -> snd_card_register (all PCMs visible)
 * and remove_card(handle) -> snd_card_disconnect + snd_card_free.
 * card_handle is a daemon-assigned index in [0, MR_ALSA_MAX_CARDS); global_pcm_id
 * is the manager's m_apALSAChip[] slot (distinct from the per-card ALSA device
 * index). All return 0 or a negative errno. */
extern int mr_alsa_audio_add_card(int card_handle, const char *id, uint8_t domain);
extern int mr_alsa_audio_add_pcm_to_card(int card_handle, int global_pcm_id, uint32_t sample_rate, const char *name);
extern int mr_alsa_audio_register_card(int card_handle);
extern int mr_alsa_audio_remove_card(int card_handle);
/* W9 #14: set a chip's advisory ALSA latency (frames). chip_ptr is the
 * manager's m_apALSAChip[] slot (== struct mr_alsa_audio_chip*). 0 / negative
 * errno; takes effect at the next prepare(). */
extern int mr_alsa_audio_set_playout_delay(void *chip_ptr, int32_t delay);
extern int mr_alsa_audio_set_capture_delay(void *chip_ptr, int32_t delay);
/* Tear down ALL cards (clean slate). Called at module unload and from the
 * manager's full Reset (MT_ALSA_Msg_Reset, pcm_id < 0) so a restarting daemon
 * redeclares onto an empty module. Safe no-op when no cards exist. */
extern void mr_alsa_audio_remove_all_cards(void);

#if	defined(__cplusplus)
}
#endif // defined(__cplusplus)

#endif // __audio_driver_h__
