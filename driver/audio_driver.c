/****************************************************************************
*
*  Module Name    : audio_driver.c
*  Version        :
*
*  Abstract       : RAVENNA/AES67 ALSA LKM
*
*  Written by     : Beguec Frederic
*  Date           : 27/04/2016
*  Modified by    : Baume Florian
*  Date           : 02/2018
*  Modification   : Added capture capabilities
*  Modified by    : Baume Florian
*  Date           : 06/2019
*  Modification   : Added mmap support
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

#include <linux/version.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/atomic.h>
#include <linux/compiler.h>  /* READ_ONCE / WRITE_ONCE */

#include <sound/core.h>
#include <sound/control.h>
#include <sound/tlv.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>

#include "../common/MergingRAVENNACommon.h"
#include "MTConvert.h"
#include "audio_driver.h"
#include "module_timer.h"


#define SND_MR_ALSA_AUDIO_DRIVER    "snd_merging_rav"
#define CARD_NAME "Merging RAVENNA"
#define MR_ALSA_AUDIO_PM_OPS    NULL // TODO to be changed for implementing Power Management

#define MR_ALSA_RINGBUFFER_NB_FRAMES (RINGBUFFERSIZE) // multiple of 48 * 16 and multiple of 64 * 16 //32768. Note * 16 because up to 16FS
#define MR_ALSA_NB_FRAMES_PER_PERIOD_AT_1FS_LEGACY (DEFAULT_NADAC_TICFRAMESIZE)
#define MR_ALSA_NB_CHANNELS_MAX (MAX_NUMBEROFINPUTS)

#define MR_ALSA_PTP_FRAME_RATE_FOR_DSD (352800)

#define MR_ALSA_SUBSTREAM_MAX 128


/// reminder:
//#define SNDRV_PCM_FORMAT_DSD_U8         ((__force snd_pcm_format_t) 48) /* DSD, 1-byte samples DSD (x8)
//#define SNDRV_PCM_FORMAT_DSD_U16_LE     ((__force snd_pcm_format_t) 49) /* DSD, 2-byte samples DSD (x16), little endian
//#define SNDRV_PCM_FORMAT_DSD_U32_LE     ((__force snd_pcm_format_t) 50) /* DSD, 4-byte samples DSD (x32), little endian
//#define SNDRV_PCM_FORMAT_DSD_U16_BE     ((__force snd_pcm_format_t) 51) /* DSD, 2-byte samples DSD (x16), big endian
//#define SNDRV_PCM_FORMAT_DSD_U32_BE     ((__force snd_pcm_format_t) 52) /* DSD, 4-byte samples DSD (x32), big endian


static int index = SNDRV_DEFAULT_IDX1; /* Index 0-max */
static char *id = SNDRV_DEFAULT_STR1; /* Id for card */
static bool enable = SNDRV_DEFAULT_ENABLE1; /* Enable this card */
static int pcm_devs = 1;

module_param(index, int, 0444);
MODULE_PARM_DESC(index, "Index value for " CARD_NAME " soundcard.");
module_param(id, charp, 0444);
MODULE_PARM_DESC(id, "ID string for " CARD_NAME " soundcard.");
module_param(enable, bool, 0444);
MODULE_PARM_DESC(enable, "Enable " CARD_NAME " soundcard.");
module_param(pcm_devs, int, 0444);
MODULE_PARM_DESC(pcm_devs, "PCM devices # (1) for Merging RAVENNA Audio driver.");

static int jitter_buffer_multiplier = 3;  /* 3x packet time */
module_param(jitter_buffer_multiplier, int, 0644);  /* writable at runtime via sysfs */
MODULE_PARM_DESC(jitter_buffer_multiplier,
    "Jitter buffer depth as multiple of TIC frame size (default: 3)");

#define SUB_ALLOC_OUT_OF_SPACE -1
#define SUB_ALLOC_ADDED 1
#define SUB_ALLOC_ALREADY_ADDED 2
#define SUB_ALLOC_REMOVED 3
#define SUB_ALLOC_NOT_FOUND 4

static struct platform_device *g_device;
static void *g_ravenna_peer;
static struct alsa_ops *g_mr_alsa_audio_ops;

/* W10 multi-card: the module owns up to MR_ALSA_MAX_CARDS independent ALSA
 * cards, created/destroyed live (snd-usb-audio style). Each card owns its chips
 * (PCM devices) and frees only its own via its private_free. The manager stays
 * card-agnostic — it indexes chips by GLOBAL pcm_id (m_apALSAChip[]); the
 * per-card device index (chips[] below, 0..N) is a separate space. */
struct mr_alsa_card {
    struct snd_card *card;
    struct mr_alsa_audio_chip *chips[MR_ALSA_MAX_EXTRA_PCMS + 1]; /* by per-card device idx */
    unsigned int chip_count;
    bool registered;
    uint8_t domain;   /* W11: the card's PTP clock domain (cards may share one) */
};
static struct mr_alsa_card g_cards[MR_ALSA_MAX_CARDS];

/* the g_cards[] slot owning a snd_card (used by the per-card private_free). */
static struct mr_alsa_card *mr_alsa_card_of(struct snd_card *card)
{
    int i;
    for (i = 0; i < MR_ALSA_MAX_CARDS; ++i)
        if (g_cards[i].card == card)
            return &g_cards[i];
    return NULL;
}


static int mr_alsa_audio_pcm_capture_copy_internal( struct snd_pcm_substream *substream,
                                            int channel, uint32_t pos,
                                            void __user *src,
                                            snd_pcm_uframes_t count);
static int mr_alsa_audio_pcm_playback_copy_internal( struct snd_pcm_substream *substream,
                                            int channel, uint32_t pos,
                                            void __user *src,
                                            snd_pcm_uframes_t count);

/* Forward declarations for optimized de-interleave functions */
static void playback_deinterleave_s32le(unsigned char *playback_buffer,
    uint32_t ring_buffer_frames, uint32_t ravenna_pos,
    const unsigned char *src, unsigned int channels, snd_pcm_uframes_t frames);
static void playback_deinterleave_s24le(unsigned char *playback_buffer,
    uint32_t ring_buffer_frames, uint32_t ravenna_pos,
    const unsigned char *src, unsigned int channels, snd_pcm_uframes_t frames);
static void playback_deinterleave_s24_3le(unsigned char *playback_buffer,
    uint32_t ring_buffer_frames, uint32_t ravenna_pos,
    const unsigned char *src, unsigned int channels, snd_pcm_uframes_t frames);
static void playback_deinterleave_s16le(unsigned char *playback_buffer,
    uint32_t ring_buffer_frames, uint32_t ravenna_pos,
    const unsigned char *src, unsigned int channels, snd_pcm_uframes_t frames);

/// "chip" : the main private structure
struct mr_alsa_audio_chip
{
    void *ravenna_peer; /// Ravenna User object (is used for any call to mr_alsa_audio_ops functions)
    struct alsa_ops *mr_alsa_audio_ops;  /// alsa to Ravenna callback functions

    //
    spinlock_t lock;

    /* only one playback and/or capture stream */
    struct snd_pcm_substream *capture_substream;
    spinlock_t capture_lock;
    struct snd_pcm_substream *playback_substream;
    spinlock_t playback_lock;

    struct platform_device *dev;

    unsigned char *capture_buffer;  /// non interleaved Ravenna Ring Buffer
    void* capture_buffer_channels_map[MR_ALSA_NB_CHANNELS_MAX]; // array of pointer to each channels of capture_buffer
    unsigned char *playback_buffer;  /// non interleaved Ravenna Ring Buffer
    uint32_t playback_buffer_pos;    /// in ravenna samples
    uint32_t capture_buffer_pos;    /// in ravenna samples
    uint64_t playback_buffer_alsa_sac;   /// in alsa frames
    uint64_t playback_buffer_rav_sac;    /// in alsa frames
    u64 current_alsa_playback_format; /// init with Ravenna Manager then retrieved from hw params
    u64 current_alsa_capture_format; /// init with Ravenna Manager then retrieved from hw params
    uint32_t current_alsa_playback_stride; /// number of data bytes per sample (ex: 3 for format = SNDRV_PCM_FORMAT_S24_3LE, 4 for format = SNDRV_PCM_FORMAT_S24_LE, 4 for format = SNDRV_PCM_FORMAT_S32_LE)
    uint32_t current_alsa_capture_stride; /// number of data bytes per sample (ex: 3 for format = SNDRV_PCM_FORMAT_S24_3LE, 4 for format = SNDRV_PCM_FORMAT_S24_LE, 4 for format = SNDRV_PCM_FORMAT_S32_LE)

    pid_t capture_pid;  /* process id which uses capture */
    pid_t playback_pid; /* process id which uses playback */
    int running;        /* running status */

    /* per-PCM IO state, consulted by the manager's AudioFrameTIC loop */
    bool playback_io;
    bool capture_io;

    /*
     * Multi-rate Stage 2: per-PCM rate + frame size, configured at AddPCM
     * time (or, for chip 0, at probe and tracked by manager-wide
     * SetSampleRate). Distinct from `current_rate` below, which reflects
     * the latest userspace hw_params request — these two will converge
     * once Stage 2 task #5 narrows hw_params to a single rate per chip,
     * but for now `pcm_sample_rate` is the source of truth for the
     * tick-path readers via the *_for_pcm callbacks.
     *
     * Writers (netlink context: AddPCM, SetSamplingRate) use
     * smp_store_release; readers (softirq context: TIC tick, RTP stream
     * hot paths) use smp_load_acquire — same publish discipline as the
     * chip-slot pointer in the manager.
     *
     * 2026-06-09 review fix (F3): rate and frame_size are PACKED into one
     * u64 (rate in the high 32 bits, frame_size in the low 32) published
     * with a single release-store, so a reader can never observe a rate
     * from one generation paired with a frame_size from another. Access
     * only via mr_alsa_audio_{set,get}_pcm_sample_rate/frame_size.
     */
    uint64_t pcm_rate_and_frame;

    /* W6: per-chip ALSA constraint state (Decision 7 — a chip advertises
     * exactly its configured rate). Filled by set_pcm_sample_rate, the
     * single rate-publish chokepoint (attach, chip-0 UpdateFrameSize,
     * W10 SetPCMRate), always with IO quiesced; consumed at pcm_open.
     * Replaces the global g_constraints_* arrays that every open of every
     * chip used to rewrite in place (cross-chip race + wrong content for
     * per-rate chips). */
    unsigned int supported_rates[3];
    struct snd_pcm_hw_constraint_list constraints_rates;
    unsigned int supported_period_sizes[4];
    struct snd_pcm_hw_constraint_list constraints_period_sizes;

    /* W7: ALSA device name from AddPCM (group name); empty ⇒ CARD_NAME.
     * Sized to MT_ALSA_PCM_NAME_MAXLEN (common/MT_ALSA_message_defs.h);
     * kept in lockstep by hand. Set before the chip is published. */
    char pcm_name[32];

    unsigned int current_rate;  /// updated on each alsa hw_params and prepare
    unsigned int current_dsd;   /// 0 for pcm, 1 for dsd64, 2 for dsd128, 4 for dsd256. updated on each alsa hw_params and prepare

    unsigned int nb_playback_interrupts_per_period; /// number of or interrupts call between 2 snd_pcm_elapsed() calls (should always be 1 in PCM)
    unsigned int current_playback_interrupt_idx;    ///from 0 to nb_interrupts_per_period - 1
    unsigned int nb_capture_interrupts_per_period;  /// number of or interrupts call between 2 snd_pcm_elapsed() calls (should always be 1 in PCM)
    unsigned int current_capture_interrupt_idx;     ///from 0 to nb_interrupts_per_period - 1

    uint32_t current_nbinputs; /// updated on each alsa hw_params and prepare
    uint32_t current_nboutputs; /// updated on each alsa hw_params and prepare

    struct snd_kcontrol *playback_volume_control;
    struct snd_kcontrol *playback_switch_control;
    int32_t current_playback_volume; /// cached value for volume control
    int32_t current_playback_switch; /// cached value for switch control

    /* W15: per-PCM read-only "current rate" control (Hz). Unlike the card-level
     * master volume/switch above (device-0-only, NADAC model), this is genuinely
     * per-PCM (scoped via id.device). It lets an ALSA capture client read the
     * chip's active rate and re-open at it after an in-place re-rate; the
     * in-place path snd_ctl_notify()s it on a live rate change. */
    struct snd_kcontrol *current_rate_control;

    /* W15: in-place re-rate latch. 0 = normal; nonzero = ARMED for this target
     * rate. While armed the chip keeps running at its live rate; close() applies
     * the re-key (re-rate to pending_rate) once the chip goes idle. The PCM Rate
     * kcontrol reports pending_rate (the target) while armed so a follower
     * reopens at it. Written in netlink/process context only (never softirq). */
    uint32_t pending_rate;

    /* W9 #14: per-PCM advisory ALSA latency (frames), latched into
     * runtime->delay at prepare() so snd_pcm_delay() reports this chip's own
     * latency. Set per-pcm_id via MT_ALSA_Msg_Set{Playout,Capture}Delay
     * (replaces the former manager-wide m_n*Delay). NOTE: advisory only — the
     * real RTP buffering depth is the per-stream link offset, not this. */
    int32_t playout_delay;
    int32_t capture_delay;

    struct snd_card *card;  /* one card */
    struct snd_pcm *pcm;    /* has one pcm */

    /* Multi-rate / W10: the manager's GLOBAL pcm_id for this chip (its slot in
     * m_apALSAChip[]). Equals the per-card ALSA device index (chip->pcm->device)
     * under single-card, but DIVERGES under multi-card (every card has a device
     * 0). The manager-facing *_for_pcm callbacks key on THIS, never on the
     * per-card device index. Set at chip_create from device_idx. */
    int global_pcm_id;

    atomic_t dma_playback_offset;
    atomic_t dma_capture_offset;

    unsigned int pcm_playback_buffer_size;
    unsigned int pcm_capture_buffer_size;

    uint8_t *dma_playback_buffer;
    uint8_t *dma_capture_buffer;

    /* Optimized playback de-interleave — set at prepare time based on format */
    void (*playback_deinterleave_fn)(unsigned char *playback_buffer,
                                     uint32_t ring_buffer_frames,
                                     uint32_t ravenna_pos,
                                     const unsigned char *src,
                                     unsigned int channels,
                                     snd_pcm_uframes_t frames);

    /*
     * Optimized capture interleave — set at prepare time based on format.
     * Signature matches MTConvert* mapped interleave functions.
     */
    int (*capture_interleave_fn)(void **channel_map,
                                 const uint32_t ravenna_pos,
                                 void *dst,
                                 const uint32_t channels,
                                 const uint32_t frames);
};


/// channel mappings (NADAC only)
// This should be removed from the open source version
static const struct snd_pcm_chmap_elem mr_alsa_audio_nadac_playback_ch_map[9] = {
    { .channels = 1,
      .map = { SNDRV_CHMAP_MONO } },
    { .channels = 2,
      .map = { SNDRV_CHMAP_FL, SNDRV_CHMAP_FR } },
    { .channels = 3,
      .map = { SNDRV_CHMAP_FL, SNDRV_CHMAP_FR, SNDRV_CHMAP_FC} },
    { .channels = 4,
      .map = { SNDRV_CHMAP_FL, SNDRV_CHMAP_FR, SNDRV_CHMAP_RL, SNDRV_CHMAP_RR} }, // quadro
    { .channels = 5,
      .map = { SNDRV_CHMAP_FL, SNDRV_CHMAP_FR, SNDRV_CHMAP_FC, SNDRV_CHMAP_RL, SNDRV_CHMAP_RR} }, // 5.0 smpte
    { .channels = 6,
      .map = { SNDRV_CHMAP_FL, SNDRV_CHMAP_FR, SNDRV_CHMAP_FC, SNDRV_CHMAP_LFE, SNDRV_CHMAP_RL, SNDRV_CHMAP_RR} }, // 5.1 smpte
    { .channels = 7,
      .map = { SNDRV_CHMAP_FL, SNDRV_CHMAP_FR, SNDRV_CHMAP_FC, SNDRV_CHMAP_LFE, SNDRV_CHMAP_RL, SNDRV_CHMAP_RR, SNDRV_CHMAP_RC} }, // 6.1 smpte
    { .channels = 8,
      .map = { SNDRV_CHMAP_FL, SNDRV_CHMAP_FR, SNDRV_CHMAP_FC, SNDRV_CHMAP_LFE, SNDRV_CHMAP_RL, SNDRV_CHMAP_RR, SNDRV_CHMAP_RLC, SNDRV_CHMAP_RRC} }, // 7.1 smpte
    { }
};


/// NADAC Master Output controls definitions

///Playback Volume control
static int mr_alsa_audio_output_gain_info(  struct snd_kcontrol *kcontrol,// TODO Playback Volume control
                                            struct snd_ctl_elem_info *uinfo)
{
    struct mr_alsa_audio_chip *chip = NULL;
    if(kcontrol == NULL || uinfo == NULL)
        return -EINVAL;
    chip = snd_kcontrol_chip(kcontrol);
    if(chip == NULL)
        return -EINVAL;
    if(chip->ravenna_peer == NULL || chip->mr_alsa_audio_ops == NULL)
        return -EINVAL;

    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    //chip->mr_alsa_audio_ops->get_nb_outputs(chip->ravenna_peer, &uinfo->count);
    //printk("mr_alsa_audio_output_gain_info: uinfo->count = %u \n", uinfo->count);
    uinfo->count = 1;
    uinfo->value.integer.min = -99;
    uinfo->value.integer.max = 0;
    return 0;
}

static int mr_alsa_audio_output_gain_get(struct snd_kcontrol *kcontrol,// TODO Playback Volume control
                    struct snd_ctl_elem_value *ucontrol)
{
    struct mr_alsa_audio_chip *chip = NULL;
    int err = 0;
    //unsigned int chidx = 0;
    //printk(" >> >> enter mr_alsa_audio_output_gain_get\n");
    if(kcontrol == NULL || ucontrol == NULL)
        return -EINVAL;
    chip = snd_kcontrol_chip(kcontrol);
    if(chip == NULL)
        return -EINVAL;

    //spin_lock_irq(&chip->lock);
    if(chip->ravenna_peer == NULL || chip->mr_alsa_audio_ops == NULL)
    {
        err = -EINVAL;
    }
    else
    {
        //printk(KERN_INFO "mr_alsa_audio_output_gain_get: value = %d \n", chip->current_playback_volume);
        //err = chip->mr_alsa_audio_ops->get_master_volume_value(chip->ravenna_peer, (int)SNDRV_PCM_STREAM_PLAYBACK, &value);
        ucontrol->value.integer.value[0] = chip->current_playback_volume;
        // if(chip->current_nboutputs > 1)
        // {
            // for(chidx = 1; chidx < chip->current_nboutputs; ++chidx)
                // ucontrol->value.integer.value[chidx] = chip->current_playback_volume;
        // }
        err = 0;
    }
    //spin_unlock_irq(&chip->lock);
    return err;
}
static int mr_alsa_audio_output_gain_put(struct snd_kcontrol *kcontrol,// TODO Playback Volume control
                    struct snd_ctl_elem_value *ucontrol)
{
    struct mr_alsa_audio_chip *chip = NULL;
    int32_t value = 0;
    int err = 0;
    if(kcontrol == NULL || ucontrol == NULL)
        return -EINVAL;
    chip = snd_kcontrol_chip(kcontrol);
    if(chip == NULL)
        return -EINVAL;
    //spin_lock_irq(&chip->lock);
    if(chip->ravenna_peer == NULL || chip->mr_alsa_audio_ops == NULL)
    {
        err = -EINVAL;
    }
    else
    {
        //printk(KERN_DEBUG "mr_alsa_audio_output_gain_put: numid= %u; name=  %s; iface= %u; sub= %u; index= %u\n", ucontrol->id.numid, ucontrol->id.name, ucontrol->id.iface, ucontrol->id.subdevice, ucontrol->id.index);
        value = ucontrol->value.integer.value[0];
        //printk(KERN_INFO "mr_alsa_audio_output_gain_put: value[0] = %d\n", value);
        if(chip->current_playback_volume != (int32_t)value)
        {
            chip->current_playback_volume = (int32_t)value;
            err = chip->mr_alsa_audio_ops->notify_master_volume_change(chip->ravenna_peer, (int)SNDRV_PCM_STREAM_PLAYBACK, (int32_t)value);
            //spin_unlock_irq(&chip->lock);
            return 1;
        }
    }
    //spin_unlock_irq(&chip->lock);
    return err;
}

static const DECLARE_TLV_DB_SCALE(mr_alsa_audio_db_scale_output_gain, -9900, 100, 0); /// min = -9900 * 0.01 = -99 dB, 100*0.01 dB = 1 dB step

static struct snd_kcontrol_new mr_alsa_audio_ctrl_output_gain = {
    .name = "Master Playback Volume",
    .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
    .access = SNDRV_CTL_ELEM_ACCESS_READWRITE | SNDRV_CTL_ELEM_ACCESS_TLV_READ,
    .info = mr_alsa_audio_output_gain_info,
    .get = mr_alsa_audio_output_gain_get,
    .put = mr_alsa_audio_output_gain_put,
    .tlv = {.p = mr_alsa_audio_db_scale_output_gain}
};

/// Playback Switch control
static int mr_alsa_audio_output_switch_info(struct snd_kcontrol *kcontrol,
                                            struct snd_ctl_elem_info *uinfo)
{
    struct mr_alsa_audio_chip *chip = NULL;
    if(kcontrol == NULL || uinfo == NULL)
        return -EINVAL;
    chip = snd_kcontrol_chip(kcontrol);
    if(chip == NULL)
        return -EINVAL;
    if(chip->ravenna_peer == NULL || chip->mr_alsa_audio_ops == NULL)
        return -EINVAL;

    uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
    //chip->mr_alsa_audio_ops->get_nb_outputs(chip->ravenna_peer, &uinfo->count);
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 1;
    return 0;
}


static int mr_alsa_audio_output_switch_get( struct snd_kcontrol *kcontrol,
                                            struct snd_ctl_elem_value *ucontrol)
{
    struct mr_alsa_audio_chip *chip = NULL;
    int err = 0;
    //unsigned int chidx = 0;
    //printk(" >> >> enter mr_alsa_audio_output_switch_get\n");
    if(kcontrol == NULL || ucontrol == NULL)
        return -EINVAL;
    chip = snd_kcontrol_chip(kcontrol);
    if(chip == NULL)
        return -EINVAL;

    //spin_lock_irq(&chip->lock);
    if(chip->ravenna_peer == NULL || chip->mr_alsa_audio_ops == NULL)
    {
        err = -EINVAL;
    }
    else
    {
        //printk(" >> >> mr_alsa_audio_output_switch_get: value = %d\n", chip->current_playback_switch);
        //err = chip->mr_alsa_audio_ops->get_master_switch_value(chip->ravenna_peer, (int)SNDRV_PCM_STREAM_PLAYBACK, &value);
        ucontrol->value.integer.value[0] = chip->current_playback_switch;
        // if(chip->current_nboutputs > 1)
        // {
            // for(chidx = 1; chidx < chip->current_nboutputs; ++chidx)
                // ucontrol->value.integer.value[chidx] = chip->current_playback_switch;
        // }
        err = 0;
    }
    //spin_unlock_irq(&chip->lock);

    return err;
}
static int mr_alsa_audio_output_switch_put( struct snd_kcontrol *kcontrol,
                                            struct snd_ctl_elem_value *ucontrol)
{
    struct mr_alsa_audio_chip *chip = NULL;
    int32_t value = 0;
    int err = 0;
    // printk(" >> >> enter mr_alsa_audio_output_switch_put\n");
    if(kcontrol == NULL || ucontrol == NULL)
        return -EINVAL;
    chip = snd_kcontrol_chip(kcontrol);
    if(chip == NULL)
        return -EINVAL;

    //spin_lock_irq(&chip->lock);
    if(chip->ravenna_peer == NULL || chip->mr_alsa_audio_ops == NULL)
    {
        err = -EINVAL;
    }
    else
    {
        value = ucontrol->value.integer.value[0];
        //printk(" >> >> mr_alsa_audio_output_switch_put: value = %d\n", value);
        if(chip->current_playback_switch != (int32_t)value)
        {
            chip->current_playback_switch = (int32_t)value;
            err = chip->mr_alsa_audio_ops->notify_master_switch_change(chip->ravenna_peer, (int)SNDRV_PCM_STREAM_PLAYBACK, (int32_t)value);
        }
    }
    //spin_unlock_irq(&chip->lock);

    return err;
}

static struct snd_kcontrol_new mr_alsa_audio_ctrl_output_switch = {
    .name = "Master Playback Switch",
    .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
    .info = mr_alsa_audio_output_switch_info,
    .get = mr_alsa_audio_output_switch_get,
    .put = mr_alsa_audio_output_switch_put
};

//////////////////////////////////////////////////////////////////////////////////////////////////
/* W6: only caller was pcm_open's manager-rate-derived frame size, replaced
 * by the chip's own published frame. Kept for reference. */
static uint32_t __maybe_unused mr_alsa_audio_get_samplerate_factor(unsigned int rate)
{
    if(rate <= 48000)
        return 1;
    else if(rate <= 96000)
        return 2;
    else if(rate <= 192000)
        return 4;
    else if(rate <= 384000)
        return 8;
    else if(rate <= 768000)
        return 16;
    else if(rate <= 1536000)
        return 32;
    else if(rate <= 3072000)
        return 64;
    else if(rate <= 6144000)
        return 128;
    else if(rate <= 12288000)
        return 256;
    else if(rate <= 24576000)
        return 512;
    else
        return -1;
}

static uint32_t mr_alsa_audio_get_dsd_sample_rate(snd_pcm_format_t format, unsigned int rate)
{
    uint32_t dsd_rate = rate;
    switch(format)
    {
    #ifdef SNDRV_PCM_FORMAT_DSD_U8
        case SNDRV_PCM_FORMAT_DSD_U8:
            dsd_rate *= 8;
            break;
    #endif
    #ifdef SNDRV_PCM_FORMAT_DSD_U16_LE
        case SNDRV_PCM_FORMAT_DSD_U16_LE:
            dsd_rate *= 16;
            break;
    #endif
    #ifdef SNDRV_PCM_FORMAT_DSD_U16_BE
        case SNDRV_PCM_FORMAT_DSD_U16_BE:
            dsd_rate *= 16;
            break;
    #endif
    #ifdef SNDRV_PCM_FORMAT_DSD_U32_LE
        case SNDRV_PCM_FORMAT_DSD_U32_LE:
            dsd_rate *= 32;
            break;
    #endif
    #ifdef SNDRV_PCM_FORMAT_DSD_U32_BE
        case SNDRV_PCM_FORMAT_DSD_U32_BE:
            dsd_rate *= 32;
            break;
    #endif
        default:
            return 0;
    }

    if(dsd_rate >= 2822400)
        return dsd_rate;
    else
        return 0;
}

static uint32_t mr_alsa_audio_get_dsd_mode(uint32_t dsdrate)
{
    switch(dsdrate)
    {
        case 2822400:
            return 1;
        case 5644800:
            return 2;
        case 11289600:
            return 4;
        case 22579200:
            return 8;
    }
    return 0;
}


//////////////////////////////////////////////////////////////////////////////////////////////
///
/// Ravenna Manager interface
/// callback functions used by ravenna manager to access alsa driver (e.g. to retrieve buffer)
///
static void* mr_alsa_audio_get_playback_buffer(void *rawchip)
{
    if(rawchip)
    {
        struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip*)rawchip;
        return chip->playback_buffer;

    }
    return NULL;
}
static uint32_t mr_alsa_audio_get_playback_buffer_size_in_frames(void *rawchip)
{
    uint32_t res = 0;
    if (rawchip)
    {
        struct mr_alsa_audio_chip* chip = (struct mr_alsa_audio_chip*)rawchip;
        //spin_lock_irq(&chip->lock);
        {
            struct snd_pcm_runtime* runtime = chip->playback_substream ? chip->playback_substream->runtime : NULL;
            if (chip->playback_buffer)
            {
                if (runtime && runtime->period_size != 0 && runtime->periods != 0)
                {
                    res = chip->current_dsd ? MR_ALSA_RINGBUFFER_NB_FRAMES : runtime->period_size * runtime->periods;
                }
                else
                {
                    res = MR_ALSA_RINGBUFFER_NB_FRAMES;
                }
            }
        }
        //spin_unlock_irq(&chip->lock);
    }
    return res;

}
static void* mr_alsa_audio_get_capture_buffer(void *rawchip)
{
    if(rawchip)
    {
        struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip*)rawchip;
        return chip->capture_buffer;
    }
    return NULL;
}
static uint32_t mr_alsa_audio_get_capture_buffer_size_in_frames(void *rawchip)
{
    // todo f10b put the buffer size in a static var (cannot lock here because of alsa call)
    uint32_t res = 0;
    if (rawchip)
    {
        struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip*)rawchip;
        //spin_lock_irq(&chip->lock);
        {
            struct snd_pcm_runtime *runtime = chip->capture_substream ? chip->capture_substream->runtime : NULL;
            if (chip->capture_buffer)
            {
                if (runtime && runtime->period_size != 0 && runtime->periods != 0)
                {
                    res = chip->current_dsd ? MR_ALSA_RINGBUFFER_NB_FRAMES : runtime->period_size * runtime->periods;
                }
                else
                {
                    res = MR_ALSA_RINGBUFFER_NB_FRAMES;
                }
            }
        }
        //spin_unlock_irq(&chip->lock);
        //if(chip->capture_buffer)
        //    return MR_ALSA_RINGBUFFER_NB_FRAMES;
    }
    return res;
}
/*
 * chip->playback_lock / chip->capture_lock are shared between SOFTIRQ context --
 * the audio TIC: mr_alsa_audio_pcm_interrupt AND the buffer-offset getters, both
 * reached from the manager's SOFT hrtimer (hrtimer_run_softirq; the offset getter
 * via the RTP transmit, SendRTPAudioPackets) -- and PROCESS context: the manager's
 * MuteOutputBuffer/MuteInputBuffer on stop/mute, and pcm_close. Every acquisition
 * therefore uses the _bh variant so a tick softirq can never run on a CPU that
 * already holds the lock in process context. Using plain spin_lock here
 * self-deadlocked: stop()/trigger-stop muted a chip (holding the lock across a
 * multi-MB memset) and the same-CPU tick spun on it forever -> hard lockup.
 * (pcm_close uses spin_lock_irq, which is stronger and equally softirq-safe.)
 */
static void mr_alsa_audio_lock_playback_buffer(void *rawchip)
{
    if(rawchip)
    {
        struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip*)rawchip;
        spin_lock_bh(&chip->playback_lock);
    }
}
static void mr_alsa_audio_unlock_playback_buffer(void *rawchip)
{
    if(rawchip)
    {
        struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip*)rawchip;
        spin_unlock_bh(&chip->playback_lock);
    }
}
static void mr_alsa_audio_lock_capture_buffer(void *rawchip)
{
    if(rawchip)
    {
        struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip*)rawchip;
        spin_lock_bh(&chip->capture_lock);
    }
}
static void mr_alsa_audio_unlock_capture_buffer(void *rawchip)
{
    if(rawchip)
    {
        struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip*)rawchip;
        spin_unlock_bh(&chip->capture_lock);
    }
}

static uint32_t mr_alsa_audio_get_pcm_frame_size(void *mr_alsa_audio_chip);
static uint32_t mr_alsa_audio_get_pcm_sample_rate(void *mr_alsa_audio_chip);

/// Driven by PTP Timer's interrupts
static int mr_alsa_audio_pcm_interrupt(void *rawchip, int direction)
{
    if(rawchip)
    {
        uint32_t ring_buffer_size = MR_ALSA_RINGBUFFER_NB_FRAMES;
        uint32_t ptp_frame_size;
        struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip*)rawchip;
        int do_period_elapsed = 0;

        /* W5 step 3: this chip's OWN frame size — per-rate cadences mean
         * the manager-wide value is wrong for any chip off the legacy
         * rate (a 96k chip would copy half a tick's frames and drift).
         * Acquire-load of the packed (rate, frame) pair published by
         * attach_alsa_driver / set_pcm_sample_rate. */
        ptp_frame_size = mr_alsa_audio_get_pcm_frame_size(chip);

        if (direction == 1) {
            /*
             * Capture path.
             * Lock FIRST, then check substream — prevents race with pcm_close
             * which sets capture_substream = NULL under capture_lock.
             */
            struct snd_pcm_substream *sub;
            struct snd_pcm_runtime *runtime;
            unsigned long bytes_to_frame_factor;

            spin_lock_bh(&chip->capture_lock);

            sub = chip->capture_substream;
            if (!sub) {
                spin_unlock_bh(&chip->capture_lock);
                return 0;
            }
            runtime = sub->runtime;
            ring_buffer_size = chip->current_dsd ? MR_ALSA_RINGBUFFER_NB_FRAMES
                             : runtime->period_size * runtime->periods;

            bytes_to_frame_factor = runtime->channels * chip->current_alsa_capture_stride;

            if (chip->capture_interleave_fn) {
                chip->capture_interleave_fn(
                    chip->capture_buffer_channels_map,
                    chip->capture_buffer_pos,
                    chip->dma_capture_buffer + (uint32_t)atomic_read(&chip->dma_capture_offset),
                    runtime->channels,
                    ptp_frame_size);
            } else {
                mr_alsa_audio_pcm_capture_copy_internal(
                    sub, runtime->channels,
                    chip->capture_buffer_pos,
                    chip->dma_capture_buffer + (uint32_t)atomic_read(&chip->dma_capture_offset),
                    ptp_frame_size);
            }

            {
                uint32_t new_offset = (uint32_t)atomic_read(&chip->dma_capture_offset)
                                    + ptp_frame_size * bytes_to_frame_factor;
                if (new_offset >= chip->pcm_capture_buffer_size)
                    new_offset -= chip->pcm_capture_buffer_size;
                atomic_set(&chip->dma_capture_offset, (int)new_offset);
            }

            chip->capture_buffer_pos += ptp_frame_size;
            if (chip->capture_buffer_pos >= ring_buffer_size)
                chip->capture_buffer_pos -= ring_buffer_size;

            if (++chip->current_capture_interrupt_idx >= chip->nb_capture_interrupts_per_period) {
                chip->current_capture_interrupt_idx = 0;
                do_period_elapsed = 1;
            }

            spin_unlock_bh(&chip->capture_lock);

            /* sub was snapshotted inside lock — safe to use here because
             * ALSA core holds its own refcount on the substream until close
             * fully completes, and snd_pcm_period_elapsed handles the
             * stopped/closing state internally. */
            if (do_period_elapsed)
                snd_pcm_period_elapsed(sub);
        }
        else if (direction == 0) {
            /*
             * Playback path — same lock-first pattern as capture.
             */
            struct snd_pcm_substream *sub;
            struct snd_pcm_runtime *runtime;
            unsigned long bytes_to_frame_factor;

            spin_lock_bh(&chip->playback_lock);

            sub = chip->playback_substream;
            if (!sub) {
                spin_unlock_bh(&chip->playback_lock);
                return 0;
            }
            runtime = sub->runtime;
            ring_buffer_size = chip->current_dsd ? MR_ALSA_RINGBUFFER_NB_FRAMES
                             : runtime->period_size * runtime->periods;

            bytes_to_frame_factor = runtime->channels * chip->current_alsa_playback_stride;

            if (chip->playback_deinterleave_fn) {
                chip->playback_deinterleave_fn(
                    chip->playback_buffer,
                    MR_ALSA_RINGBUFFER_NB_FRAMES,
                    chip->playback_buffer_pos,
                    chip->dma_playback_buffer + (uint32_t)atomic_read(&chip->dma_playback_offset),
                    runtime->channels,
                    ptp_frame_size);
                chip->playback_buffer_alsa_sac += ptp_frame_size;
            } else {
                mr_alsa_audio_pcm_playback_copy_internal(
                    sub, runtime->channels,
                    chip->playback_buffer_pos,
                    chip->dma_playback_buffer + (uint32_t)atomic_read(&chip->dma_playback_offset),
                    ptp_frame_size);
            }

            {
                uint32_t new_offset = (uint32_t)atomic_read(&chip->dma_playback_offset)
                                    + ptp_frame_size * bytes_to_frame_factor;
                if (new_offset >= chip->pcm_playback_buffer_size)
                    new_offset -= chip->pcm_playback_buffer_size;
                atomic_set(&chip->dma_playback_offset, (int)new_offset);
            }

            chip->playback_buffer_pos += ptp_frame_size;
            if (chip->playback_buffer_pos >= ring_buffer_size)
                chip->playback_buffer_pos -= ring_buffer_size;

            if (++chip->current_playback_interrupt_idx >= chip->nb_playback_interrupts_per_period) {
                chip->playback_buffer_rav_sac += ptp_frame_size;
                chip->current_playback_interrupt_idx = 0;
                do_period_elapsed = 1;
            }

            spin_unlock_bh(&chip->playback_lock);

            if (do_period_elapsed)
                snd_pcm_period_elapsed(sub);
        }
        return 0;
    }
    return -1;
}

static uint32_t mr_alsa_audio_pcm_get_playback_buffer_offset(void *rawchip)
{
    uint32_t offset = 0;
    if(rawchip)
    {
        struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip*)rawchip;
        spin_lock_bh(&chip->playback_lock);
        offset = chip->playback_buffer_pos;
        spin_unlock_bh(&chip->playback_lock);
    }
    return offset;
}

static int mr_alsa_audio_notify_master_volume_change(void* rawchip, int direction, int32_t value)
{
    int err = 0;
    if(rawchip)
    {
        struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip*)rawchip;
        if(direction == SNDRV_PCM_STREAM_PLAYBACK)
        {
            if(chip->playback_volume_control)
            {
                if(chip->current_playback_volume != value)
                {
                    chip->current_playback_volume = value;
                    snd_ctl_notify(chip->card, SNDRV_CTL_EVENT_MASK_VALUE, &chip->playback_volume_control->id);
                }
                err = 0;
            }
            else
            {
                err = -EINVAL;
            }
        }
    }
    else
    {
        err = -EINVAL;
    }
    return err;
}

static int mr_alsa_audio_notify_master_switch_change(void* rawchip, int direction, int32_t value)
{
    int err = 0;
    if(rawchip)
    {
        struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip*)rawchip;
        if(direction == SNDRV_PCM_STREAM_PLAYBACK)
        {
            if(chip->playback_switch_control)
            {
                //printk("mr_alsa_audio_notify_master_switch_change: new value = %d \n", value);
                if(value != chip->current_playback_switch)
                {
                    chip->current_playback_switch = value;
                    snd_ctl_notify(chip->card, SNDRV_CTL_EVENT_MASK_VALUE, &chip->playback_switch_control->id);
                }
                err = 0;
            }
            else
            {
                err = -EINVAL;
            }
        }
    }
    else
    {
        err = -EINVAL;
    }
    return err;
}

static void mr_alsa_audio_set_io_state(void *mr_alsa_audio_chip, bool is_playback, bool running)
{
    struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip *)mr_alsa_audio_chip;
    if (!chip)
        return;
    /* Writers: start/stop_interrupts paths (trigger callback, prepare,
     * hw_params, hw_free). Readers: AudioFrameTIC in hrtimer softirq, and
     * recompute_global_io_flags from any trigger context. The bools are
     * word-sized so torn-write is impossible on supported arches, but
     * WRITE_ONCE / READ_ONCE keeps KCSAN quiet and documents that this
     * is concurrent state. */
    if (is_playback)
        WRITE_ONCE(chip->playback_io, running);
    else
        WRITE_ONCE(chip->capture_io, running);
}

static bool mr_alsa_audio_get_io_state(void *mr_alsa_audio_chip, bool is_playback)
{
    struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip *)mr_alsa_audio_chip;
    if (!chip)
        return false;
    return is_playback ? READ_ONCE(chip->playback_io) : READ_ONCE(chip->capture_io);
}

/*
 * Multi-rate Stage 2: per-chip sample rate / frame size accessors.
 *
 * Set: the manager passes rate and frame_size together so a reader never
 * sees a torn (rate, frame_size) pair across a rate change. Two
 * smp_store_release writes give us a partial guarantee — frame_size is
 * published last, so a reader that reads frame_size first (via
 * smp_load_acquire) and then rate (via smp_load_acquire) is guaranteed to
 * see the publishing thread's rate. The hot-path readers below follow
 * that order. The other ordering (read rate then frame_size) is acceptable
 * for our callers because rate is only used for mode/format selection (it
 * doesn't multiply frame_size), and a "rate from new generation, frame_size
 * from old" combination doesn't produce a worse answer than waiting one
 * tick — frame_size still works for the old rate's audio data already in
 * the buffer, which is exactly what we want for the in-flight tick that
 * raced the rate change.
 *
 * The full atomic-pair guarantee will come if/when we serialize chip-rate
 * changes against the tick (Stage 4's per-rate hrtimers naturally do this
 * by tearing down the timer for the old rate before publishing the new
 * one).
 */
static void mr_alsa_audio_set_pcm_sample_rate(void *mr_alsa_audio_chip, uint32_t rate, uint32_t frame_size)
{
    struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip *)mr_alsa_audio_chip;
    uint32_t dsd_mode;
    uint32_t old_rate;
    if (!chip)
        return;
    old_rate = mr_alsa_audio_get_pcm_sample_rate(chip);
    /* W6: refresh this chip's constraint lists alongside the rate
     * publish. A PCM-rate chip advertises exactly its configured rate
     * and its exact tick frame as the only period size; a DSD-rate chip
     * advertises the DSD container rates (narrowed per format by
     * hw_rule_rate_by_format) and the legacy scaled period sizes. */
    dsd_mode = mr_alsa_audio_get_dsd_mode(rate);
    if (dsd_mode != 0)
    {
        chip->supported_rates[0] = 88200;
        chip->supported_rates[1] = 176400;
        chip->supported_rates[2] = 352800;
        chip->constraints_rates.count = 3;
        chip->supported_period_sizes[0] = frame_size / 8;
        chip->supported_period_sizes[1] = frame_size / 4;
        chip->supported_period_sizes[2] = frame_size / 2;
        chip->supported_period_sizes[3] = frame_size;
        chip->constraints_period_sizes.count = 4;
    }
    else
    {
        chip->supported_rates[0] = rate;
        chip->constraints_rates.count = 1;
        chip->supported_period_sizes[0] = frame_size;
        chip->constraints_period_sizes.count = 1;
    }
    chip->constraints_rates.list = chip->supported_rates;
    chip->constraints_rates.mask = 0;
    chip->constraints_period_sizes.list = chip->supported_period_sizes;
    chip->constraints_period_sizes.mask = 0;

    /* F3: single release-store of the packed pair — readers can never
     * see a torn (rate, frame_size) combination. */
    smp_store_release(&chip->pcm_rate_and_frame,
                      ((uint64_t)rate << 32) | (uint64_t)frame_size);

    /* W15: this is the single rate-publish chokepoint (attach, re-key, legacy
     * chip-0 path). If the live rate just reached a pending target, the latch
     * is satisfied — disarm. And whenever the live rate actually changed, tell
     * listeners of the PCM Rate control to re-read it (VOLATILE, no cache). */
    if (chip->pending_rate == rate)
        chip->pending_rate = 0;
    if (chip->current_rate_control && old_rate != rate)
        snd_ctl_notify(chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
                       &chip->current_rate_control->id);
}

static uint32_t mr_alsa_audio_get_pcm_sample_rate(void *mr_alsa_audio_chip)
{
    struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip *)mr_alsa_audio_chip;
    if (!chip)
        return 0;
    return (uint32_t)(smp_load_acquire(&chip->pcm_rate_and_frame) >> 32);
}

static uint32_t mr_alsa_audio_get_pcm_frame_size(void *mr_alsa_audio_chip)
{
    struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip *)mr_alsa_audio_chip;
    if (!chip)
        return 0;
    return (uint32_t)(smp_load_acquire(&chip->pcm_rate_and_frame) & 0xFFFFFFFFu);
}

/* W15: per-PCM read-only "current rate" control. Reports this chip's active
 * sample rate (Hz) so an ALSA capture client (CamillaDSP, or a small follow
 * supervisor) can read the rate and re-open at it after an in-place re-rate.
 * VOLATILE: the value changes underneath userspace on a re-rate without a .put,
 * so ALSA must not cache it. Registered per-PCM with iface PCM + id.device set
 * to the per-card device index, so cards holding several PCMs don't collide. */
static int mr_alsa_audio_current_rate_info(struct snd_kcontrol *kcontrol,
                                           struct snd_ctl_elem_info *uinfo)
{
    if (uinfo == NULL)
        return -EINVAL;
    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 768000; /* headroom over the max supported rate */
    return 0;
}

static int mr_alsa_audio_current_rate_get(struct snd_kcontrol *kcontrol,
                                          struct snd_ctl_elem_value *ucontrol)
{
    struct mr_alsa_audio_chip *chip = NULL;
    if (kcontrol == NULL || ucontrol == NULL)
        return -EINVAL;
    chip = snd_kcontrol_chip(kcontrol);
    if (chip == NULL)
        return -EINVAL;
    /* W15: while armed for an in-place re-rate, report the TARGET so a follower
     * reopens at the rate it's about to become; otherwise the live rate. */
    ucontrol->value.integer.value[0] =
        chip->pending_rate ? chip->pending_rate
                           : mr_alsa_audio_get_pcm_sample_rate(chip);
    return 0;
}

static struct snd_kcontrol_new mr_alsa_audio_ctrl_current_rate = {
    .name = "PCM Rate",
    .iface = SNDRV_CTL_ELEM_IFACE_PCM,
    .access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
    .info = mr_alsa_audio_current_rate_info,
    .get = mr_alsa_audio_current_rate_get,
};

/* W15: true when no substream is open, i.e. the chip is safe to re-key in place.
 * The substream pointers are written under the chip spinlocks in open/close;
 * an aligned pointer read here is atomic, and a stale read only costs the
 * caller (the SetPCMRate handler) one extra retry. */
static bool mr_alsa_audio_pcm_is_idle(void *mr_alsa_audio_chip)
{
    struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip *)mr_alsa_audio_chip;
    if (!chip)
        return true;
    return chip->capture_substream == NULL && chip->playback_substream == NULL;
}

/* W15: latch a pending in-place re-rate on a busy chip (target 0 = disarm).
 * Sets the PCM Rate kcontrol to the target + notifies so a follower closes and
 * reopens at it; the actual re-key happens in pcm_close when the chip goes idle.
 * Process/netlink context only. */
static void mr_alsa_audio_arm_pcm_rate(void *mr_alsa_audio_chip, uint32_t target_rate)
{
    struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip *)mr_alsa_audio_chip;
    if (!chip)
        return;
    if (chip->pending_rate == target_rate)
        return;  /* no change — don't emit a spurious control event */
    chip->pending_rate = target_rate;
    if (chip->current_rate_control)
        snd_ctl_notify(chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
                       &chip->current_rate_control->id);
}

/* W28: the chip's ARMED pending re-rate target (0 = not armed). Plain read of
 * pending_rate — a stale read only mis-times one GetPCMStatus, harmless. */
static uint32_t mr_alsa_audio_get_pcm_pending_rate(void *mr_alsa_audio_chip)
{
    struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip *)mr_alsa_audio_chip;
    return chip ? chip->pending_rate : 0;
}

/* W11: a chip's PTP clock domain is its owning card's (set at add_card). Derived
 * on demand from chip->card via the g_cards lookup — no per-chip field needed. */
static uint8_t mr_alsa_audio_get_pcm_domain(void *mr_alsa_audio_chip)
{
    struct mr_alsa_audio_chip *chip = (struct mr_alsa_audio_chip *)mr_alsa_audio_chip;
    struct mr_alsa_card *mc = chip ? mr_alsa_card_of(chip->card) : NULL;
    return mc ? mc->domain : 0;
}

static struct ravenna_mgr_ops g_ravenna_manager_ops = {
    .get_playback_buffer =  mr_alsa_audio_get_playback_buffer,
    .get_playback_buffer_size_in_frames = mr_alsa_audio_get_playback_buffer_size_in_frames,
    .get_capture_buffer =   mr_alsa_audio_get_capture_buffer,
    .get_capture_buffer_size_in_frames = mr_alsa_audio_get_capture_buffer_size_in_frames,
    .lock_playback_buffer = mr_alsa_audio_lock_playback_buffer,
    .unlock_playback_buffer = mr_alsa_audio_unlock_playback_buffer,
    .lock_capture_buffer = mr_alsa_audio_lock_capture_buffer,
    .unlock_capture_buffer = mr_alsa_audio_unlock_capture_buffer,
    .pcm_interrupt = mr_alsa_audio_pcm_interrupt,
    //.get_capture_buffer_offset = mr_alsa_audio_pcm_get_capture_buffer_offset,
    .get_playback_buffer_offset = mr_alsa_audio_pcm_get_playback_buffer_offset,
    .notify_master_volume_change = mr_alsa_audio_notify_master_volume_change,
    .notify_master_switch_change = mr_alsa_audio_notify_master_switch_change,
    .set_io_state = mr_alsa_audio_set_io_state,
    .get_io_state = mr_alsa_audio_get_io_state,
    .set_pcm_sample_rate = mr_alsa_audio_set_pcm_sample_rate,
    .get_pcm_sample_rate = mr_alsa_audio_get_pcm_sample_rate,
    .get_pcm_frame_size = mr_alsa_audio_get_pcm_frame_size,
    .get_pcm_domain = mr_alsa_audio_get_pcm_domain,
    .pcm_is_idle = mr_alsa_audio_pcm_is_idle,
    .arm_pcm_rate = mr_alsa_audio_arm_pcm_rate,
    .get_pcm_pending_rate = mr_alsa_audio_get_pcm_pending_rate
};


////////////////////////////////////////////////////////////////////////
// PCM interface



/// trigger callback
/// This is called when the pcm is started, stopped or paused.
/// Which action is specified in the second argument, SNDRV_PCM_TRIGGER_XXX in <sound/pcm.h>.
/// At least, the START and STOP commands must be defined in this callback.
/// This callback is atomic. You cannot call functions which may sleep (no mutexes or any schedule-related functions)
/// The trigger callback should be as minimal as possible, just really triggering the DMA. The other stuff should be initialized
/// hw_params and prepare callbacks properly beforehand.
static int mr_alsa_audio_pcm_trigger(struct snd_pcm_substream *alsa_sub, int cmd)
{
    struct mr_alsa_audio_chip *chip = snd_pcm_substream_chip(alsa_sub);
    struct snd_pcm_runtime *runtime = alsa_sub->runtime;
    char cmdString[64];
    printk(KERN_DEBUG "entering mr_alsa_audio_pcm_trigger (substream name=%s #%d) ...\n", alsa_sub->name, alsa_sub->number);
    if(SNDRV_PCM_TRIGGER_START == cmd)
        strcpy(cmdString, "Start");
    else if(SNDRV_PCM_TRIGGER_PAUSE_RELEASE == cmd)
        strcpy(cmdString, "Pause Release");
    else if(SNDRV_PCM_TRIGGER_RESUME == cmd)
        strcpy(cmdString, "Resume");
    else if(SNDRV_PCM_TRIGGER_STOP == cmd)
        strcpy(cmdString, "Stop");
    else if(SNDRV_PCM_TRIGGER_PAUSE_PUSH == cmd)
        strcpy(cmdString, "Pause Push");
    else if(SNDRV_PCM_TRIGGER_SUSPEND == cmd)
        strcpy(cmdString, "Suspend");
    else
        strcpy(cmdString, "Unknown");

    printk(KERN_DEBUG "mr_alsa_audio_pcm_trigger(%s), rate=%d format=%d channels=%d period_size=%lu\n",cmdString,
        runtime->rate, runtime->format, runtime->channels, runtime->period_size);

    switch (cmd) {
    case SNDRV_PCM_TRIGGER_START:
    case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
    case SNDRV_PCM_TRIGGER_RESUME:
        if(alsa_sub->stream == SNDRV_PCM_STREAM_PLAYBACK)
        {
            snd_pcm_sframes_t n = 0;
            n = snd_pcm_playback_hw_avail(runtime);
            n += runtime->delay;
        }
        chip->mr_alsa_audio_ops->start_interrupts(chip->ravenna_peer, chip, alsa_sub->stream == SNDRV_PCM_STREAM_PLAYBACK);
        return 0;

    case SNDRV_PCM_TRIGGER_STOP:
    case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
    case SNDRV_PCM_TRIGGER_SUSPEND:
        chip->mr_alsa_audio_ops->stop_interrupts(chip->ravenna_peer, chip, alsa_sub->stream == SNDRV_PCM_STREAM_PLAYBACK);
        return 0;
    default:
        return -EINVAL;
    }
}

/// prepare callback
/// This callback is called when the pcm is “prepared”. You can set the format type, sample rate,
/// etc. here. The difference from hw_params is that the prepare callback will be called each time
/// snd_pcm_prepare() is called, i.e. when recovering after underruns, etc.
static int mr_alsa_audio_pcm_prepare(struct snd_pcm_substream *substream)
{
    int err = 0;
    struct mr_alsa_audio_chip *chip = snd_pcm_substream_chip(substream);
    struct snd_pcm_runtime *runtime = substream->runtime;

    printk(KERN_DEBUG "entering mr_alsa_audio_pcm_prepare (substream name=%s #%d) ...\n", substream->name, substream->number);

    spin_lock_irq(&chip->lock);
    if(runtime)
    {
        uint32_t runtime_dsd_rate = mr_alsa_audio_get_dsd_sample_rate(runtime->format, runtime->rate);
        uint32_t runtime_dsd_mode = mr_alsa_audio_get_dsd_mode(runtime_dsd_rate);

        if(chip->ravenna_peer == NULL)
        {
            printk(KERN_ERR "mr_alsa_audio_pcm_prepare: ravenna_peer is NULL\n");
            printk(KERN_ERR "leaving mr_alsa_audio_pcm_prepare (failed) ... \n");
            spin_unlock_irq(&chip->lock);
            return -EINVAL;
        }
        printk(KERN_DEBUG "mr_alsa_audio_pcm_prepare: rate=%d format=%d channels=%d period_size=%lu, nb periods=%u\n", runtime->rate, runtime->format, runtime->channels, runtime->period_size, runtime->periods);
        chip->current_rate = mr_alsa_audio_get_pcm_sample_rate(chip);
        chip->current_dsd = mr_alsa_audio_get_dsd_mode(chip->current_rate);

        /* W6: per-PCM rates are fixed at configuration — prepare() never
         * re-rates anything anymore (the legacy path asked the daemon to
         * re-rate the WHOLE card from any client open). The single-rate
         * hw constraint makes a mismatch unreachable; this is a fail-loud
         * backstop. DSD entry is config-driven post-W6 (AddPCM at the DSD
         * rate; SetPCMRate/REST at W10/W12). */
        if(runtime_dsd_mode != 0 ? (runtime_dsd_mode != chip->current_dsd) : (chip->current_rate != runtime->rate))
        {
            printk(KERN_ERR "mr_alsa_audio_pcm_prepare: requested rate %u (dsd_mode %u) does not match this pcm's configured rate %u (dsd %u) — per-PCM rates are fixed\n",
                   runtime->rate, runtime_dsd_mode, chip->current_rate, chip->current_dsd);
            spin_unlock_irq(&chip->lock);
            return -EINVAL;
        }

        /// Number of channels
        if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
        {
            printk(KERN_DEBUG "mr_alsa_audio_pcm_prepare for playback stream\n");
            if(chip->ravenna_peer)
            {
                chip->current_nboutputs = runtime->channels;
                if(chip->playback_volume_control)
                    snd_ctl_notify(chip->card, SNDRV_CTL_EVENT_MASK_INFO, &chip->playback_volume_control->id);
                if(err < 0)
                {
                    printk(KERN_ERR "mr_alsa_audio_pcm_prepare for playback stream failed\n");
                    spin_unlock_irq(&chip->lock);
                    return -EINVAL;
                }
            }
            chip->current_alsa_playback_format = runtime->format;
            chip->current_alsa_playback_stride = snd_pcm_format_physical_width(runtime->format) >> 3;
            chip->playback_buffer_pos = 0;
            chip->playback_buffer_alsa_sac = 0;
            chip->playback_buffer_rav_sac = 0;

            chip->current_playback_interrupt_idx = 0;

            /// Ravenna DSD always uses a rate of 352k with eventual zero padding to maintain a 32 bit alignment
            /// while DSD in ALSA uses a continuous 8, 16 or 32 bit aligned stream with at 352k, 176k or 88k
            /// so respective ring buffers might have different scale and size
            chip->nb_playback_interrupts_per_period = ((runtime_dsd_mode != 0)? (MR_ALSA_PTP_FRAME_RATE_FOR_DSD / runtime->rate) : 1);

            /// Fill the additional delay between the packet output and the sound eared
            /* W9 #14: per-PCM advisory latency, read straight off this chip (no
             * manager round-trip / shared m_nPlayoutDelay). */
            runtime->delay = chip->playout_delay;

            /* Select optimized de-interleave function based on format */
            {
                uint32_t dsd_rate = mr_alsa_audio_get_dsd_sample_rate(runtime->format, runtime->rate);
                if (dsd_rate == 0) {
                    /* PCM mode — use optimized path */
                    unsigned int bits = snd_pcm_format_width(runtime->format);
                    switch (bits) {
                    case 32:
                        chip->playback_deinterleave_fn = playback_deinterleave_s32le;
                        break;
                    case 24:
                        chip->playback_deinterleave_fn =
                            (chip->current_alsa_playback_stride == 3) ?
                            playback_deinterleave_s24_3le :
                            playback_deinterleave_s24le;
                        break;
                    case 16:
                        chip->playback_deinterleave_fn = playback_deinterleave_s16le;
                        break;
                    default:
                        chip->playback_deinterleave_fn = NULL;
                        break;
                    }
                } else {
                    /* DSD mode — fall back to generic path */
                    chip->playback_deinterleave_fn = NULL;
                }
            }

            atomic_set(&chip->dma_playback_offset, 0);
            chip->dma_playback_buffer = runtime->dma_area;
            chip->pcm_playback_buffer_size = snd_pcm_lib_buffer_bytes(substream);
        }
        else if(substream->stream == SNDRV_PCM_STREAM_CAPTURE)
        {
            uint32_t offset = 0;
            /* 2026-06-09 review fix: per-PCM offset — this chip's ring and SAC,
             * not chip 0's. W10: key on the manager's GLOBAL pcm_id
             * (chip->global_pcm_id), not the per-card ALSA device index — they
             * coincide under single-card but diverge under multi-card. */
            chip->mr_alsa_audio_ops->get_input_jitter_buffer_offset_for_pcm(chip->ravenna_peer, (uint32_t)chip->global_pcm_id, &offset);
            
            printk(KERN_DEBUG "mr_alsa_audio_pcm_prepare for capture stream\n");
            if(chip->ravenna_peer)
            {
                chip->current_nbinputs = runtime->channels;

                if(err < 0)
                {
                    printk(KERN_ERR "mr_alsa_audio_pcm_prepare for capture stream failed\n");
                    spin_unlock_irq(&chip->lock);
                    return -EINVAL;
                }
            }
            chip->current_alsa_capture_format = runtime->format;
            chip->current_alsa_capture_stride = snd_pcm_format_physical_width(runtime->format) >> 3;
            chip->capture_buffer_pos = offset;
            chip->current_capture_interrupt_idx = 0;
            /* W9 #14: per-PCM advisory capture latency. Advisory only — the real
             * receive buffering is the per-sink link offset (capture_buffer_pos
             * above), not this. Previously dead (get_capture_delay had no
             * caller); now reported via runtime->delay for snd_pcm_delay(). */
            runtime->delay = chip->capture_delay;
            chip->nb_capture_interrupts_per_period = ((runtime_dsd_mode != 0)? (MR_ALSA_PTP_FRAME_RATE_FOR_DSD / runtime->rate) : 1);

            /* Select optimized capture interleave function */
            {
                unsigned int bits = snd_pcm_format_width(runtime->format);
                unsigned int stride = chip->current_alsa_capture_stride;
                switch (bits) {
                case 32:
                    chip->capture_interleave_fn = MTConvertMappedInt32ToInt32LEInterleave;
                    break;
                case 24:
                    chip->capture_interleave_fn = (stride == 3) ?
                        MTConvertMappedInt32ToInt24LEInterleave :
                        MTConvertMappedInt32ToInt24LE4ByteInterleave;
                    break;
                case 16:
                    chip->capture_interleave_fn = MTConvertMappedInt32ToInt16LEInterleave;
                    break;
                default:
                    chip->capture_interleave_fn = NULL;
                    break;
                }
            }

            atomic_set(&chip->dma_capture_offset, 0);
            chip->dma_capture_buffer = runtime->dma_area;
            chip->pcm_capture_buffer_size = snd_pcm_lib_buffer_bytes(substream);
        }
    }
    else
    {
        printk(KERN_ERR "Error in mr_alsa_audio_pcm_prepare: runtime is NULL\n");
    }
    spin_unlock_irq(&chip->lock);

    /* set_jitter_buffer_depth is outside the spinlock because it may
     * potentially sleep (future implementation). The per-entry timer base
     * period is owned by the manager's (domain,rate) registry (W5) and set
     * when the entry is keyed (tic_entry_refresh_base_period) — not from the
     * ALSA prepare path, which has no handle on the entry's clock_timer.
     * Upstream 367c166's single-timer update_base_period() call here is
     * therefore dropped. */
    if (chip->ravenna_peer && runtime) {
        uint32_t current_ptp_frame_size;
        chip->mr_alsa_audio_ops->get_interrupts_frame_size(
            chip->ravenna_peer, &current_ptp_frame_size);

        if (chip->mr_alsa_audio_ops->set_jitter_buffer_depth) {
            chip->mr_alsa_audio_ops->set_jitter_buffer_depth(
                chip->ravenna_peer,
                current_ptp_frame_size * jitter_buffer_multiplier);
        }
    }

    printk(KERN_DEBUG "Leaving mr_alsa_audio_pcm_prepare..\n");
    return err;
}


/// pointer callback
/// This callback is called when the PCM middle layer inquires the current hardware position on the buffer.
/// The position must be returned in frames, ranging from 0 to buffer_size - 1
static snd_pcm_uframes_t mr_alsa_audio_pcm_pointer(struct snd_pcm_substream *alsa_sub)
{
    struct mr_alsa_audio_chip *chip = snd_pcm_substream_chip(alsa_sub);
    uint32_t offset = 0;
    //printk("entering mr_alsa_audio_pcm_pointer (substream name=%s #%d) ...\n", alsa_sub->name, alsa_sub->number);

    /* Lock-free: interrupt handler writes atomically, we read atomically */
    if(alsa_sub->stream == SNDRV_PCM_STREAM_PLAYBACK)
    {
        struct snd_pcm_runtime *runtime = alsa_sub->runtime;
        unsigned long bytes_to_frame_factor = runtime->channels * chip->current_alsa_playback_stride;
        if (unlikely(bytes_to_frame_factor == 0))
            return 0;
        offset = (uint32_t)atomic_read(&chip->dma_playback_offset) / bytes_to_frame_factor;

        switch(chip->nb_playback_interrupts_per_period)
        {
            case 2:
                offset >>= 1;
                break;
            case 4:
                offset >>= 2;
                break;
            case 8:
                offset >>= 3;
                break;
            default:
                break;
        }
    }
    else if(alsa_sub->stream == SNDRV_PCM_STREAM_CAPTURE)
    {
        struct snd_pcm_runtime *runtime = alsa_sub->runtime;
        unsigned long bytes_to_frame_factor = runtime->channels * chip->current_alsa_capture_stride;
        if (unlikely(bytes_to_frame_factor == 0))
            return 0;
        offset = (uint32_t)atomic_read(&chip->dma_capture_offset) / bytes_to_frame_factor;

        switch(chip->nb_capture_interrupts_per_period)
        {
            case 2:
                offset >>= 1;
                break;
            case 4:
                offset >>= 2;
                break;
            case 8:
                offset >>= 3;
                break;
            default:
                break;
        }
    }
    return offset;
}

/// hardware descriptor
/// The info field contains the type and capabilities of this pcm. The bit flags are
/// defined in <sound/asound.h> as SNDRV_PCM_INFO_XXX. Here, at least, you have
/// to specify whether the mmap is supported and which interleaved format is supported.
/// When the is supported, add the SNDRV_PCM_INFO_MMAP flag here. When the hardware
/// supports the interleaved or the non-interleaved formats, SNDRV_PCM_INFO_INTERLEAVED or
/// SNDRV_PCM_INFO_NONINTERLEAVED flag must be set, respectively. If both are supported, you
/// can set both, too.
/// In the above example, MMAP_VALID and BLOCK_TRANSFER are specified for the OSS mmap mode.
/// Usually both are set. Of course, MMAP_VALID is set only if the mmap is really supported.
/// The other possible flags are SNDRV_PCM_INFO_PAUSE and SNDRV_PCM_INFO_RESUME. The
/// PAUSE bit means that the pcm supports the “pause” operation, while the RESUME bit means that the pcm
/// supports the full “suspend/resume” operation. If the PAUSE flag is set, the trigger callback below
/// must handle the corresponding (pause push/release) commands. The suspend/resume trigger commands
/// can be defined even without the RESUME flag. See Power Management section for details.
/// When the PCM substreams can be synchronized (typically, synchronized start/stop of a playback and
/// a capture streams), you can give SNDRV_PCM_INFO_SYNC_START, too. In this case, you'll need
/// to check the linked-list of PCM substreams in the trigger callback. This will be described in the later
/// section.
/// - formats field contains the bit-flags of supported formats (SNDRV_PCM_FMTBIT_XXX). If the
/// hardware supports more than one format, give all or'ed bits.
/// - rates field contains the bit-flags of supported rates (SNDRV_PCM_RATE_XXX). When the chip
/// supports continuous rates, pass CONTINUOUS bit additionally. The pre-defined rate bits are provided
/// only for typical rates. If your chip supports unconventional rates, you need to add the KNOT bit and set
/// up the hardware constraint manually (explained later).
/// - rate_min and rate_max define the minimum and maximum sample rate. This should correspond
/// somehow to rates bits.
/// - channel_min and channel_max define, as you might already expected, the minimum and
/// maximum number of channels.
/// - buffer_bytes_max defines the maximum buffer size in bytes. There is no buffer_bytes_min
/// field, since it can be calculated from the minimum period size and the minimum number of periods.
/// Meanwhile, period_bytes_min and define the minimum and maximum size of the period in bytes.
/// periods_max and periods_min define the maximum and minimum number of periods in the
/// buffer.
/// The “period” is a term that corresponds to a fragment in the OSS world. The period defines the size at
/// which a PCM interrupt is generated. This size strongly depends on the hardware. Generally, the smaller
/// period size will give you more interrupts, that is, more controls. In the case of capture, this size defines
/// the input latency. On the other hand, the whole buffer size defines the output latency for the playback
/// direction.
/// - There is also a field fifo_size. This specifies the size of the hardware FIFO, but currently it is neither
/// used in the driver nor in the alsa-lib. So, you can ignore this field.
static struct snd_pcm_hardware mr_alsa_audio_pcm_hardware_playback =
{
    .info =     (   SNDRV_PCM_INFO_MMAP | /* hardware supports mmap */
                    SNDRV_PCM_INFO_INTERLEAVED |
                    /*SNDRV_PCM_INFO_NONINTERLEAVED |  channels are not interleaved */
                    SNDRV_PCM_INFO_BLOCK_TRANSFER | /* hardware transfer block of samples */
                    SNDRV_PCM_INFO_JOINT_DUPLEX |
                    SNDRV_PCM_INFO_PAUSE | /* pause ioctl is supported */
                    SNDRV_PCM_INFO_MMAP_VALID /*| period data are valid during transfer */
                    //SNDRV_PCM_INFO_BATCH /* double buffering */
                     /*| SNDRV_PCM_INFO_JOINT_DUPLEX*/ /*| SNDRV_PCM_INFO_PAUSE*/ /*| SNDRV_PCM_INFO_RESUME*/),
    .formats = (
    #ifdef SNDRV_PCM_FMTBIT_DSD_U8
            SNDRV_PCM_FMTBIT_DSD_U8 |
    #endif
    #ifdef SNDRV_PCM_FMTBIT_DSD_U16_BE
            SNDRV_PCM_FMTBIT_DSD_U16_BE |
    #endif
    #ifdef SNDRV_PCM_FMTBIT_DSD_U32_BE
            SNDRV_PCM_FMTBIT_DSD_U32_BE |
    #endif
        SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
        SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE),
    .rates =    (SNDRV_PCM_RATE_KNOT|SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000|SNDRV_PCM_RATE_88200|SNDRV_PCM_RATE_96000|SNDRV_PCM_RATE_176400|SNDRV_PCM_RATE_192000),
    .rate_min =         44100,
    .rate_max =         384000,
    .channels_min =     1,
    .channels_max =     MR_ALSA_NB_CHANNELS_MAX,
    .buffer_bytes_max = MR_ALSA_RINGBUFFER_NB_FRAMES * MR_ALSA_NB_CHANNELS_MAX * 4, // 4 bytes per sample, 128 ch
    .period_bytes_min = 6 * 1 * 2,  /* 6 frames (AES67 125us), 1 channel, 16-bit minimum */
    .period_bytes_max = 512 * 8 * MR_ALSA_NB_CHANNELS_MAX * 4,  /* 512 (max legacy) * 8FS * max_ch * 32bit */
    .periods_min =      2, // min number of periods per buffer (for 8fs)
    .periods_max =      96, // max number of periods per buffer (for 1fs)
    .fifo_size =        0
};


static struct snd_pcm_hardware mr_alsa_audio_pcm_hardware_capture =
{
    .info =     (   SNDRV_PCM_INFO_MMAP | /* hardware supports mmap */
                    SNDRV_PCM_INFO_INTERLEAVED |
                    /*SNDRV_PCM_INFO_NONINTERLEAVED |  channels are not interleaved */
                    SNDRV_PCM_INFO_BLOCK_TRANSFER | /* hardware transfer block of samples */
                    SNDRV_PCM_INFO_JOINT_DUPLEX |
                    SNDRV_PCM_INFO_PAUSE | /* pause ioctl is supported */
                    SNDRV_PCM_INFO_MMAP_VALID /*|  period data are valid during transfer */
                    //SNDRV_PCM_INFO_BATCH /* double buffering */
                     /*| SNDRV_PCM_INFO_JOINT_DUPLEX*/ /*| SNDRV_PCM_INFO_PAUSE*/ /*| SNDRV_PCM_INFO_RESUME*/), // TODO (mmap, pause/resume, duplex)
    //.formats =  (SNDRV_PCM_FMTBIT_S32_LE/* | SNDRV_PCM_FMTBIT_S24_3LE*//* | SNDRV_PCM_FMTBIT_FLOAT_LE*/), // TODO (float?)
    .formats = (
        SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
        SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE),
    .rates =    (SNDRV_PCM_RATE_KNOT|SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000|SNDRV_PCM_RATE_88200|SNDRV_PCM_RATE_96000|SNDRV_PCM_RATE_176400|SNDRV_PCM_RATE_192000),
    .rate_min =         44100,
    .rate_max =         384000,
    .channels_min =     1,
    .channels_max =     MR_ALSA_NB_CHANNELS_MAX,
    .buffer_bytes_max = MR_ALSA_RINGBUFFER_NB_FRAMES * MR_ALSA_NB_CHANNELS_MAX * 4, // 4 bytes per sample, 128 ch
    .period_bytes_min = 6 * 1 * 2,  /* 6 frames (AES67 125us), 1 channel, 16-bit minimum */
    .period_bytes_max = 512 * 8 * MR_ALSA_NB_CHANNELS_MAX * 4,  /* 512 (max legacy) * 8FS * max_ch * 32bit */
    .periods_min =      2, // min number of periods per buffer (for 8fs)
    .periods_max =      96, // min number of periods per buffer (for 1fs)
    .fifo_size =        0
};


static int mr_alsa_audio_pcm_capture_copy_internal(  struct snd_pcm_substream *substream,
                                            int channel, uint32_t pos,
                                            void __user *src,
                                            snd_pcm_uframes_t count)
{
    struct mr_alsa_audio_chip *chip = snd_pcm_substream_chip(substream);
    struct snd_pcm_runtime *runtime = substream->runtime;
    bool interleaved = (runtime->access == SNDRV_PCM_ACCESS_RW_INTERLEAVED || runtime->access == SNDRV_PCM_ACCESS_MMAP_INTERLEAVED);
    unsigned int nb_logical_bits = snd_pcm_format_width(runtime->format);
    unsigned int strideIn = chip->current_alsa_capture_stride;
    uint32_t ravenna_buffer_pos = pos;

    if(interleaved)
    {
        switch(nb_logical_bits)
        {
            case 16:
                MTConvertMappedInt32ToInt16LEInterleave(chip->capture_buffer_channels_map, ravenna_buffer_pos, src, runtime->channels, count);
                break;
            case 24:
            {
                switch(strideIn)
                {
                    case 3:
                        MTConvertMappedInt32ToInt24LEInterleave(chip->capture_buffer_channels_map, ravenna_buffer_pos, src, runtime->channels, count);
                    break;
                    case 4:
                        MTConvertMappedInt32ToInt24LE4ByteInterleave(chip->capture_buffer_channels_map, ravenna_buffer_pos, src, runtime->channels, count);
                    break;
                    default:
                    {
                        printk(KERN_WARNING "Capture copy in 24bit with StrideIn = %u is not supported\n", strideIn);
                        return -EINVAL;
                    }
                }
                break;
            }
            case 32:
                MTConvertMappedInt32ToInt32LEInterleave(chip->capture_buffer_channels_map, ravenna_buffer_pos, src, runtime->channels, count);
                break;
        }
    }
    else
    {
        printk(KERN_WARNING "Uninterleaved Capture is not yet supported\n");
        return -EINVAL;
    }
    return count;
}


/*
 * Optimized playback de-interleave functions.
 * These replace the generic per-sample switch in the inner loop.
 * Each function handles one format, with ring buffer wrap computed
 * outside the inner loop for maximum throughput.
 *
 * Alignment note: src comes from runtime->dma_area + dma_playback_offset.
 * dma_area is page-aligned, and dma_playback_offset is always a multiple
 * of (channels * stride), so 4-byte alignment is guaranteed for S32_LE.
 */

static void playback_deinterleave_s32le(unsigned char *playback_buffer,
                                        uint32_t ring_buffer_frames,
                                        uint32_t ravenna_pos,
                                        const unsigned char *src,
                                        unsigned int channels,
                                        snd_pcm_uframes_t frames)
{
    unsigned int ch;
    const int32_t *in = (const int32_t *)src;
    uint32_t stride_out = 4;
    uint32_t ring_bytes = ring_buffer_frames * stride_out;

    for (ch = 0; ch < channels; ch++) {
        int32_t *out = (int32_t *)(playback_buffer + ch * ring_bytes);
        uint32_t before_wrap = ring_buffer_frames - ravenna_pos;
        uint32_t first = min((uint32_t)frames, before_wrap);
        uint32_t f;

        for (f = 0; f < first; f++)
            out[ravenna_pos + f] = in[f * channels + ch];
        for (f = first; f < (uint32_t)frames; f++)
            out[f - first] = in[f * channels + ch];
    }
}

static void playback_deinterleave_s24le(unsigned char *playback_buffer,
                                        uint32_t ring_buffer_frames,
                                        uint32_t ravenna_pos,
                                        const unsigned char *src,
                                        unsigned int channels,
                                        snd_pcm_uframes_t frames)
{
    unsigned int ch;
    const unsigned char *in = src;
    uint32_t stride_in = 4;
    uint32_t stride_out = 4;
    uint32_t ring_bytes = ring_buffer_frames * stride_out;

    for (ch = 0; ch < channels; ch++) {
        unsigned char *out = playback_buffer + ch * ring_bytes;
        uint32_t before_wrap = ring_buffer_frames - ravenna_pos;
        uint32_t first = min((uint32_t)frames, before_wrap);
        uint32_t f;

        for (f = 0; f < first; f++) {
            const unsigned char *s = in + (f * channels + ch) * stride_in;
            unsigned char *d = out + (ravenna_pos + f) * stride_out;
            d[3] = s[2]; d[2] = s[1]; d[1] = s[0]; d[0] = 0;
        }
        for (f = first; f < (uint32_t)frames; f++) {
            const unsigned char *s = in + (f * channels + ch) * stride_in;
            unsigned char *d = out + (f - first) * stride_out;
            d[3] = s[2]; d[2] = s[1]; d[1] = s[0]; d[0] = 0;
        }
    }
}

static void playback_deinterleave_s24_3le(unsigned char *playback_buffer,
                                          uint32_t ring_buffer_frames,
                                          uint32_t ravenna_pos,
                                          const unsigned char *src,
                                          unsigned int channels,
                                          snd_pcm_uframes_t frames)
{
    unsigned int ch;
    const unsigned char *in = src;
    uint32_t stride_in = 3;
    uint32_t stride_out = 4;
    uint32_t ring_bytes = ring_buffer_frames * stride_out;

    for (ch = 0; ch < channels; ch++) {
        unsigned char *out = playback_buffer + ch * ring_bytes;
        uint32_t before_wrap = ring_buffer_frames - ravenna_pos;
        uint32_t first = min((uint32_t)frames, before_wrap);
        uint32_t f;

        for (f = 0; f < first; f++) {
            const unsigned char *s = in + (f * channels + ch) * stride_in;
            unsigned char *d = out + (ravenna_pos + f) * stride_out;
            d[3] = s[2]; d[2] = s[1]; d[1] = s[0]; d[0] = 0;
        }
        for (f = first; f < (uint32_t)frames; f++) {
            const unsigned char *s = in + (f * channels + ch) * stride_in;
            unsigned char *d = out + (f - first) * stride_out;
            d[3] = s[2]; d[2] = s[1]; d[1] = s[0]; d[0] = 0;
        }
    }
}

static void playback_deinterleave_s16le(unsigned char *playback_buffer,
                                        uint32_t ring_buffer_frames,
                                        uint32_t ravenna_pos,
                                        const unsigned char *src,
                                        unsigned int channels,
                                        snd_pcm_uframes_t frames)
{
    unsigned int ch;
    const unsigned char *in = src;
    uint32_t stride_in = 2;
    uint32_t stride_out = 4;
    uint32_t ring_bytes = ring_buffer_frames * stride_out;

    for (ch = 0; ch < channels; ch++) {
        unsigned char *out = playback_buffer + ch * ring_bytes;
        uint32_t before_wrap = ring_buffer_frames - ravenna_pos;
        uint32_t first = min((uint32_t)frames, before_wrap);
        uint32_t f;

        for (f = 0; f < first; f++) {
            const unsigned char *s = in + (f * channels + ch) * stride_in;
            unsigned char *d = out + (ravenna_pos + f) * stride_out;
            d[3] = s[1]; d[2] = s[0]; d[1] = 0; d[0] = 0;
        }
        for (f = first; f < (uint32_t)frames; f++) {
            const unsigned char *s = in + (f * channels + ch) * stride_in;
            unsigned char *d = out + (f - first) * stride_out;
            d[3] = s[1]; d[2] = s[0]; d[1] = 0; d[0] = 0;
        }
    }
}

static int mr_alsa_audio_pcm_playback_copy_internal( struct snd_pcm_substream *substream,
                                            int channel, uint32_t pos,
                                            void __user *src,
                                            snd_pcm_uframes_t count)
{
    struct mr_alsa_audio_chip *chip = snd_pcm_substream_chip(substream);
    struct snd_pcm_runtime *runtime = substream->runtime;
    int chn = 0;
    bool interleaved = (runtime->access == SNDRV_PCM_ACCESS_RW_INTERLEAVED || runtime->access == SNDRV_PCM_ACCESS_MMAP_INTERLEAVED);
    unsigned int nb_logical_bits = snd_pcm_format_width(runtime->format);
    unsigned int strideIn = chip->current_alsa_playback_stride;
    unsigned int strideOut = snd_pcm_format_physical_width(SNDRV_PCM_FORMAT_S32_LE) >> 3;
    uint32_t dsdrate = mr_alsa_audio_get_dsd_sample_rate(runtime->format, runtime->rate);
    uint32_t dsdmode = (dsdrate > 0? mr_alsa_audio_get_dsd_mode(dsdrate) : 0);
    uint32_t ravenna_buffer_pos = pos;

    if(interleaved)
    {
        /// de-interleaving
        unsigned char *in, *out;
        unsigned int stepIn = runtime->channels * strideIn;
        unsigned int stepOut = strideOut * chip->nb_playback_interrupts_per_period;
        uint32_t ring_buffer_size = MR_ALSA_RINGBUFFER_NB_FRAMES * strideOut;

        for (chn = 0; chn < runtime->channels; ++chn)
        {
            uint32_t currentOutPos = ravenna_buffer_pos * strideOut;
            snd_pcm_uframes_t frmCnt = 0;
            in = (unsigned char*)src + chn * strideIn;
            out = chip->playback_buffer + chn * ring_buffer_size + currentOutPos;
            //
            ///Conversion to Signed integer 32 bit LE
            for (frmCnt = 0; frmCnt < count; ++frmCnt)
            {
                /// assumes Little Endian
                if (dsdmode == 0)
                {
                    switch (nb_logical_bits)
                    {
                    case 16:
                        out[3] = in[1];
                        out[2] = in[0];
                        out[1] = 0;
                        out[0] = 0;
                        break;
                    case 24:
                        out[3] = in[2];
                        out[2] = in[1];
                        out[1] = in[0];
                        out[0] = 0;
                        break;
                    case 32:
                        *(int32_t*)out = *(int32_t*)in;
                        break;
                    }
                }
                else
                {
                    /// interleaved DSD stream to non interleaved 32 bit aligned blocks with 1/2/4 DSD bytes per 32 bit
                    uint32_t out_cnt;
                    for (out_cnt = 0; out_cnt < chip->nb_playback_interrupts_per_period; ++out_cnt)
                    {
                        switch (dsdmode)
                        {
                        case 1: ///DSD64
                            ((int32_t*)out)[out_cnt] = *(int32_t*)(in + out_cnt) & 0xFF;
                            break;
                            case 2: ///DSD128
                            ((int32_t*)out)[out_cnt] = (((int32_t)(in[2 * out_cnt + 1]) << 8) | ((int32_t)(in[2 * out_cnt]))) & 0xFFFF;
                            break;
                        case 4: ///DSD256
                            ((int32_t*)out)[out_cnt] = *(int32_t*)(in);
                            break;
                        }
                    }
                }

                in += stepIn;
                if (currentOutPos + stepOut >= ring_buffer_size)
                {
                    currentOutPos = 0;
                    out = chip->playback_buffer + chn * ring_buffer_size;
                }
                else
                {
                    currentOutPos += stepOut;
                    out += stepOut;
                }
            }
        }
    }
    else
    {
        printk(KERN_WARNING "Uninterleaved Playback is not supported\n");
        return -EINVAL;
    }
    
    chip->playback_buffer_alsa_sac += count;
    return count;
}



/// hw_params callback
/// This is called when the hardware parameter (hw_params) is set up by the application, that is, once when
/// the buffer size, the period size, the format, etc. are defined for the pcm substream.
/// Many hardware setups should be done in this callback, including the allocation of buffers.
/// Parameters to be initialized are retrieved by params_xxx() macros. To allocate buffer, you can call
/// a helper function,
/// snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
/// snd_pcm_lib_malloc_pages() is available only when the DMA buffers have been pre-allocated.
/// Note that this and prepare callbacks may be called multiple times per initialization. For example, the
/// OSS emulation may call these callbacks at each change via its ioctl.
/// Thus, you need to be careful not to allocate the same buffers many times, which will lead to memory leaks!
/// Calling the helper function above many times is OK. It will release the previous buffer automatically when
/// it was already allocated.
/// Another note is that this callback is non-atomic (schedulable). This is important, because the trigger
/// callback is atomic (non-schedulable). That is, mutexes or any schedule-related functions are not available
/// in trigger callback.
static int mr_alsa_audio_pcm_hw_params( struct snd_pcm_substream *substream,
                                        struct snd_pcm_hw_params *params)
{
    int err = 0;

    struct mr_alsa_audio_chip *chip = snd_pcm_substream_chip(substream);
    //struct snd_pcm_runtime *runtime = substream->runtime;
    //unsigned long flags;
    uint32_t ptp_frame_size = 0;
    unsigned int        rate = params_rate(params);
    snd_pcm_format_t    format = params_format(params);
    unsigned int nbCh = params_channels(params);
    unsigned int periodSize = params_period_size(params);
    unsigned int nbPeriods = params_periods(params);
    unsigned int bufferSize = params_buffer_size(params);
    unsigned int bufferBytes = params_buffer_bytes(params);
    uint32_t dsd_rate = mr_alsa_audio_get_dsd_sample_rate(format, rate);
    uint32_t dsd_mode = mr_alsa_audio_get_dsd_mode(dsd_rate);

    printk(KERN_DEBUG "mr_alsa_audio_pcm_hw_params (enter): rate=%d format=%d channels=%d period_size=%u, nb_periods=%u, buffer_bytes=%u\n", rate, format, nbCh, periodSize, nbPeriods, bufferBytes);
    spin_lock_irq(&chip->lock);
    
    #ifdef MUTE_CHECK
        playback_mute_detected = false;
    #endif

    /* W6: per-PCM rates are fixed at configuration (Decision 7). The
     * legacy path here asked the daemon to re-rate the WHOLE card (global
     * SetSamplingRate round-trip) from any unprivileged client open —
     * under per-rate timer entries that is exactly the shared-entry
     * re-key hazard. The single-rate hw constraint makes a mismatched
     * negotiation unreachable; fail-loud backstop only. */
    chip->current_rate = mr_alsa_audio_get_pcm_sample_rate(chip);
    chip->current_dsd = mr_alsa_audio_get_dsd_mode(chip->current_rate);
    if(dsd_mode != 0 ? (dsd_mode != chip->current_dsd) : (rate != chip->current_rate))
    {
        printk(KERN_ERR "mr_alsa_audio_pcm_hw_params: requested rate %u (dsd_mode %u) does not match this pcm's configured rate %u (dsd %u) — per-PCM rates are fixed\n",
               rate, dsd_mode, chip->current_rate, chip->current_dsd);
        spin_unlock_irq(&chip->lock);
        return -EINVAL;
    }

    /* W5 step 3: per-chip frame size for the period-size sanity check
     * (also always initialized — the old ops call left ptp_frame_size
     * uninitialized when ravenna_peer was NULL). */
    ptp_frame_size = mr_alsa_audio_get_pcm_frame_size(chip);

    if(periodSize != ptp_frame_size)
        printk(KERN_WARNING "mr_alsa_audio_pcm_hw_params : periodSize (%u) differs from ptp_frame_size (%u)\n", periodSize, ptp_frame_size);

    if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
    {
        chip->current_alsa_playback_stride = snd_pcm_format_physical_width(format) >> 3;
        chip->playback_buffer_pos = 0;
        chip->playback_buffer_alsa_sac = 0;
        chip->playback_buffer_rav_sac = 0;

        chip->current_playback_interrupt_idx = 0;

        /// Ravenna DSD always uses a rate of 352k with eventual zero padding to maintain a 32 bit alignment
        /// while DSD in ALSA uses a continuous 8, 16 or 32 bit aligned stream with at 352k, 176k or 88k
        /// so respective ring buffers might have different scale and size
        chip->nb_playback_interrupts_per_period = ((dsd_mode != 0)? (MR_ALSA_PTP_FRAME_RATE_FOR_DSD / rate) : 1);
        if(nbPeriods * ptp_frame_size * chip->nb_playback_interrupts_per_period != MR_ALSA_RINGBUFFER_NB_FRAMES)
            printk(KERN_INFO "mr_alsa_audio_pcm_hw_params (playback): nbPeriods (%u) differs from expected (%u)\n", nbPeriods, MR_ALSA_RINGBUFFER_NB_FRAMES / (ptp_frame_size * chip->nb_playback_interrupts_per_period));
    }
    else if(substream->stream == SNDRV_PCM_STREAM_CAPTURE)
    {
        chip->current_alsa_capture_stride = snd_pcm_format_physical_width(format) >> 3;
        chip->current_capture_interrupt_idx = 0;

        /// Ravenna DSD always uses a rate of 352k with eventual zero padding to maintain a 32 bit alignment
        /// while DSD in ALSA uses a continuous 8, 16 or 32 bit aligned stream with at 352k, 176k or 88k
        /// so respective ring buffers might have different scale and size
        chip->nb_capture_interrupts_per_period = ((dsd_mode != 0)? (MR_ALSA_PTP_FRAME_RATE_FOR_DSD / rate) : 1);
        if(nbPeriods * chip->nb_capture_interrupts_per_period * ptp_frame_size != MR_ALSA_RINGBUFFER_NB_FRAMES)
            printk(KERN_INFO "mr_alsa_audio_pcm_hw_params (capture): nbPeriods (%u) differs from expected (%u)\n", nbPeriods, MR_ALSA_RINGBUFFER_NB_FRAMES / (ptp_frame_size * chip->nb_capture_interrupts_per_period));
    }

    if(bufferSize != nbPeriods * ptp_frame_size)
        printk(KERN_INFO "mr_alsa_audio_pcm_hw_params : bufferSize (%u) differs from expected (%u)\n", bufferSize, nbPeriods * ptp_frame_size);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0)
    err = snd_pcm_lib_alloc_vmalloc_buffer(substream, bufferBytes);
#endif

    spin_unlock_irq(&chip->lock);

    printk(KERN_DEBUG "mr_alsa_audio_pcm_hw_params done: rate=%d format=%d channels=%d period_size=%u, nb_periods=%u, buffer_bytes=%u\n", rate, format, nbCh, periodSize, nbPeriods, bufferBytes);
    return err;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0)
/// hw_free callback
/// This is called to release the resources allocated via hw_params. For example, releasing the buffer via
/// snd_pcm_lib_malloc_pages() is done by calling the following: snd_pcm_lib_free_pages(substream);
/// This function is always called before the close callback is called. Also, the callback may be called multiple
/// times, too. Keep track whether the resource was already released.
static int mr_alsa_audio_pcm_hw_free(struct snd_pcm_substream *substream)
{
    int err = 0;
    
    if (substream)
    {
        struct mr_alsa_audio_chip *chip = snd_pcm_substream_chip(substream);

        printk(KERN_DEBUG "entering mr_alsa_audio_pcm_hw_free (substream name=%s #%d) ...\n", substream->name, substream->number);
        spin_lock_irq(&chip->lock);
        err = snd_pcm_lib_free_vmalloc_buffer(substream);
        spin_unlock_irq(&chip->lock);
    }
    return err;
}
#endif



/* W6: the rate / period-size constraint lists are per chip
 * (chip->constraints_rates / chip->constraints_period_sizes, filled by
 * set_pcm_sample_rate) — the old g_constraints_* globals were rewritten
 * in place by every open of every chip and advertised all 8 Ravenna
 * rates on every PCM. */

/* DSD formats are advertised only on DSD-configured chips (W6). */
#ifdef SNDRV_PCM_FMTBIT_DSD_U8
#define MR_DSD_FMT_U8 SNDRV_PCM_FMTBIT_DSD_U8
#else
#define MR_DSD_FMT_U8 0
#endif
#ifdef SNDRV_PCM_FMTBIT_DSD_U16_BE
#define MR_DSD_FMT_U16_BE SNDRV_PCM_FMTBIT_DSD_U16_BE
#else
#define MR_DSD_FMT_U16_BE 0
#endif
#ifdef SNDRV_PCM_FMTBIT_DSD_U16_LE
#define MR_DSD_FMT_U16_LE SNDRV_PCM_FMTBIT_DSD_U16_LE
#else
#define MR_DSD_FMT_U16_LE 0
#endif
#ifdef SNDRV_PCM_FMTBIT_DSD_U32_BE
#define MR_DSD_FMT_U32_BE SNDRV_PCM_FMTBIT_DSD_U32_BE
#else
#define MR_DSD_FMT_U32_BE 0
#endif
#ifdef SNDRV_PCM_FMTBIT_DSD_U32_LE
#define MR_DSD_FMT_U32_LE SNDRV_PCM_FMTBIT_DSD_U32_LE
#else
#define MR_DSD_FMT_U32_LE 0
#endif
#define MR_ALSA_DSD_FMTBITS (MR_DSD_FMT_U8 | MR_DSD_FMT_U16_BE | MR_DSD_FMT_U16_LE | MR_DSD_FMT_U32_BE | MR_DSD_FMT_U32_LE)

static int mr_alsa_audio_hw_rule_rate_by_format( struct snd_pcm_hw_params *params,
                                             struct snd_pcm_hw_rule *rule)
{
    //struct mr_alsa_audio_chip *chip = rule->private;
    int ret = 0;
    struct snd_interval *r = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
    struct snd_mask *f = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);
    uint64_t fmask = f->bits[0] + ((uint64_t)f->bits[1] << 32);
    //uint32_t orig_min = r->min, orig_max = r->max;
    #ifdef SNDRV_PCM_FMTBIT_DSD_U8
    if (fmask == SNDRV_PCM_FMTBIT_DSD_U8)
    {
        struct snd_interval t;
        snd_interval_any(&t);
        t.min = t.max = 352800;
        t.integer = 1;
        ret = snd_interval_refine(r, &t);

    }
    #endif
    if (!(fmask & ~(
    #ifdef SNDRV_PCM_FMTBIT_DSD_U16_BE
        SNDRV_PCM_FMTBIT_DSD_U16_BE |
    #endif
    #ifdef SNDRV_PCM_FMTBIT_DSD_U16_LE
        SNDRV_PCM_FMTBIT_DSD_U16_LE |
    #endif
        0)))
    {
        const unsigned int rates[] = {176400, 352800};
        ret = snd_interval_list(r, ARRAY_SIZE(rates), rates, 0);

    }
    if (!(fmask & ~(
    #ifdef SNDRV_PCM_FMTBIT_DSD_U32_BE
        SNDRV_PCM_FMTBIT_DSD_U32_BE |
    #endif
    #ifdef SNDRV_PCM_FMTBIT_DSD_U32_LE
        SNDRV_PCM_FMTBIT_DSD_U32_LE |
    #endif
        0)))
    {
        const unsigned int rates[] = {88200, 176400, 352800};
        ret = snd_interval_list(r, ARRAY_SIZE(rates), rates, 0);

    }
    //printk("mr_alsa_audio_hw_rule_rate_by_format returns %d : [%u, %u] => [%u, %u]\n", ret, orig_min, orig_max, r->min, r->max);
    return ret;
}


static int mr_alsa_audio_hw_rule_period_size_by_rate(struct snd_pcm_hw_params *params,
                                                     struct snd_pcm_hw_rule *rule)
{
    struct mr_alsa_audio_chip *chip = rule->private;
    struct snd_interval *ps = hw_param_interval(params, SNDRV_PCM_HW_PARAM_PERIOD_SIZE);
    struct snd_interval *r = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
    struct snd_interval t;
    uint32_t minPTPFrameSize, maxPTPFrameSize;
    unsigned int sr_factor;

    snd_interval_any(&t);
    chip->mr_alsa_audio_ops->get_min_interrupts_frame_size(chip->ravenna_peer, &minPTPFrameSize);
    chip->mr_alsa_audio_ops->get_max_interrupts_frame_size(chip->ravenna_peer, &maxPTPFrameSize);

    if (r->min > 192000 && r->max <= 384000)
        sr_factor = 8;
    else if (r->min > 96000 && r->max <= 192000)
        sr_factor = 4;
    else if (r->min > 48000 && r->max <= 96000)
        sr_factor = 2;
    else
        sr_factor = 1;

    t.min = minPTPFrameSize * sr_factor;
    t.max = maxPTPFrameSize * sr_factor;
    t.integer = 1;

    return snd_interval_refine(ps, &t);
}

#if 0
static int mr_alsa_audio_hw_rule_period_nb_by_rate_and_format(  struct snd_pcm_hw_params *params,
                                                                struct snd_pcm_hw_rule *rule)
{
    //unsigned int nbPeriods;
    struct mr_alsa_audio_chip *chip = rule->private;
    struct snd_interval *pn = hw_param_interval(params, SNDRV_PCM_HW_PARAM_PERIODS);
    struct snd_interval *r = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
    struct snd_mask *f = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);
    uint64_t fmask = f->bits[0] + ((uint64_t)f->bits[1] << 32);
    struct snd_interval t;
    uint32_t minPTPFrameSize, maxPTPFrameSize;
    int ret = 0;
    //uint32_t orig_min = pn->min, orig_max = pn->max;
    snd_interval_any(&t);
    chip->mr_alsa_audio_ops->get_min_interrupts_frame_size(chip->ravenna_peer, &minPTPFrameSize);
    chip->mr_alsa_audio_ops->get_max_interrupts_frame_size(chip->ravenna_peer, &maxPTPFrameSize);
    if (r->min > 192000 && r->max <= 384000)
    {
        t.min = t.max = MR_ALSA_RINGBUFFER_NB_FRAMES / min(maxPTPFrameSize, (minPTPFrameSize * 8)); // 48
        t.integer = 1;
       // printk("mr_alsa_audio_hw_rule_period_nb_by_rate Period Nb interval for SR= [%u, %u] => %u\n", r->min, r->max, t.min);
        ret = snd_interval_refine(pn, &t);
    }
    else if (r->min > 96000 && r->max <= 192000)
    {
        uint32_t nbPeriods = MR_ALSA_RINGBUFFER_NB_FRAMES / min(maxPTPFrameSize, (minPTPFrameSize * 4));
        if (!(fmask & ~(
        #ifdef SNDRV_PCM_FMTBIT_DSD_U16_BE
            SNDRV_PCM_FMTBIT_DSD_U16_BE |
        #endif
        #ifdef SNDRV_PCM_FMTBIT_DSD_U16_LE
            SNDRV_PCM_FMTBIT_DSD_U16_LE |
        #endif
        #ifdef SNDRV_PCM_FMTBIT_DSD_U32_BE
            SNDRV_PCM_FMTBIT_DSD_U32_BE |
        #endif
        #ifdef SNDRV_PCM_FMTBIT_DSD_U32_LE
            SNDRV_PCM_FMTBIT_DSD_U32_LE |
        #endif
            0)))
            nbPeriods >>= 1;
        t.min = t.max = nbPeriods; //24
        t.integer = 1;
        //printk("mr_alsa_audio_hw_rule_period_nb_by_rate Period Nb interval for SR= [%u, %u] => %u\n", r->min, r->max, t.min);
        ret = snd_interval_refine(pn, &t);
    }
    else if (r->min > 48000 && r->max <= 96000)
    {
        uint32_t nbPeriods = MR_ALSA_RINGBUFFER_NB_FRAMES / min(maxPTPFrameSize, (minPTPFrameSize * 2));
        if (!(fmask & ~(
        #ifdef SNDRV_PCM_FMTBIT_DSD_U32_BE
            SNDRV_PCM_FMTBIT_DSD_U32_BE |
        #endif
        #ifdef SNDRV_PCM_FMTBIT_DSD_U32_LE
            SNDRV_PCM_FMTBIT_DSD_U32_LE |
        #endif
            0)))
            nbPeriods >>= 2;
        t.max = nbPeriods;
        t.min = 1;
        t.integer = 1;
        // printk("mr_alsa_audio_hw_rule_period_nb_by_rate Period Nb interval for SR= [%u, %u] => %u\n", r->min, r->max, nbPeriods);
        ret = snd_interval_refine(pn, &t);
    }
    else if (r->max < 64000)
    {
        t.max = MR_ALSA_RINGBUFFER_NB_FRAMES / minPTPFrameSize;
        t.min = 1;
        t.integer = 1;
        // printk("mr_alsa_audio_hw_rule_period_nb_by_rate Period Nb interval for SR= [%u, %u] => %u\n", r->min, r->max, t.max);
        ret = snd_interval_refine(pn, &t);
    }
    //printk("mr_alsa_audio_hw_rule_period_nb_by_rate_and_format returns %d : [%u, %u] => [%u, %u]\n", ret, orig_min, orig_max, pn->min, pn->max);
    return ret;
}
#endif



/// open callback
/// This is called when a pcm substream is opened.
/// At least, here you have to initialize the runtime->hw record.
/// You can allocate a private data in this callback.
/// If the hardware configuration needs more constraints, set the hardware constraints here, too.

static int mr_alsa_audio_pcm_open(struct snd_pcm_substream *substream)
{
    int ret = 0;
    struct mr_alsa_audio_chip *chip = snd_pcm_substream_chip(substream);
    struct snd_pcm_runtime *runtime = substream->runtime;
    uint32_t minPTPFrameSize, maxPTPFrameSize, ptp_frame_size;
    //uint32_t period_time_us_min, period_time_us_max;
    size_t period_bytes_min, period_bytes_max;
    unsigned int periods_min, periods_max;

    chip->mr_alsa_audio_ops->get_min_interrupts_frame_size(chip->ravenna_peer, &minPTPFrameSize);
    chip->mr_alsa_audio_ops->get_max_interrupts_frame_size(chip->ravenna_peer, &maxPTPFrameSize);

    //period_time_us_min = (minPTPFrameSize * 1000000) / 48000; // TODO finde max and Min according the Horus Frame size limitation
    //period_time_us_max = 1 + (minPTPFrameSize * 1000000) / 44100;

    printk(KERN_DEBUG "entering mr_alsa_audio_pcm_open (substream name=%s #%d) ...\n", substream->name, substream->number);
    /* W15: barrier against a concurrent in-place re-key (from another
     * substream's last close). Briefly takes+releases the manager's registry
     * lock so this open reads a fully-applied (rate, constraints), never a
     * half-swapped state. Cheap + uncontended in the common case. */
    if (chip->mr_alsa_audio_ops && chip->mr_alsa_audio_ops->registry_barrier)
        chip->mr_alsa_audio_ops->registry_barrier(chip->ravenna_peer);
    /* W6: this chip's OWN configured rate and tick frame govern the
     * open — the manager-wide rate is meaningless under per-PCM rates. */
    chip->current_rate = mr_alsa_audio_get_pcm_sample_rate(chip);
    chip->current_dsd = mr_alsa_audio_get_dsd_mode(chip->current_rate);

    ptp_frame_size = mr_alsa_audio_get_pcm_frame_size(chip);

    if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
    {
        /// retrieve the supported range of nb bits per sample based on mr_alsa_audio_pcm_hardware_playback.formats mask
        struct snd_interval t;
        struct snd_mask fmt_mask;
        unsigned int k;
        int err = 0;
        /* W6: customize a LOCAL copy of the template — the old code
         * rewrote the shared static per open (cross-chip race) and
         * advertised the full rate span on every chip. */
        struct snd_pcm_hardware hw = mr_alsa_audio_pcm_hardware_playback;
        if (chip->current_dsd == 0)
            hw.formats &= ~(u64)MR_ALSA_DSD_FMTBITS;
        if (chip->current_dsd != 0)
        {
            hw.rate_min = 88200;   /* DSD container rates */
            hw.rate_max = 352800;
        }
        else
        {
            hw.rate_min = hw.rate_max = chip->current_rate;
        }
        t.min = UINT_MAX;
        t.max = 0;
        t.openmin = 0;
        t.openmax = 0;
        t.empty = 1;
        snd_mask_none(&fmt_mask);
        fmt_mask.bits[0] = (u_int32_t)hw.formats;
        fmt_mask.bits[1] = (u_int32_t)(hw.formats >> 32);
        for (k = 0; k <= SNDRV_PCM_FORMAT_LAST; ++k)
        {
            int bits;
            if (! snd_mask_test(&fmt_mask, k))
                continue;
            bits = snd_pcm_format_physical_width(k);
            if (bits <= 0)
                continue; /* ignore invalid formats */
            if (t.min > (unsigned)bits)
                t.min = bits;
            if (t.max < (unsigned)bits)
                t.max = bits;
        }

        printk("mr_alsa_audio_pcm_open: playback format nb bits range: [%u, %u]\n", t.min, t.max);

        period_bytes_min = minPTPFrameSize * hw.channels_min * (t.min >> 3); // amount of data in bytes for min channels, smallest sample size in bytes, minimum period size
        period_bytes_max = maxPTPFrameSize * hw.channels_max * (t.max >> 3); // amount of data in bytes for max channels, largest sample size in bytes, maximum period size
        periods_min = 2;
        periods_max = MR_ALSA_RINGBUFFER_NB_FRAMES / maxPTPFrameSize;

        hw.period_bytes_min = period_bytes_min;
        hw.period_bytes_max = period_bytes_max;
        hw.periods_min = periods_min;
        hw.periods_max = periods_max;

        printk("mr_alsa_audio_pcm_open: playback period size range: [%zu, %zu], periods range: [%u, %u]\n",
              period_bytes_min, period_bytes_max, periods_min, periods_max);

        runtime->hw = hw;

        // TODO
        /*if (chip->capture_substream == NULL)
            mr_alsa_audio_stop_audio(chip);*/

        chip->playback_pid = current->pid;
        chip->playback_substream = substream;

        chip->current_alsa_playback_stride = snd_pcm_format_physical_width(SNDRV_PCM_FORMAT_S32_LE) >> 3;

        /// channels
        chip->mr_alsa_audio_ops->get_nb_outputs(chip->ravenna_peer, &chip->current_nboutputs);

        /// synchronizes controls values (Playback volume and switch)
        chip->mr_alsa_audio_ops->get_master_volume_value(chip->ravenna_peer, (int)substream->stream, &chip->current_playback_volume);
        err = chip->mr_alsa_audio_ops->get_master_switch_value(chip->ravenna_peer, (int)substream->stream, &chip->current_playback_switch);
        if(err != 0)
            printk(KERN_WARNING "mr_alsa_audio_pcm_open: get_master_switch_value error\n");
        /*else
            printk("mr_alsa_audio_pcm_open: get_master_switch_value returns %d\n", chip->current_playback_switch);*/
        /* 2026-06-10 hardware-test fix: master volume/switch controls are
         * registered on chip 0 ONLY (per-PCM controls land in W9), so these
         * pointers are NULL on chips 1+. The unguarded notify dereferenced
         * &NULL->id inside snd_ctl_notify on the FIRST playback open of a
         * dynamically added PCM -> oops holding the card control lock ->
         * hard lockup of every other PCM on the card. Same guard the
         * prepare path has always had. */
        if (chip->playback_volume_control)
            snd_ctl_notify(chip->card, SNDRV_CTL_EVENT_MASK_VALUE, &chip->playback_volume_control->id);
        if (chip->playback_switch_control)
            snd_ctl_notify(chip->card, SNDRV_CTL_EVENT_MASK_VALUE, &chip->playback_switch_control->id);
    }
    else if(substream->stream == SNDRV_PCM_STREAM_CAPTURE)
    {
         /// retrieve the supported range of nb bits per sample based on mr_alsa_audio_pcm_hardware_capture.formats mask
        struct snd_interval t;
        struct snd_mask fmt_mask;
        unsigned int k;
        /* W6: local template copy (see playback). */
        struct snd_pcm_hardware hw = mr_alsa_audio_pcm_hardware_capture;
        if (chip->current_dsd == 0)
            hw.formats &= ~(u64)MR_ALSA_DSD_FMTBITS;
        if (chip->current_dsd != 0)
        {
            hw.rate_min = 88200;   /* DSD container rates */
            hw.rate_max = 352800;
        }
        else
        {
            hw.rate_min = hw.rate_max = chip->current_rate;
        }
        t.min = UINT_MAX;
        t.max = 0;
        t.openmin = 0;
        t.openmax = 0;
        t.empty = 1;
        snd_mask_none(&fmt_mask);
        fmt_mask.bits[0] = (u_int32_t)hw.formats;
        fmt_mask.bits[1] = (u_int32_t)(hw.formats >> 32);
        for (k = 0; k <= SNDRV_PCM_FORMAT_LAST; ++k)
        {
            int bits;
            if (! snd_mask_test(&fmt_mask, k))
                continue;
            bits = snd_pcm_format_physical_width(k);
            if (bits <= 0)
                continue; /* ignore invalid formats */
            if (t.min > (unsigned)bits)
                t.min = bits;
            if (t.max < (unsigned)bits)
                t.max = bits;
        }
        printk("mr_alsa_audio_pcm_open: capture format nb bits range: [%u, %u]\n", t.min, t.max);

        period_bytes_min = minPTPFrameSize * hw.channels_min * (t.min >> 3); // amount of data in bytes for min channels, smallest sample size in bytes, minimum period size
        period_bytes_max = maxPTPFrameSize * hw.channels_max * (t.max >> 3); // amount of data in bytes for max channels, largest sample size in bytes, maximum period size
        periods_min = 2;
        periods_max = MR_ALSA_RINGBUFFER_NB_FRAMES / maxPTPFrameSize;

        printk("mr_alsa_audio_pcm_open: capture period size range: [%zu, %zu], periods range: [%u, %u]\n",
              period_bytes_min, period_bytes_max, periods_min, periods_max);

        hw.period_bytes_min = period_bytes_min;
        hw.period_bytes_max = period_bytes_max;
        hw.periods_min = periods_min;
        hw.periods_max = periods_max;

        runtime->hw = hw;
        chip->capture_pid = current->pid;
        chip->capture_substream = substream;

        chip->current_alsa_capture_stride = snd_pcm_format_physical_width(SNDRV_PCM_FORMAT_S32_LE) >> 3;

        /// channels
        chip->mr_alsa_audio_ops->get_nb_inputs(chip->ravenna_peer, &chip->current_nbinputs);
        //printk("mr_alsa_audio_pcm_open: NB OF INPUT %u\n", chip->current_nbinputs);
    }

    /// constraints stuff
    /// Sample rate supported list:
    ret = snd_pcm_hw_constraint_list(   runtime, 0,
                                        SNDRV_PCM_HW_PARAM_RATE,
                                        &chip->constraints_rates);
    if(ret < 0)
    {
        printk("mr_alsa_audio_pcm_open: Unsupported sample rate (%u Hz)\n", runtime->rate);
        printk("unsuccessfully leaving mr_alsa_audio_pcm_open...\n");
        return ret;
    }
    /// Sample rate constraint by format
    snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
                        mr_alsa_audio_hw_rule_rate_by_format, chip,
                        SNDRV_PCM_HW_PARAM_FORMAT, -1);

    ///Periods (W6: per-chip list, filled at rate publish — no per-open
    /// rewrite of shared state)

    ret = snd_pcm_hw_constraint_list(  runtime, 0,
                                       SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
                                       &chip->constraints_period_sizes);

    /// rules Period Size by Rate
    snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_SIZE,
                        mr_alsa_audio_hw_rule_period_size_by_rate, chip,
                        SNDRV_PCM_HW_PARAM_RATE, -1);

    printk("Current PTPFrame Size = %u, minPTPFrameSize = %u, maxPTPFrameSize = %u\n", 
        ptp_frame_size, minPTPFrameSize, maxPTPFrameSize);

    snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_SIZE, ptp_frame_size);

#if 0
    ///rules Nb Periods by Rate
    snd_pcm_hw_rule_add(runtime, 0, SNDRV_PCM_HW_PARAM_PERIODS,
                        mr_alsa_audio_hw_rule_period_nb_by_rate_and_format, chip,
                        SNDRV_PCM_HW_PARAM_RATE, SNDRV_PCM_HW_PARAM_FORMAT, -1);

    snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_PERIODS, (MR_ALSA_RINGBUFFER_NB_FRAMES >> 2) / maxPTPFrameSize);
#endif

    if(ret < 0)
    {
        printk("mr_alsa_audio_pcm_open: Unsupported period size (%lu smp)\n", runtime->period_size);
        printk("unsuccessfully leaving mr_alsa_audio_pcm_open...\n");
        return ret;
    }

    //snd_pcm_hw_constraint_minmax(runtime, SNDRV_PCM_HW_PARAM_PERIOD_TIME, period_time_us_min, period_time_us_max);

    return 0;
}


/// close callback
/// Obviously, this is called when a pcm substream is closed.
/// Any private instance for a pcm substream allocated in the open callback will be released here
static int mr_alsa_audio_pcm_close(struct snd_pcm_substream *substream)
{
    struct mr_alsa_audio_chip *chip = snd_pcm_substream_chip(substream);

    printk(KERN_DEBUG "entering mr_alsa_audio_pcm_close (substream name=%s #%d) ...\n", substream->name, substream->number);
    if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
    {
        spin_lock_irq(&chip->playback_lock);
        chip->playback_pid = -1;
        chip->playback_substream = NULL;
        spin_unlock_irq(&chip->playback_lock);
    }
    else if(substream->stream == SNDRV_PCM_STREAM_CAPTURE)
    {
        spin_lock_irq(&chip->capture_lock);
        chip->capture_pid = -1;
        chip->capture_substream = NULL;
        spin_unlock_irq(&chip->capture_lock);
    }

    /* W15: in-place re-rate latch. If this was the last close and the chip is
     * armed for a pending rate, apply the re-key now — the chip is idle, so the
     * (domain,rate) registry move is safe. set_pcm_rate runs in the manager
     * under the registry lock and (via set_pcm_sample_rate) clears pending_rate
     * + notifies the PCM Rate control. Done outside the chip spinlocks: the
     * re-key may sleep (hrtimer_cancel of a freed old entry). A racing reopen is
     * handled by registry_barrier in pcm_open. */
    if (chip->pending_rate &&
        chip->capture_substream == NULL && chip->playback_substream == NULL &&
        chip->mr_alsa_audio_ops && chip->mr_alsa_audio_ops->set_pcm_rate)
    {
        uint32_t target = chip->pending_rate;
        int err = chip->mr_alsa_audio_ops->set_pcm_rate(chip->ravenna_peer,
                                                        chip->global_pcm_id, target);
        if (err < 0)
            printk(KERN_WARNING "mr_alsa_audio_pcm_close: pcm_id %d in-place re-rate to %u failed: %d\n",
                   chip->global_pcm_id, target, err);
        else {
            printk(KERN_INFO "mr_alsa_audio_pcm_close: pcm_id %d re-rated in place to %u on last close\n",
                   chip->global_pcm_id, target);
            /* W15: tell the daemon the armed re-rate applied, so it re-attaches
             * the sink now (the netlink SetPCMRate path doesn't notify — its
             * command reply already informs the daemon). */
            if (chip->mr_alsa_audio_ops->notify_pcm_rate_applied)
                chip->mr_alsa_audio_ops->notify_pcm_rate_applied(
                    chip->ravenna_peer, chip->global_pcm_id, target);
        }
    }
    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////
static struct snd_pcm_ops mr_alsa_audio_pcm_playback_ops = {
    .open =     mr_alsa_audio_pcm_open,
    .close =    mr_alsa_audio_pcm_close,
    .ioctl =    snd_pcm_lib_ioctl,
    .hw_params =    mr_alsa_audio_pcm_hw_params,
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0)
    .hw_free =  mr_alsa_audio_pcm_hw_free,
#endif
    .prepare =  mr_alsa_audio_pcm_prepare,
    .trigger =  mr_alsa_audio_pcm_trigger,
    .pointer =  mr_alsa_audio_pcm_pointer,
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0)
    .page =     snd_pcm_lib_get_vmalloc_page
#endif
};

/////////////////////////////////////////////////////////////////////////////////////
static struct snd_pcm_ops mr_alsa_audio_pcm_capture_ops = {
    .open =     mr_alsa_audio_pcm_open,
    .close =    mr_alsa_audio_pcm_close,
    .ioctl =    snd_pcm_lib_ioctl,
    .hw_params =    mr_alsa_audio_pcm_hw_params,
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0)
    .hw_free =  mr_alsa_audio_pcm_hw_free,
#endif
    .prepare =  mr_alsa_audio_pcm_prepare,
    .trigger =  mr_alsa_audio_pcm_trigger,
    .pointer =  mr_alsa_audio_pcm_pointer,
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 12, 0)
    .page =     snd_pcm_lib_get_vmalloc_page
#endif
};


/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////


/* prototypes */
static int mr_alsa_audio_create_alsa_devices(struct snd_card *card,
                     struct mr_alsa_audio_chip *chip, int device_idx, int global_pcm_id);
static int mr_alsa_audio_create_pcm(struct snd_card *card,
                struct mr_alsa_audio_chip *chip, int device_idx);

//static int hdspm_set_toggle_setting(struct hdspm *hdspm, u32 regmask, int out);
static int mr_alsa_audio_set_defaults(struct mr_alsa_audio_chip *chip);
//static int hdspm_system_clock_mode(struct mr_alsa_audio_chip *chip;



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Controls interface
/// TODO
static int mr_alsa_audio_create_controls(   struct snd_card *card,
                                            struct mr_alsa_audio_chip *chip)
{
    int err = 0;
    /* No lock here. This runs during card creation, before snd_card_register
     * (deferred-register), so no userspace control .get/.put can race these
     * field inits. The previous spin_lock_irq was both unnecessary and unsafe:
     * get_master_{volume,switch}_value do a netlink round-trip to the daemon
     * that *sleeps* (wait_event_interruptible_timeout), and snd_ctl_add
     * allocates — sleeping under spin_lock_irq is "scheduling while atomic".
     * It was masked while chips were created at probe (the daemon wasn't
     * connected yet, so the K2U query short-circuited on daemon_pid_ == -1);
     * W10 multi-card creates chips after the daemon's Hello, so the query now
     * genuinely waits and tripped the BUG. */
    chip->mr_alsa_audio_ops->get_master_volume_value(chip->ravenna_peer, (int)SNDRV_PCM_STREAM_PLAYBACK, &chip->current_playback_volume);
    chip->playback_volume_control = snd_ctl_new1(&mr_alsa_audio_ctrl_output_gain, chip);
    err = snd_ctl_add(card, chip->playback_volume_control);
    if (err >= 0)
    {
        err = chip->mr_alsa_audio_ops->get_master_switch_value(chip->ravenna_peer, (int)SNDRV_PCM_STREAM_PLAYBACK, &chip->current_playback_switch);
        if(err != 0)

            printk("mr_alsa_audio_pcm_open: get_master_switch_value error\n");
        else
            printk(KERN_INFO "mr_alsa_audio_pcm_open: get_master_switch_value returns %d\n", chip->current_playback_switch);
        chip->playback_switch_control = snd_ctl_new1(&mr_alsa_audio_ctrl_output_switch, chip);
        err = snd_ctl_add(card, chip->playback_switch_control);
    }
    return err;
}

int mr_alsa_audio_set_defaults(struct mr_alsa_audio_chip *chip)
{
    // TODO
    return 0;
}

/*------------------------------------------------------------
   memory interface
 ------------------------------------------------------------*/
static int mr_alsa_audio_preallocate_memory(struct mr_alsa_audio_chip *chip)
{
    int err;
    unsigned int i;
    struct snd_pcm *pcm;
    size_t wanted;

    pcm = chip->pcm;
    wanted = mr_alsa_audio_pcm_hardware_playback.buffer_bytes_max * 4; // MR_ALSA_RINGBUFFER_NB_FRAMES * MR_ALSA_NB_CHANNELS_MAX * 4;

    chip->playback_buffer = vmalloc(wanted);
    if(!chip->playback_buffer)
    {
        err = -ENOMEM;
        printk(KERN_ERR "mr_alsa_audio_preallocate_memory: could not allocate playback buffer (%zd bytes vmalloc requested...\n", wanted);
        goto _failed;
    }
    printk("mr_alsa_audio_preallocate_memory: allocated playback buffer of %zd bytes vmalloc requested\n", wanted);
    memset(chip->playback_buffer, 0, wanted);

    wanted = mr_alsa_audio_pcm_hardware_capture.buffer_bytes_max * 4; // MR_ALSA_RINGBUFFER_NB_FRAMES * MR_ALSA_NB_CHANNELS_MAX * 4;

    chip->capture_buffer = vmalloc(wanted);
    if(!chip->capture_buffer)
    {
        err = -ENOMEM;
        printk(KERN_ERR "mr_alsa_audio_preallocate_memory: could not allocate capture buffer (%zd bytes vmalloc requested...\n", wanted);
        goto _failed;
    }
    printk("mr_alsa_audio_preallocate_memory: allocated capture buffer of %zd bytes vmalloc requested\n", wanted);
    memset(chip->capture_buffer, 0, wanted);
    for (i = 0; i < MR_ALSA_NB_CHANNELS_MAX; i++)
    {
        chip->capture_buffer_channels_map[i] = (void*)chip->capture_buffer + MR_ALSA_RINGBUFFER_NB_FRAMES * i * 4;
    }
    return 0;

_failed:

    printk(KERN_ERR "mr_alsa_audio_preallocate_memory: Could not preallocate %zd Bytes\n", wanted);
    return err;
}

static void mr_alsa_audio_free_preallocate_memory(struct mr_alsa_audio_chip *chip)
{
    if(chip->playback_buffer)
        vfree(chip->playback_buffer);
    chip->playback_buffer = NULL;

    if(chip->capture_buffer)
        vfree(chip->capture_buffer);
    chip->capture_buffer = NULL;

    memset(chip->capture_buffer_channels_map, 0x00, sizeof(chip->capture_buffer_channels_map));
}


//////////////////////////////////////////////////////////////////////////
/*
 * Stage 1 multi-PCM:
 *  - device_idx == 0 runs in probe(), BEFORE snd_card_register. The ALSA
 *    core's snd_card_register → snd_device_register_all walk will register
 *    this PCM, so we must NOT call snd_device_register ourselves here.
 *  - device_idx > 0 runs from MT_ALSA_Msg_AddPCM in netlink-RX context,
 *    AFTER snd_card_register has completed. The PCM is added to
 *    card->devices in SNDRV_DEV_BUILD state by snd_pcm_new but has no
 *    /dev/snd/pcmCxDy{p,c} chardev until snd_device_register runs it
 *    through dev_register → snd_register_device. We call it explicitly
 *    here, following the i2sbus_attach_codec / snd_emu8000_pcm_new
 *    in-tree precedent.
 *  Ordering follows i2sbus: ops → chmap → snd_device_register → managed
 *  buffer. private_data is set before ops so any open() racing the chmap
 *  add sees a populated chip pointer. set_managed_buffer_all is post-
 *  register because the substream then exists in its final form.
 */
static int mr_alsa_audio_create_pcm(struct snd_card *card,
                                    struct mr_alsa_audio_chip *chip,
                                    int device_idx)
{
    struct snd_pcm *pcm;
    int err;
    err = snd_pcm_new(card, CARD_NAME, device_idx, 1, 1, &pcm);
    if (err < 0)
        return err;

    chip->pcm = pcm;
    pcm->private_data = chip;
    /* W7: per-PCM device name (aplay -l) when the daemon supplied one via
     * AddPCM; otherwise the historical CARD_NAME. */
    if (chip->pcm_name[0])
        snprintf(pcm->name, sizeof(pcm->name), "%s %s", CARD_NAME, chip->pcm_name);
    else
        strcpy(pcm->name, CARD_NAME);

    snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &mr_alsa_audio_pcm_playback_ops);
    snd_pcm_add_chmap_ctls(pcm, SNDRV_PCM_STREAM_PLAYBACK, mr_alsa_audio_nadac_playback_ch_map, 8, 0, NULL);
    snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,  &mr_alsa_audio_pcm_capture_ops);
    pcm->info_flags = SNDRV_PCM_INFO_JOINT_DUPLEX;

    err = mr_alsa_audio_preallocate_memory(chip);
    if (err < 0)
    {
        printk(KERN_ERR "mr_alsa_audio_preallocate_memory failed...\n");
        return err;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0)
    /* 2026-06-09 review fix: attach managed buffers BEFORE the chardev
     * becomes visible. snd_device_register fires the uevent that lets
     * userspace open the node; a client whose open/hw_params landed in
     * the old ordering's window got runtime->dma_area == NULL and the
     * next TIC's copy NULL-dereffed in tasklet context. In-tree drivers
     * set managed buffers before registering. */
    snd_pcm_set_managed_buffer_all(pcm, SNDRV_DMA_TYPE_VMALLOC, NULL, 0, 0);
#endif

    /* Do NOT snd_device_register here. Every PCM is added to the card BEFORE
     * register_card (the deferred-register model: add_card -> add_pcm_to_card
     * x N -> register_card; add_pcm_to_card rejects an already-registered card,
     * and reconfiguration is recreate-card, never a live add). snd_pcm_new has
     * placed this device on card->devices in BUILD state; snd_card_register's
     * sweep registers EVERY such device (dev 0, 1, 2, ...) at once -- same as it
     * already does for device 0. The earlier per-PCM snd_device_register for
     * device_idx > 0 assumed a post-register dynamic add; on the not-yet-
     * registered card it failed with -ENOENT (pcmC<n>D1p kobject_add, no parent
     * registration yet) -- which is exactly why a 2nd PCM never worked. The
     * managed buffer is already attached above, before that card-wide sweep. */
    return err;
}

///////////////////////////////////////////////////////////////////////////////

static int mr_alsa_audio_create_alsa_devices(   struct snd_card *card,
                                                struct mr_alsa_audio_chip *chip,
                                                int device_idx,
                                                int global_pcm_id)
{
    int err;
    err = mr_alsa_audio_create_pcm(card, chip, device_idx); // will also allocate the capture and playback buffers once for all
    if (err < 0)
        return err;

    chip->playback_buffer_pos = 0;
    chip->playback_buffer_alsa_sac = 0;
    chip->playback_buffer_rav_sac = 0;
    chip->current_alsa_capture_stride = 0;
    chip->current_alsa_playback_stride = 0;
    chip->current_alsa_capture_format = -1;
    chip->current_alsa_playback_format = -1;

    chip->current_rate = -1;
    chip->current_dsd = 0;

    chip->nb_playback_interrupts_per_period = 1; /// default PCM case
    chip->current_playback_interrupt_idx = 0;

    chip->nb_capture_interrupts_per_period = 1;
    chip->current_capture_interrupt_idx = 0;

    chip->current_nbinputs = 0;
    chip->current_nboutputs = 0;
    //chip->ravenna_input_buffer_off = 0;
    //chip->ravenna_output_buffer_off = 0;

    chip->playback_pid = -1;
    chip->capture_pid = -1;
    chip->capture_substream = NULL;
    spin_lock_init(&chip->capture_lock);
    chip->playback_substream = NULL;
    spin_lock_init(&chip->playback_lock);

    chip->playback_volume_control = NULL;
    chip->playback_switch_control = NULL;
    chip->current_rate_control = NULL;
    chip->pending_rate = 0;
    chip->current_playback_volume = 0;
    chip->current_playback_switch = 1;


    /// creates controls (for NADAC)
    /* Stage 1 multi-PCM: master volume/switch are shared across all PCMs
     * and registered on the first chip only. Per-PCM controls land in
     * Stage 2 (per-group sample rates). */
    if (device_idx == 0)
    {
        err = mr_alsa_audio_create_controls(card, chip);
        if (err < 0)
            return err;
    }

    /* W15: a per-PCM read-only current-rate control on EVERY chip (unlike the
     * card-level master volume/switch above). Scoped per-PCM via id.device so
     * PCMs sharing a card don't collide. The chip's rate is already published
     * (add_pcm_to_card stashes it before chip_create), so the first read is
     * correct. */
    chip->current_rate_control = snd_ctl_new1(&mr_alsa_audio_ctrl_current_rate, chip);
    if (chip->current_rate_control == NULL)
        return -ENOMEM;
    chip->current_rate_control->id.device = device_idx;
    err = snd_ctl_add(card, chip->current_rate_control);
    if (err < 0)
        return err;

    /// sets callbacks for Ravenna Manager
    if(chip->ravenna_peer && chip->mr_alsa_audio_ops)
    {
        printk(KERN_INFO "Register ALSA driver into Ravenna Peer for pcm_id=%d...\n", global_pcm_id);
        /* 2026-06-11 review fix: attach can now fail legitimately (no
         * (domain, rate) timer entry). Swallowing it produced a
         * "successful" PCM with no chip slot and no clock — silent dead
         * audio (the trap would have armed for real when W11 multiplies
         * the registry keyspace). */
        err = chip->mr_alsa_audio_ops->register_alsa_driver(chip->ravenna_peer, &g_ravenna_manager_ops, (void*)chip, global_pcm_id);
        if (err < 0)
        {
            printk(KERN_ERR "register_alsa_driver failed for pcm_id=%d (err %d)\n", global_pcm_id, err);
            return err;
        }
        chip->mr_alsa_audio_ops->get_nb_inputs(chip->ravenna_peer, &chip->current_nbinputs);
        chip->mr_alsa_audio_ops->get_nb_outputs(chip->ravenna_peer, &chip->current_nboutputs);

        /* W6: per-chip period-size constraints were filled by
         * set_pcm_sample_rate during register_alsa_driver above — the
         * global array refresh is gone. */
    }
    else
    {
        printk(KERN_ERR "Register ALSA driver into Ravenna Peer FAILED (ravenna_peer = 0x%p, chip->mr_alsa_audio_ops = 0x%p)...\n", chip->ravenna_peer, chip->mr_alsa_audio_ops);
    }

    err = mr_alsa_audio_set_defaults(chip);
    if (err < 0)
    {
        printk(KERN_ERR "mr_alsa_audio_set_defaults failed...\n");
        return err;
    }

    return 0;
}

static int mr_alsa_audio_chip_create(   struct snd_card *card,
                                        struct mr_alsa_audio_chip *chip,
                                        void *ravenna_peer,
                                        struct alsa_ops *ops,
                                        int device_idx,
                                        int global_pcm_id)
{

    int ret = 0;
    spin_lock_init(&chip->lock);
    chip->card = card;
    chip->ravenna_peer = ravenna_peer;
    chip->mr_alsa_audio_ops = ops;
    chip->global_pcm_id = global_pcm_id; /* manager's global pcm_id — see struct comment */

    dev_dbg(card->dev, "create alsa devices.\n");
    ret = mr_alsa_audio_create_alsa_devices(card, chip, device_idx, global_pcm_id);
    if(ret < 0)
    {
        printk(KERN_ERR "mr_alsa_audio_create_alsa_devices failed.. \n");
        return ret;
    }
    return 0;
}
static int mr_alsa_audio_chip_free(struct mr_alsa_audio_chip* chip)
{
    mr_alsa_audio_free_preallocate_memory(chip);
    return 0;
}

/*
 * card->private_free callback. Runs from snd_card_do_free, AFTER
 * snd_device_free_all has run and freed every snd_pcm on the card via
 * snd_pcm_dev_free. Implication: at this point chip->pcm is a dangling
 * pointer for every chip (PCM 0's chip == card->private_data and every
 * extra chip alike). Don't dereference chip->pcm here, and don't add any
 * helper that does. Today mr_alsa_audio_chip_free only frees buffer
 * memory (mr_alsa_audio_free_preallocate_memory), so we're safe — but
 * any future field tracked off chip->pcm would UAF here.
 */
static void mr_alsa_audio_card_free(struct snd_card *card)
{
    struct mr_alsa_card *mc = mr_alsa_card_of(card);
    unsigned int i;
    if (!mc)
        return;
    /* W10 multi-card: the chips were already detached from the manager
     * (m_apALSAChip[] slot cleared, timer refcount released) by the teardown
     * path BEFORE snd_card_free — so the manager never resolves a freed chip.
     * Here we only reclaim each chip's buffers + struct. Their substreams are
     * gone (snd_device_free_all ran), so chip->pcm is dangling — never deref it. */
    for (i = 0; i < mc->chip_count; ++i)
    {
        if (!mc->chips[i])
            continue;
        mr_alsa_audio_chip_free(mc->chips[i]);
        kfree(mc->chips[i]);
        mc->chips[i] = NULL;
    }
    mc->chip_count = 0;
    mc->card = NULL;
    mc->registered = false;
}


/// probe callback
/// This is the constructor for the platform device (callback provided to and called by platform_driver_register()).
/// W10 multi-card: no cards are created here. The daemon drives card/PCM bringup
/// on demand (AddCard -> AddPCM x N -> RegisterCard); probe only validates that
/// the module is enabled, leaving the platform device as the lifecycle anchor.
static int mr_alsa_audio_chip_probe(struct platform_device *devptr)
{
    /* W10 multi-card: cards are created on demand by the daemon (AddCard),
     * not at probe. The platform device is only the module's lifecycle anchor
     * (the parent for each card's snd_card_new). */
    (void)devptr;
    if (!enable)
        return -ENOENT;
    return 0;
}

/* W10 multi-card lifecycle (daemon-driven; see audio_driver.h):
 *   add_card -> add_pcm_to_card x N -> register_card   (bringup / reconfigure)
 *   remove_card                                         (teardown)
 * A card's PCMs are all created BEFORE its snd_card_register (deferred-register),
 * so a card always appears to userspace with its full PCM set (PA/PW enumerate
 * a card's PCMs once, at card-detect). card_handle indexes g_cards[];
 * global_pcm_id is the manager's m_apALSAChip[] slot (distinct from the per-card
 * ALSA device index assigned 0..N within the card).
 */
int mr_alsa_audio_add_card(int card_handle, const char *id, uint8_t domain)
{
    struct snd_card *card;
    struct mr_alsa_card *mc;
    int err;
    /* domain is the card-level PTP clock domain; pinned 0 until W11 (logged below). */
    if (card_handle < 0 || card_handle >= MR_ALSA_MAX_CARDS)
    {
        printk(KERN_ERR "mr_alsa_audio_add_card: handle %d out of range [0..%d)\n",
               card_handle, MR_ALSA_MAX_CARDS);
        return -EINVAL;
    }
    mc = &g_cards[card_handle];
    if (mc->card)
    {
        printk(KERN_ERR "mr_alsa_audio_add_card: handle %d already in use\n", card_handle);
        return -EEXIST;
    }
    /* extra_size 0: private_data unused (private_free finds the card via
     * mr_alsa_card_of); index -1 auto-allocates a free system card slot. */
    err = snd_card_new(&g_device->dev, -1, (id && id[0]) ? id : NULL,
                       THIS_MODULE, 0, &card);
    if (err < 0)
    {
        printk(KERN_ERR "mr_alsa_audio_add_card: snd_card_new(handle=%d) failed: %d\n",
               card_handle, err);
        return err;
    }
    card->private_free = mr_alsa_audio_card_free;
    strscpy(card->driver, SND_MR_ALSA_AUDIO_DRIVER, sizeof(card->driver));
    strscpy(card->shortname, (id && id[0]) ? id : CARD_NAME, sizeof(card->shortname));
    strlcat(card->longname, card->shortname, sizeof(card->longname));
    snd_card_set_dev(card, &g_device->dev);
    mc->card = card;
    mc->chip_count = 0;
    mc->registered = false;
    mc->domain = domain;   /* W11: kept so the chip→tick-entry bind can key on it */
    dev_info(&g_device->dev, "mr_alsa_audio_add_card: card %d (%s) created, domain %u\n",
             card_handle, card->id, domain);
    return 0;
}

int mr_alsa_audio_add_pcm_to_card(int card_handle, int global_pcm_id,
                                  uint32_t sample_rate, const char *name)
{
    struct mr_alsa_card *mc;
    struct mr_alsa_audio_chip *chip;
    int device_idx, err;
    if (card_handle < 0 || card_handle >= MR_ALSA_MAX_CARDS || !g_cards[card_handle].card)
    {
        printk(KERN_ERR "mr_alsa_audio_add_pcm_to_card: invalid card handle %d\n", card_handle);
        return -EINVAL;
    }
    mc = &g_cards[card_handle];
    if (mc->registered)
    {
        printk(KERN_ERR "mr_alsa_audio_add_pcm_to_card: card %d already registered; PCMs must be added before RegisterCard\n",
               card_handle);
        return -EINVAL;
    }
    if (mc->chip_count >= ARRAY_SIZE(mc->chips))
    {
        printk(KERN_ERR "mr_alsa_audio_add_pcm_to_card: card %d is full\n", card_handle);
        return -ENOSPC;
    }
    device_idx = (int)mc->chip_count;
    chip = kzalloc(sizeof(*chip), GFP_KERNEL);
    if (!chip)
        return -ENOMEM;
    chip->dev = g_device;
    /* F3 packed pair: rate high 32 bits, frame_size 0 until attach derives it.
     * Stashed before chip_create runs register_alsa_driver -> attach (which
     * reads it and publishes the coherent (rate, frame_size) pair on the chip
     * before the manager's slot becomes visible to tick-path readers). */
    chip->pcm_rate_and_frame = ((uint64_t)sample_rate << 32);
    if (name)
    {
        strncpy(chip->pcm_name, name, sizeof(chip->pcm_name) - 1);
        chip->pcm_name[sizeof(chip->pcm_name) - 1] = '\0';
    }
    err = mr_alsa_audio_chip_create(mc->card, chip, g_ravenna_peer,
                                    g_mr_alsa_audio_ops, device_idx, global_pcm_id);
    /* Record the chip in the card's list whether create succeeded or not: once
     * chip_create ran, snd_pcm_new may have published it via private_data, so it
     * must live until card_free. On failure also unhook it from the manager
     * (no-op if attach didn't run). */
    mc->chips[device_idx] = chip;
    mc->chip_count = device_idx + 1;
    if (err < 0)
    {
        printk(KERN_ERR "mr_alsa_audio_add_pcm_to_card: chip_create(card=%d dev=%d pcm_id=%d) failed: %d\n",
               card_handle, device_idx, global_pcm_id, err);
        if (g_mr_alsa_audio_ops && g_mr_alsa_audio_ops->unregister_alsa_driver)
            g_mr_alsa_audio_ops->unregister_alsa_driver(g_ravenna_peer, chip);
        return err;
    }
    dev_info(&g_device->dev, "mr_alsa_audio_add_pcm_to_card: card %d (%s) dev %d pcm_id %d rate %u\n",
             card_handle, mc->card->id, device_idx, global_pcm_id, sample_rate);
    return 0;
}

/* W9 #14: store this chip's advisory ALSA latency. chip_ptr is the manager's
 * m_apALSAChip[] slot (== struct mr_alsa_audio_chip*), resolved by the caller
 * via get_chip_by_pcm_id(). Takes effect at the next prepare() (PCM open). */
int mr_alsa_audio_set_playout_delay(void *chip_ptr, int32_t delay)
{
    struct mr_alsa_audio_chip *chip = chip_ptr;
    if (!chip)
        return -EINVAL;
    chip->playout_delay = delay;
    return 0;
}

int mr_alsa_audio_set_capture_delay(void *chip_ptr, int32_t delay)
{
    struct mr_alsa_audio_chip *chip = chip_ptr;
    if (!chip)
        return -EINVAL;
    chip->capture_delay = delay;
    return 0;
}

int mr_alsa_audio_register_card(int card_handle)
{
    struct mr_alsa_card *mc;
    int err;
    if (card_handle < 0 || card_handle >= MR_ALSA_MAX_CARDS || !g_cards[card_handle].card)
        return -EINVAL;
    mc = &g_cards[card_handle];
    if (mc->registered)
        return 0; /* idempotent */
    err = snd_card_register(mc->card);
    if (err < 0)
    {
        printk(KERN_ERR "mr_alsa_audio_register_card: snd_card_register(handle=%d) failed: %d\n",
               card_handle, err);
        return err;
    }
    mc->registered = true;
    dev_info(&g_device->dev, "mr_alsa_audio_register_card: hw:%s registered (%u pcm%s)\n",
             mc->card->id, mc->chip_count, mc->chip_count == 1 ? "" : "s");
    return 0;
}

/* Tear down one card: detach its chips from the manager (clears each
 * m_apALSAChip[] slot + releases its (domain,rate) timer refcount) BEFORE
 * freeing the card, so the manager never resolves a freed chip (the fix for the
 * old unload-with-streams UAF). snd_card_free then disconnects + frees the
 * card; the per-card private_free (mr_alsa_audio_card_free) reclaims the chips
 * and clears the g_cards[] slot. */
static void mr_alsa_audio_teardown_card(struct mr_alsa_card *mc)
{
    unsigned int i;
    if (!mc || !mc->card)
        return;
    for (i = 0; i < mc->chip_count; ++i)
        if (mc->chips[i] && g_mr_alsa_audio_ops && g_mr_alsa_audio_ops->unregister_alsa_driver)
            g_mr_alsa_audio_ops->unregister_alsa_driver(g_ravenna_peer, mc->chips[i]);
    dev_info(&g_device->dev, "mr_alsa_audio_teardown_card: card %d (%s) torn down (%u pcm)\n",
             (int)(mc - g_cards), mc->card->id, mc->chip_count);
    snd_card_free(mc->card);
}

int mr_alsa_audio_remove_card(int card_handle)
{
    if (card_handle < 0 || card_handle >= MR_ALSA_MAX_CARDS || !g_cards[card_handle].card)
        return -EINVAL;
    mr_alsa_audio_teardown_card(&g_cards[card_handle]);
    return 0;
}

/* W10 multi-card: tear down every card this module created — the clean-slate
 * primitive. Used at module unload (chip_remove) and when a daemon session
 * resets everything (MT_ALSA_Msg_Reset with pcm_id < 0), so a restarting
 * daemon always (re)declares its cards onto an empty module rather than
 * colliding with the previous session's cards. teardown_card detaches each
 * chip from the manager before snd_card_free, so a concurrent tick never
 * resolves a freed chip. Empty slots are skipped, so this is a safe no-op on
 * a fresh module. */
void mr_alsa_audio_remove_all_cards(void)
{
    int i;
    for (i = 0; i < MR_ALSA_MAX_CARDS; ++i)
        mr_alsa_audio_teardown_card(&g_cards[i]);
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
static int mr_alsa_audio_chip_remove(struct platform_device *devptr)
#else
static void mr_alsa_audio_chip_remove(struct platform_device *devptr)
#endif
{
    (void)devptr;
    mr_alsa_audio_remove_all_cards();
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
    return 0;
#endif
}



static struct platform_driver mr_alsa_audio_driver = {
    .probe      = mr_alsa_audio_chip_probe,
    .remove     = mr_alsa_audio_chip_remove,
    .driver     = {
        .name   = SND_MR_ALSA_AUDIO_DRIVER,
        .pm = MR_ALSA_AUDIO_PM_OPS,
    },
};

static void mr_alsa_audio_unregister_all(void)
{
    if(g_device != NULL)
    {
        platform_device_unregister(g_device);
        g_device = NULL;
    }
    platform_driver_unregister(&mr_alsa_audio_driver);
}

/// entry point: should be called by module init
int mr_alsa_audio_card_init(void* ravenna_peer, struct alsa_ops *callbacks)
{

    int err, cards;
    struct platform_device *device = NULL;

    g_ravenna_peer = ravenna_peer;
    g_mr_alsa_audio_ops = callbacks;
    err = platform_driver_register(&mr_alsa_audio_driver);
    if (err < 0)
        return err;

    cards = 0;

    if (enable)
    {
        device = platform_device_register_simple(SND_MR_ALSA_AUDIO_DRIVER, 0, NULL, 0);
        if (!IS_ERR(device))
        {
            /* W10 multi-card: probe creates no card; success = the platform
             * anchor registered. Cards are added later via AddCard. */
            g_device = device;
            cards++;
        }
        else
        {
            printk(KERN_ERR "mr_alsa_audio_card_init: platform_device_register_simple failed..\n" );
        }
    }
    if (!cards)
    {
        printk(KERN_ERR "mr_alsa_audio: No Merging Ravenna audio driver enabled\n" );

        mr_alsa_audio_unregister_all();
        return -ENODEV;
    }

    return 0;
}

/// exit point: should be called by module exit
void mr_alsa_audio_card_exit(void)
{
    printk(KERN_INFO "entering mr_alsa_audio_card_exit..\n" );
    mr_alsa_audio_unregister_all();
    g_ravenna_peer = NULL;
    g_mr_alsa_audio_ops = NULL;
    printk(KERN_INFO "leaving mr_alsa_audio_card_exit..\n");
}
