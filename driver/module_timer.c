/****************************************************************************
*
*  Module Name    : module_timer.c
*  Version        :
*
*  Abstract       : RAVENNA/AES67 ALSA LKM
*
*  Written by     : Baume Florian
*  Date           : 15/04/2016
*  Modified by    : multi-pcm-stage1 fork
*  Date           : 06/2026
*  Modification   : multi-rate W5 — instance-based clock timers (one
*                   hrtimer per (domain, rate) registry entry) replacing
*                   the file-scope singleton. Rebased onto upstream
*                   bondagit-2.1 (367c166): CPU-pinning, future-absolute
*                   arming and READ/WRITE_ONCE period access adopted
*                   per-entry.
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

#include <linux/module.h>
#include <linux/interrupt.h> // for tasklet
#include <linux/wait.h>
#include <linux/cpumask.h>
#include <linux/smp.h>

#include "module_main.h"
#include "module_timer.h"

/* CPU pinning for the audio hrtimer(s) — from upstream 367c166. One global
 * affinity applied to every (domain,rate) entry; -1 = no pinning (the W5
 * default, i.e. plain ABS_SOFT). Distinct per-entry affinity is a future
 * refinement. */
static int audio_cpu_affinity = -1;
module_param(audio_cpu_affinity, int, 0444);
MODULE_PARM_DESC(audio_cpu_affinity, "CPU core to pin the audio hrtimers to (-1 for any CPU, default -1)");

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,0,0) && LINUX_VERSION_CODE < KERNEL_VERSION(6,15,0)
/* struct tasklet_hrtimer lives in module_timer.h (W5) so it can be embedded
 * per (domain,rate) entry in struct clock_timer. */
static inline
void tasklet_hrtimer_cancel(struct tasklet_hrtimer *ttimer)
{
	hrtimer_cancel(&ttimer->timer);
	tasklet_kill(&ttimer->tasklet);
}

static enum hrtimer_restart __hrtimer_tasklet_trampoline(struct hrtimer *timer)
{
	struct tasklet_hrtimer *ttimer =
		container_of(timer, struct tasklet_hrtimer, timer);
	tasklet_hi_schedule(&ttimer->tasklet);
	return HRTIMER_NORESTART;
}

static void __tasklet_hrtimer_trampoline(unsigned long data)
{
	struct tasklet_hrtimer *ttimer = (void *)data;
	enum hrtimer_restart restart;
	restart = ttimer->function(&ttimer->timer);
	if (restart != HRTIMER_NORESTART)
		hrtimer_restart(&ttimer->timer);
}

static void tasklet_hrtimer_init(struct tasklet_hrtimer *ttimer,
			  enum hrtimer_restart (*function)(struct hrtimer *),
			  clockid_t which_clock, enum hrtimer_mode mode)
{
	hrtimer_init(&ttimer->timer, which_clock, mode);
	ttimer->timer.function = __hrtimer_tasklet_trampoline;
	tasklet_init(&ttimer->tasklet, __tasklet_hrtimer_trampoline,
		     (unsigned long)ttimer);
	ttimer->function = function;
}

static inline
void tasklet_hrtimer_start(struct tasklet_hrtimer *ttimer, ktime_t time,
			   const enum hrtimer_mode mode)
{
	hrtimer_start(&ttimer->timer, time, mode);
}
#else
/* >=6.15: arm an entry's hrtimer ON the target CPU (upstream 367c166's
 * CPU-pinning mechanism), generalized to take the entry's own timer. */
struct start_clock_timer_info {
    struct hrtimer* timer;
    ktime_t expiry;
};

static void start_clock_timer_on_cpu(void *info)
{
    struct start_clock_timer_info *ti = info;

    hrtimer_start(ti->timer, ti->expiry,
                  audio_cpu_affinity != -1 ? HRTIMER_MODE_ABS_PINNED_SOFT : HRTIMER_MODE_ABS_SOFT);
}
#endif

static enum hrtimer_restart timer_callback(struct hrtimer *timer)
{
    int ret_overrun;
    ktime_t period;
    uint64_t next_wakeup;
    uint64_t now;
    struct clock_timer *ct;

#if LINUX_VERSION_CODE < KERNEL_VERSION(6,15,0)
    {
        struct tasklet_hrtimer *ttimer =
            container_of(timer, struct tasklet_hrtimer, timer);
        ct = container_of(ttimer, struct clock_timer, my_hrtimer_);
    }
#else
    ct = container_of(timer, struct clock_timer, my_hrtimer_);
#endif

    do
    {
        get_clock_time(&now);
        t_clock_timer_tick(ct->ctx, &next_wakeup, now);
        period = ktime_set(0, next_wakeup - now);

        if (now > next_wakeup)
        {
            //printk(KERN_INFO "Timer won't sleep, clock_timer is recall instantly\n");
            period = ktime_set(0, 0);
        }
        else if (ktime_to_ns(period) > READ_ONCE(ct->max_period_allowed) ||
                 ktime_to_ns(period) < READ_ONCE(ct->min_period_allowed))
        {
            //printk(KERN_INFO "Timer period out of range. Target period = %lld\n", ct->base_period_);
            if (ktime_to_ns(period) > (unsigned long)5E9L)
            {
                //printk(KERN_ERR "Timer period greater than 5s, set it to 1s!\n");
                period = ktime_set(0,((unsigned long)1E9L)); //1s
            }
        }

        if (READ_ONCE(ct->stop_))
        {
            return HRTIMER_NORESTART;
        }
    }
    while (ktime_to_ns(period) == 0); // this able to be rarely true

    ret_overrun = hrtimer_forward_now(timer, period);
    // comment it when running in VM
    /*if(ret_overrun > 1)
        printk(KERN_INFO "Timer overrun ! (%d times)\n", ret_overrun);*/
    return HRTIMER_RESTART;

}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,15,0)
static int validate_audio_cpu_affinity(int cpu)
{
    if (cpu == -1)
        return 0; /* -1 means no pinning (any CPU) */

    if (cpu < 0 || cpu >= nr_cpu_ids)
    {
        printk(KERN_ERR "MergingRavennaALSA: Invalid CPU core %d (valid range: 0-%d or -1 for any)",
               cpu, nr_cpu_ids - 1);
        return -EINVAL;
    }

    if (!cpu_online(cpu))
    {
        printk(KERN_WARNING "MergingRavennaALSA: CPU core %d is not online", cpu);
        return -EINVAL;
    }

    return 0;
}
#endif

int init_clock_timer(struct clock_timer* ct, void* ctx)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,15,0)
    int ret = validate_audio_cpu_affinity(audio_cpu_affinity);
    if (ret != 0)
    {
        printk("MergingRavennaALSA: Failed to validate CPU audio affinity parameter");
        return ret;
    }
#endif
    ct->stop_ = 0;
    ct->ctx = ctx;
#if LINUX_VERSION_CODE < KERNEL_VERSION(6,15,0)
    tasklet_hrtimer_init(&ct->my_hrtimer_, timer_callback, CLOCK_MONOTONIC/*_RAW*/, HRTIMER_MODE_ABS);
#else
    /* PINNED_SOFT when a CPU is requested, else plain ABS_SOFT (W5 default). */
    hrtimer_setup(&ct->my_hrtimer_, timer_callback, CLOCK_MONOTONIC/*_RAW*/,
        audio_cpu_affinity != -1 ? HRTIMER_MODE_ABS_PINNED_SOFT : HRTIMER_MODE_ABS_SOFT);
#endif
    set_base_period(ct, 1333333); // 1.3 ms until the owning entry sets its real cadence
    return 0;
}

int start_clock_timer(struct clock_timer* ct)
{
    uint64_t period = READ_ONCE(ct->base_period_);
    /* Future-absolute expiry (upstream 367c166): arming at ABS time
     * == base_period ns fires almost immediately; offset from now instead. */
    ktime_t expiry = ktime_add(ktime_get(), ns_to_ktime(period));

    WRITE_ONCE(ct->stop_, 0);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6,15,0)
    tasklet_hrtimer_start(&ct->my_hrtimer_, expiry, HRTIMER_MODE_ABS);
#else
    if (audio_cpu_affinity != -1)
    {
        struct start_clock_timer_info info = { .timer = &ct->my_hrtimer_, .expiry = expiry };
        smp_call_function_single(audio_cpu_affinity, start_clock_timer_on_cpu, &info, true);
    }
    else
    {
        hrtimer_start(&ct->my_hrtimer_, expiry, HRTIMER_MODE_ABS_SOFT);
    }
#endif
    return 0;
}

void stop_clock_timer(struct clock_timer* ct)
{
    /* 2026-06-11 review fix: the in-callback stop gate was dead code (no
     * writer). On <6.15, tasklet_hrtimer_cancel runs a pending tasklet to
     * completion — whose trampoline RE-ARMS the hrtimer via
     * hrtimer_restart — so "stop" could return with the timer alive:
     * fatal now that timers are per-entry and destructible. Setting
     * stop_ first makes the callback return HRTIMER_NORESTART. */
    WRITE_ONCE(ct->stop_, 1);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6,15,0)
    tasklet_hrtimer_cancel(&ct->my_hrtimer_);
#else
    hrtimer_cancel(&ct->my_hrtimer_);
#endif
}

void get_clock_time(uint64_t* clock_time)
{
    ktime_t kt_now;
    kt_now = ktime_get();
    *clock_time = (uint64_t)ktime_to_ns(kt_now);
}

/* W9 robustness (2026-06-16): hard bounds on the tick period so a degenerate
 * frame/rate — from a buggy or ABI-mismatched daemon — can never produce a
 * sub-100us "fire immediately forever" timer that storms a CPU. 100us sits
 * safely below the smallest VALID tick (48 frames @ 384k = 125us), so it never
 * clamps a legitimate cadence; the 5s ceiling matches the existing arm-time
 * guard. Defense in depth: update_base_period rejects degenerate inputs up
 * front, set_base_period floors as a backstop so the derived min/max window
 * can never be poisoned by a bad base. */
#define MR_TIC_MIN_PERIOD_NS   100000ULL       /* 100 us */
#define MR_TIC_MAX_PERIOD_NS   5000000000ULL   /* 5 s    */
#define MR_TIC_MAX_SANE_RATE   1000000U        /* 1 MHz; valid tick rates top out at 384k */
#define MR_TIC_MAX_SANE_FRAME  8192U           /* generous; real max_tic_frame_size <= 1024 */

void set_base_period(struct clock_timer* ct, uint64_t base_period)
{
    if (base_period < MR_TIC_MIN_PERIOD_NS)
        base_period = MR_TIC_MIN_PERIOD_NS;
    else if (base_period > MR_TIC_MAX_PERIOD_NS)
        base_period = MR_TIC_MAX_PERIOD_NS;
    WRITE_ONCE(ct->base_period_, base_period);
    WRITE_ONCE(ct->min_period_allowed, base_period / 7);
    WRITE_ONCE(ct->max_period_allowed, (base_period * 10) / 6);
    printk(KERN_INFO "Base period set to %llu ns\n", (unsigned long long)base_period);
}

void update_base_period(struct clock_timer* ct, uint32_t tic_frame_size, uint32_t sample_rate)
{
    uint64_t period_ns;
    if (sample_rate == 0 || sample_rate > MR_TIC_MAX_SANE_RATE ||
        tic_frame_size == 0 || tic_frame_size > MR_TIC_MAX_SANE_FRAME)
    {
        printk(KERN_WARNING "update_base_period: refusing degenerate frame=%u rate=%u (period unchanged)\n",
               tic_frame_size, sample_rate);
        return;
    }
    period_ns = ((uint64_t)tic_frame_size * 1000000000ULL) / sample_rate;
    set_base_period(ct, period_ns);
}
