/****************************************************************************
*
*  Module Name    : module_timer.c
*  Version        :
*
*  Abstract       : RAVENNA/AES67 ALSA LKM
*
*  Written by     : Baume Florian
*  Date           : 15/04/2016
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

#include <linux/atomic.h>
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/module.h>       /* for module_param, MODULE_PARM_DESC */
#include <linux/sched.h>
#include <linux/sched/types.h>  /* for sched_set_fifo() (kernel 5.9+) */
#include <linux/version.h>

#include "module_main.h"
#include "module_timer.h"

#define MAX_CATCHUP_ITERATIONS 4

/*
 * base_period_ is read by timer_callback (hardirq/kthread context) and
 * written by set_base_period (process/PTP context). Use WRITE_ONCE/READ_ONCE
 * to prevent torn reads on 32-bit architectures and compiler reordering.
 */
static uint64_t base_period_;
static uint64_t max_period_allowed;
static uint64_t min_period_allowed;

/*
 * stop_ controls the shutdown sequence. It is accessed from:
 *   - timer_callback (hardirq context, potentially different CPU)
 *   - audio_work_fn (kthread context)
 *   - start_clock_timer / stop_clock_timer / kill_clock_timer (process context)
 * Must use atomic_t for SMP visibility guarantees.
 */
static atomic_t stop_ = ATOMIC_INIT(0);

static struct hrtimer my_hrtimer_;
static struct kthread_worker *audio_worker;
static struct kthread_work audio_work;

static int audio_cpu_affinity = -1;
module_param(audio_cpu_affinity, int, 0444);
MODULE_PARM_DESC(audio_cpu_affinity, "CPU core to pin audio thread to (-1 = auto)");

static void audio_work_fn(struct kthread_work *work)
{
    uint64_t next_wakeup, now;
    int iterations = 0;

    if (atomic_read(&stop_))
        return;

    /*
     * The Ravenna manager's t_clock_timer() may request immediate
     * re-processing (next_wakeup <= now) when it is behind schedule.
     * We honor this but cap iterations to prevent unbounded spinning.
     */
    do {
        t_clock_timer(&next_wakeup);
        get_clock_time(&now);
    } while (!atomic_read(&stop_) && now >= next_wakeup &&
             ++iterations < MAX_CATCHUP_ITERATIONS);
}

static enum hrtimer_restart timer_callback(struct hrtimer *timer)
{
    struct kthread_worker *worker;

    if (atomic_read(&stop_))
        return HRTIMER_NORESTART;

    /* Read worker pointer with READ_ONCE to guard against concurrent
     * kill_clock_timer setting it to NULL after hrtimer_cancel. */
    worker = READ_ONCE(audio_worker);
    if (!worker)
        return HRTIMER_NORESTART;

    kthread_queue_work(worker, &audio_work);
    hrtimer_forward_now(timer, ns_to_ktime(READ_ONCE(base_period_)));
    return HRTIMER_RESTART;
}

int init_clock_timer(void)
{
    atomic_set(&stop_, 0);

    /* Create kthread worker — optionally pinned to a CPU */
    if (audio_cpu_affinity >= 0)
        audio_worker = kthread_create_worker_on_cpu(
            audio_cpu_affinity, 0, "ravenna-audio");
    else
        audio_worker = kthread_create_worker(0, "ravenna-audio");

    if (IS_ERR(audio_worker)) {
        printk(KERN_ERR "ravenna: failed to create audio worker thread\n");
        return PTR_ERR(audio_worker);
    }

    /* Set SCHED_FIFO with default RT priority (MAX_RT_PRIO/2).
     * sched_set_fifo() available since kernel 5.9. */
    sched_set_fifo(audio_worker->task);

    kthread_init_work(&audio_work, audio_work_fn);

    /* Initialize hrtimer.
     * On PREEMPT_RT: use HRTIMER_MODE_ABS (softirq-kthread context,
     *   safe for kthread_queue_work).
     * On non-RT: use HRTIMER_MODE_ABS_HARD (true hard IRQ, most
     *   deterministic wakeup). */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,15,0)
  #ifdef CONFIG_PREEMPT_RT
    hrtimer_setup(&my_hrtimer_, timer_callback,
                  CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
  #else
    hrtimer_setup(&my_hrtimer_, timer_callback,
                  CLOCK_MONOTONIC, HRTIMER_MODE_ABS_HARD);
  #endif
#else
  #ifdef CONFIG_PREEMPT_RT
    hrtimer_init(&my_hrtimer_, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
  #else
    hrtimer_init(&my_hrtimer_, CLOCK_MONOTONIC, HRTIMER_MODE_ABS_HARD);
  #endif
    my_hrtimer_.function = timer_callback;
#endif

    WRITE_ONCE(base_period_, 1000000); /* 1ms default (AES67 48 frames @ 48kHz) */
    set_base_period(1000000);

    printk(KERN_INFO "ravenna: audio kthread created (cpu=%d)\n",
           audio_cpu_affinity);
    return 0;
}

/*
 * kill_clock_timer — called from module_exit only.
 * Permanently tears down the timer and kthread worker.
 * After this, init_clock_timer must be called to use the timer again.
 *
 * Ordering: hrtimer_cancel serializes with any running timer_callback,
 * ensuring no new work is queued after cancel returns.
 * kthread_destroy_worker flushes any already-queued work before joining.
 */
void kill_clock_timer(void)
{
    atomic_set(&stop_, 1);
    smp_wmb(); /* ensure stop_ visible before cancelling timer */
    hrtimer_cancel(&my_hrtimer_);
    if (audio_worker) {
        kthread_destroy_worker(audio_worker);
        WRITE_ONCE(audio_worker, NULL);
    }
}

/*
 * start_clock_timer — (re)starts the periodic timer.
 * Called from StartAudioFrameTICTimer which always calls
 * StopAudioFrameTICTimer first, so stop_ will be 1.
 * We must clear stop_ before arming the timer.
 */
int start_clock_timer(void)
{
    uint64_t period = READ_ONCE(base_period_);
    ktime_t expiry;

    atomic_set(&stop_, 0);
    smp_wmb(); /* ensure stop_=0 visible before timer fires */

    /* Use a future absolute time to avoid an immediate spurious firing */
    expiry = ktime_add(ktime_get(), ns_to_ktime(period));

#ifdef CONFIG_PREEMPT_RT
    hrtimer_start(&my_hrtimer_, expiry, HRTIMER_MODE_ABS);
#else
    hrtimer_start(&my_hrtimer_, expiry, HRTIMER_MODE_ABS_HARD);
#endif
    return 0;
}

/*
 * stop_clock_timer — stops the periodic timer but keeps the kthread alive.
 * Called from StopAudioFrameTICTimer during normal operation (sample rate
 * changes, stream stop). The timer can be restarted via start_clock_timer.
 *
 * Note: kill_clock_timer may have already destroyed audio_worker during
 * module exit — guard with NULL check.
 */
void stop_clock_timer(void)
{
    atomic_set(&stop_, 1);
    smp_wmb(); /* ensure stop_ visible to timer_callback and audio_work_fn */
    hrtimer_cancel(&my_hrtimer_);
    if (audio_worker)
        kthread_flush_worker(audio_worker);
}

void get_clock_time(uint64_t *clock_time)
{
    *clock_time = (uint64_t)ktime_to_ns(ktime_get());
}

void set_base_period(uint64_t base_period)
{
    WRITE_ONCE(base_period_, base_period);
    WRITE_ONCE(min_period_allowed, base_period / 7);
    WRITE_ONCE(max_period_allowed, (base_period * 10) / 6);
    printk(KERN_INFO "ravenna: base period set to %llu ns\n",
           (unsigned long long)base_period);
}

void update_base_period(uint32_t tic_frame_size, uint32_t sample_rate)
{
    uint64_t period_ns;
    if (sample_rate == 0)
        return;
    period_ns = ((uint64_t)tic_frame_size * 1000000000ULL) / sample_rate;
    set_base_period(period_ns);
}
