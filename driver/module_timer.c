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
*                   the file-scope singleton
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

#include <linux/interrupt.h> // for tasklet
#include <linux/wait.h>

#include "module_main.h"
#include "module_timer.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,0,0) && LINUX_VERSION_CODE < KERNEL_VERSION(6,15,0)

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
        else if (ktime_to_ns(period) > ct->max_period_allowed || ktime_to_ns(period) < ct->min_period_allowed)
        {
            //printk(KERN_INFO "Timer period out of range: %lld [ms]. Target period = %lld\n", ktime_to_ns(period) / 1000000, ct->base_period_ / 1000000);
            if (ktime_to_ns(period) > (unsigned long)5E9L)
            {
                //printk(KERN_ERR "Timer period greater than 5s, set it to 1s!\n");
                period = ktime_set(0,((unsigned long)1E9L)); //1s
            }
        }

        if(READ_ONCE(ct->stop_))
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

int init_clock_timer(struct clock_timer* ct, void* ctx)
{
    ct->stop_ = 0;
    ct->ctx = ctx;
#if LINUX_VERSION_CODE < KERNEL_VERSION(6,15,0)
    tasklet_hrtimer_init(&ct->my_hrtimer_, timer_callback, CLOCK_MONOTONIC/*_RAW*/, HRTIMER_MODE_ABS);
#else
    hrtimer_setup(&ct->my_hrtimer_, timer_callback, CLOCK_MONOTONIC/*_RAW*/, HRTIMER_MODE_ABS_SOFT);
#endif
    set_base_period(ct, 1333333); // 1.3 ms until the owning entry sets its real cadence
    return 0;
}

int start_clock_timer(struct clock_timer* ct)
{
    ktime_t period = ktime_set(0, ct->base_period_);
    WRITE_ONCE(ct->stop_, 0);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6,15,0)
    tasklet_hrtimer_start(&ct->my_hrtimer_, period, HRTIMER_MODE_ABS);
#else
    hrtimer_start(&ct->my_hrtimer_, period, HRTIMER_MODE_ABS_SOFT);
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

void set_base_period(struct clock_timer* ct, uint64_t base_period)
{
    ct->base_period_ = base_period;
    ct->min_period_allowed = ct->base_period_ / 7;
    ct->max_period_allowed = (ct->base_period_ * 10) / 6;
    printk(KERN_INFO "Base period set to %lld ns\n", ct->base_period_);
}
