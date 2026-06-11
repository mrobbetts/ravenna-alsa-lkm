/****************************************************************************
*
*  Module Name    : module_timer.h
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

#ifndef MODULE_TIMER_H_INCLUDED
#define MODULE_TIMER_H_INCLUDED

#include <linux/version.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#include "MTAL_stdint.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(6,15,0)
#include <linux/interrupt.h> // for tasklet
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,0,0) && LINUX_VERSION_CODE < KERNEL_VERSION(6,15,0)
/* removed from the kernel in 5.0; reintroduced here so the pre-6.15
 * tasklet-based timer path keeps working (moved from module_timer.c so
 * the struct can be embedded in struct clock_timer) */
struct tasklet_hrtimer {
	struct hrtimer		timer;
	struct tasklet_struct	tasklet;
	enum hrtimer_restart	(*function)(struct hrtimer *);
};
#endif

/*
 * Multi-rate W5: one clock_timer per (domain, rate) registry entry. The
 * old file-scope singletons (base period, clamp window, stop flag) are
 * per-instance fields; ctx carries the owning registry entry back into
 * the callback (t_clock_timer_tick).
 */
struct clock_timer {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6,15,0)
	struct tasklet_hrtimer my_hrtimer_;
#else
	struct hrtimer my_hrtimer_;
#endif
	uint64_t base_period_;
	uint64_t max_period_allowed;
	uint64_t min_period_allowed;
	int stop_;
	void* ctx;
};

#if defined(__cplusplus)
extern "C"
{
#endif // defined(__cplusplus)
extern int init_clock_timer(struct clock_timer* ct, void* ctx);
extern int start_clock_timer(struct clock_timer* ct);
extern void stop_clock_timer(struct clock_timer* ct);

extern void set_base_period(struct clock_timer* ct, uint64_t base_period);

extern void get_clock_time(uint64_t* clock_time);
#if defined(__cplusplus)
}
#endif // defined(__cplusplus)

#endif // MODULE_TIMER_H_INCLUDED
