// SPDX-License-Identifier: GPL-2.0
/*
 *  Kernel timekeeping code and accessor functions. Based on code from
 *  timer.c, moved in commit 8524070b7982.
 */
#include <linux/timekeeper_internal.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/kobject.h>
#include <linux/percpu.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/nmi.h>
#include <linux/sched.h>
#include <linux/sched/loadavg.h>
#include <linux/sched/clock.h>
#include <linux/syscore_ops.h>
#include <linux/clocksource.h>
#include <linux/jiffies.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/tick.h>
#include <linux/stop_machine.h>
#include <linux/pvclock_gtod.h>
#include <linux/compiler.h>
#include <linux/audit.h>
#include <linux/random.h>

#include <vdso/auxclock.h>

#include "tick-internal.h"
#include "ntp_internal.h"
#include "timekeeping_internal.h"

#define TK_CLEAR_NTP		(1 << 0)
#define TK_CLOCK_WAS_SET	(1 << 1)

#define TK_UPDATE_ALL		(TK_CLEAR_NTP | TK_CLOCK_WAS_SET)

enum timekeeping_adv_mode {
	/* Update timekeeper when a tick has passed */
	TK_ADV_TICK,

	/* Update timekeeper on a direct frequency change */
	TK_ADV_FREQ
};

/*
 * The most important data for readout fits into a single 64 byte
 * cache line.
 */
struct tk_data {
	seqcount_raw_spinlock_t	seq;
	struct timekeeper	timekeeper;
	struct timekeeper	shadow_timekeeper;
	raw_spinlock_t		lock;
} ____cacheline_aligned;

static struct tk_data timekeeper_data[TIMEKEEPERS_MAX];

/* The core timekeeper */
#define tk_core		(timekeeper_data[TIMEKEEPER_CORE])

#ifdef CONFIG_POSIX_AUX_CLOCKS
static inline bool tk_get_aux_ts64(unsigned int tkid, struct timespec64 *ts)
{
	return ktime_get_aux_ts64(CLOCK_AUX + tkid - TIMEKEEPER_AUX_FIRST, ts);
}

static inline bool tk_is_aux(const struct timekeeper *tk)
{
	return tk->id >= TIMEKEEPER_AUX_FIRST && tk->id <= TIMEKEEPER_AUX_LAST;
}
#else
static inline bool tk_get_aux_ts64(unsigned int tkid, struct timespec64 *ts)
{
	return false;
}

static inline bool tk_is_aux(const struct timekeeper *tk)
{
	return false;
}
#endif

/* flag for if timekeeping is suspended */
int __read_mostly timekeeping_suspended;

/**
 * struct tk_fast - NMI safe timekeeper
 * @seq:	Sequence counter for protecting updates. The lowest bit
 *		is the index for the tk_read_base array
 * @base:	tk_read_base array. Access is indexed by the lowest bit of
 *		@seq.
 *
 * See @update_fast_timekeeper() below.
 */
struct tk_fast {
	seqcount_latch_t	seq;
	struct tk_read_base	base[2];
};

/* Suspend-time cycles value for halted fast timekeeper. */
static u64 cycles_at_suspend;

static u64 dummy_clock_read(struct clocksource *cs)
{
	if (timekeeping_suspended)
		return cycles_at_suspend;
	return local_clock();
}

static struct clocksource dummy_clock = {
	.read = dummy_clock_read,
};

/*
 * Boot time initialization which allows local_clock() to be utilized
 * during early boot when clocksources are not available. local_clock()
 * returns nanoseconds already so no conversion is required, hence mult=1
 * and shift=0. When the first proper clocksource is installed then
 * the fast time keepers are updated with the correct values.
 */
#define FAST_TK_INIT						\
	{							\
		.clock		= &dummy_clock,			\
		.mask		= CLOCKSOURCE_MASK(64),		\
		.mult		= 1,				\
		.shift		= 0,				\
	}

static struct tk_fast tk_fast_mono ____cacheline_aligned = {
	.seq     = SEQCNT_LATCH_ZERO(tk_fast_mono.seq),
	.base[0] = FAST_TK_INIT,
	.base[1] = FAST_TK_INIT,
};

static struct tk_fast tk_fast_raw  ____cacheline_aligned = {
	.seq     = SEQCNT_LATCH_ZERO(tk_fast_raw.seq),
	.base[0] = FAST_TK_INIT,
	.base[1] = FAST_TK_INIT,
};

#ifdef CONFIG_POSIX_AUX_CLOCKS
static __init void tk_aux_setup(void);
static void tk_aux_update_clocksource(void);
static void tk_aux_advance(void);
#else
static inline void tk_aux_setup(void) { }
static inline void tk_aux_update_clocksource(void) { }
static inline void tk_aux_advance(void) { }
#endif

unsigned long timekeeper_lock_irqsave(void)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&tk_core.lock, flags);
	return flags;
}

void timekeeper_unlock_irqrestore(unsigned long flags)
{
	raw_spin_unlock_irqrestore(&tk_core.lock, flags);
}

/*
 * Multigrain timestamps require tracking the latest fine-grained timestamp
 * that has been issued, and never returning a coarse-grained timestamp that is
 * earlier than that value.
 *
 * mg_floor represents the latest fine-grained time that has been handed out as
 * a file timestamp on the system. This is tracked as a monotonic ktime_t, and
 * converted to a realtime clock value on an as-needed basis.
 *
 * Maintaining mg_floor ensures the multigrain interfaces never issue a
 * timestamp earlier than one that has been previously issued.
 *
 * The exception to this rule is when there is a backward realtime clock jump. If
 * such an event occurs, a timestamp can appear to be earlier than a previous one.
 */
static __cacheline_aligned_in_smp atomic64_t mg_floor;

static inline void tk_normalize_xtime(struct timekeeper *tk)
{
	while (tk->tkr_mono.xtime_nsec >= ((u64)NSEC_PER_SEC << tk->tkr_mono.shift)) {
		tk->tkr_mono.xtime_nsec -= (u64)NSEC_PER_SEC << tk->tkr_mono.shift;
		tk->xtime_sec++;
	}
	while (tk->tkr_raw.xtime_nsec >= ((u64)NSEC_PER_SEC << tk->tkr_raw.shift)) {
		tk->tkr_raw.xtime_nsec -= (u64)NSEC_PER_SEC << tk->tkr_raw.shift;
		tk->raw_sec++;
	}
}

static inline struct timespec64 tk_xtime(const struct timekeeper *tk)
{
	struct timespec64 ts;

	ts.tv_sec = tk->xtime_sec;
	ts.tv_nsec = (long)(tk->tkr_mono.xtime_nsec >> tk->tkr_mono.shift);
	return ts;
}

static inline struct timespec64 tk_xtime_coarse(const struct timekeeper *tk)
{
	struct timespec64 ts;

	ts.tv_sec = tk->xtime_sec;
	ts.tv_nsec = tk->coarse_nsec;
	return ts;
}

/*
 * Update the nanoseconds part for the coarse time keepers. They can't rely
 * on xtime_nsec because xtime_nsec could be adjusted by a small negative
 * amount when the multiplication factor of the clock is adjusted, which
 * could cause the coarse clocks to go slightly backwards. See
 * timekeeping_apply_adjustment(). Thus we keep a separate copy for the coarse
 * clockids which only is updated when the clock has been set or  we have
 * accumulated time.
 */
static inline void tk_update_coarse_nsecs(struct timekeeper *tk)
{
	tk->coarse_nsec = tk->tkr_mono.xtime_nsec >> tk->tkr_mono.shift;
}

static void tk_set_xtime(struct timekeeper *tk, const struct timespec64 *ts)
{
	tk->xtime_sec = ts->tv_sec;
	tk->tkr_mono.xtime_nsec = (u64)ts->tv_nsec << tk->tkr_mono.shift;
	tk_update_coarse_nsecs(tk);
}

static void tk_xtime_add(struct timekeeper *tk, const struct timespec64 *ts)
{
	tk->xtime_sec += ts->tv_sec;
	tk->tkr_mono.xtime_nsec += (u64)ts->tv_nsec << tk->tkr_mono.shift;
	tk_normalize_xtime(tk);
	tk_update_coarse_nsecs(tk);
}

static void tk_set_wall_to_mono(struct timekeeper *tk, struct timespec64 wtm)
{
	struct timespec64 tmp;

	/*
	 * Verify consistency of: offset_real = -wall_to_monotonic
	 * before modifying anything
	 */
	set_normalized_timespec64(&tmp, -tk->wall_to_monotonic.tv_sec,
					-tk->wall_to_monotonic.tv_nsec);
	WARN_ON_ONCE(tk->offs_real != timespec64_to_ktime(tmp));
	tk->wall_to_monotonic = wtm;
	set_normalized_timespec64(&tmp, -wtm.tv_sec, -wtm.tv_nsec);
	/* Paired with READ_ONCE() in ktime_mono_to_any() */
	WRITE_ONCE(tk->offs_real, timespec64_to_ktime(tmp));
	WRITE_ONCE(tk->offs_tai, ktime_add(tk->offs_real, ktime_set(tk->tai_offset, 0)));
}

static inline void tk_update_sleep_time(struct timekeeper *tk, ktime_t delta)
{
	/* Paired with READ_ONCE() in ktime_mono_to_any() */
	WRITE_ONCE(tk->offs_boot, ktime_add(tk->offs_boot, delta));
	/*
	 * Timespec representation for VDSO update to avoid 64bit division
	 * on every update.
	 */
	tk->monotonic_to_boot = ktime_to_timespec64(tk->offs_boot);
}

/*
 * tk_clock_read - atomic clocksource read() helper
 *
 * This helper is necessary to use in the read paths because, while the
 * seqcount ensures we don't return a bad value while structures are updated,
 * it doesn't protect from potential crashes. There is the possibility that
 * the tkr's clocksource may change between the read reference, and the
 * clock reference passed to the read function.  This can cause crashes if
 * the wrong clocksource is passed to the wrong read function.
 * This isn't necessary to use when holding the tk_core.lock or doing
 * a read of the fast-timekeeper tkrs (which is protected by its own locking
 * and update logic).
 */
static inline u64 tk_clock_read(const struct tk_read_base *tkr)
{
	struct clocksource *clock = READ_ONCE(tkr->clock);

	return clock->read(clock);
}

/**
 * tk_setup_internals - Set up internals to use clocksource clock.
 *
 * @tk:		The target timekeeper to setup.
 * @clock:		Pointer to clocksource.
 *
 * Calculates a fixed cycle/nsec interval for a given clocksource/adjustment
 * pair and interval request.
 *
 * Unless you're the timekeeping code, you should not be using this!
 */
static void tk_setup_internals(struct timekeeper *tk, struct clocksource *clock)
{
	u64 interval;
	u64 tmp, ntpinterval;
	struct clocksource *old_clock;

	++tk->cs_was_changed_seq;
	old_clock = tk->tkr_mono.clock;
	tk->tkr_mono.clock = clock;
	tk->tkr_mono.mask = clock->mask;
	tk->tkr_mono.cycle_last = tk_clock_read(&tk->tkr_mono);

	tk->tkr_raw.clock = clock;
	tk->tkr_raw.mask = clock->mask;
	tk->tkr_raw.cycle_last = tk->tkr_mono.cycle_last;

	/* Do the ns -> cycle conversion first, using original mult */
	tmp = NTP_INTERVAL_LENGTH;
	tmp <<= clock->shift;
	ntpinterval = tmp;
	tmp += clock->mult/2;
	do_div(tmp, clock->mult);
	if (tmp == 0)
		tmp = 1;

	interval = (u64) tmp;
	tk->cycle_interval = interval;

	/* Go back from cycles -> shifted ns */
	tk->xtime_interval = interval * clock->mult;
	tk->xtime_remainder = ntpinterval - tk->xtime_interval;
	tk->raw_interval = interval * clock->mult;

	 /* if changing clocks, convert xtime_nsec shift units */
	if (old_clock) {
		int shift_change = clock->shift - old_clock->shift;
		if (shift_change < 0) {
			tk->tkr_mono.xtime_nsec >>= -shift_change;
			tk->tkr_raw.xtime_nsec >>= -shift_change;
		} else {
			tk->tkr_mono.xtime_nsec <<= shift_change;
			tk->tkr_raw.xtime_nsec <<= shift_change;
		}
	}

	tk->tkr_mono.shift = clock->shift;
	tk->tkr_raw.shift = clock->shift;

	tk->ntp_error = 0;
	tk->ntp_error_shift = NTP_SCALE_SHIFT - clock->shift;
	tk->ntp_tick = ntpinterval << tk->ntp_error_shift;

	/*
	 * The timekeeper keeps its own mult values for the currently
	 * active clocksource. These value will be adjusted via NTP
	 * to counteract clock drifting.
	 */
	tk->tkr_mono.mult = clock->mult;
	tk->tkr_raw.mult = clock->mult;
	tk->ntp_err_mult = 0;
	tk->skip_second_overflow = 0;
}

/* Timekeeper helper functions. */
static noinline u64 delta_to_ns_safe(const struct tk_read_base *tkr, u64 delta)
{
	return mul_u64_u32_add_u64_shr(delta, tkr->mult, tkr->xtime_nsec, tkr->shift);
}

static inline u64 timekeeping_cycles_to_ns(const struct tk_read_base *tkr, u64 cycles)
{
	/* Calculate the delta since the last update_wall_time() */
	u64 mask = tkr->mask, delta = (cycles - tkr->cycle_last) & mask;

	/*
	 * This detects both negative motion and the case where the delta
	 * overflows the multiplication with tkr->mult.
	 */
	if (unlikely(delta > tkr->clock->max_cycles)) {
		/*
		 * Handle clocksource inconsistency between CPUs to prevent
		 * time from going backwards by checking for the MSB of the
		 * mask being set in the delta.
		 */
		if (delta & ~(mask >> 1))
			return tkr->xtime_nsec >> tkr->shift;

		return delta_to_ns_safe(tkr, delta);
	}

	return ((delta * tkr->mult) + tkr->xtime_nsec) >> tkr->shift;
}

static __always_inline u64 timekeeping_get_ns(const struct tk_read_base *tkr)
{
	return timekeeping_cycles_to_ns(tkr, tk_clock_read(tkr));
}

/**
 * update_fast_timekeeper - Update the fast and NMI safe monotonic timekeeper.
 * @tkr: Timekeeping readout base from which we take the update
 * @tkf: Pointer to NMI safe timekeeper
 *
 * We want to use this from any context including NMI and tracing /
 * instrumenting the timekeeping code itself.
 *
 * Employ the latch technique; see @write_seqcount_latch.
 *
 * So if a NMI hits the update of base[0] then it will use base[1]
 * which is still consistent. In the worst case this can result is a
 * slightly wrong timestamp (a few nanoseconds). See
 * @ktime_get_mono_fast_ns.
 */
static void update_fast_timekeeper(const struct tk_read_base *tkr,
				   struct tk_fast *tkf)
{
	struct tk_read_base *base = tkf->base;

	/* Force readers off to base[1] */
	write_seqcount_latch_begin(&tkf->seq);

	/* Update base[0] */
	memcpy(base, tkr, sizeof(*base));

	/* Force readers back to base[0] */
	write_seqcount_latch(&tkf->seq);

	/* Update base[1] */
	memcpy(base + 1, base, sizeof(*base));

	write_seqcount_latch_end(&tkf->seq);
}

static __always_inline u64 __ktime_get_fast_ns(struct tk_fast *tkf)
{
	struct tk_read_base *tkr;
	unsigned int seq;
	u64 now;

	do {
		seq = read_seqcount_latch(&tkf->seq);
		tkr = tkf->base + (seq & 0x01);
		now = ktime_to_ns(tkr->base);
		now += timekeeping_get_ns(tkr);
	} while (read_seqcount_latch_retry(&tkf->seq, seq));

	return now;
}

/**
 * ktime_get_mono_fast_ns - Fast NMI safe access to clock monotonic
 *
 * This timestamp is not guaranteed to be monotonic across an update.
 * The timestamp is calculated by:
 *
 *	now = base_mono + clock_delta * slope
 *
 * So if the update lowers the slope, readers who are forced to the
 * not yet updated second array are still using the old steeper slope.
 *
 * tmono
 * ^
 * |    o  n
 * |   o n
 * |  u
 * | o
 * |o
 * |12345678---> reader order
 *
 * o = old slope
 * u = update
 * n = new slope
 *
 * So reader 6 will observe time going backwards versus reader 5.
 *
 * While other CPUs are likely to be able to observe that, the only way
 * for a CPU local observation is when an NMI hits in the middle of
 * the update. Timestamps taken from that NMI context might be ahead
 * of the following timestamps. Callers need to be aware of that and
 * deal with it.
 */
u64 notrace ktime_get_mono_fast_ns(void)
{
	return __ktime_get_fast_ns(&tk_fast_mono);
}
EXPORT_SYMBOL_GPL(ktime_get_mono_fast_ns);

/**
 * ktime_get_raw_fast_ns - Fast NMI safe access to clock monotonic raw
 *
 * Contrary to ktime_get_mono_fast_ns() this is always correct because the
 * conversion factor is not affected by NTP/PTP correction.
 */
u64 notrace ktime_get_raw_fast_ns(void)
{
	return __ktime_get_fast_ns(&tk_fast_raw);
}
EXPORT_SYMBOL_GPL(ktime_get_raw_fast_ns);

/**
 * ktime_get_boot_fast_ns - NMI safe and fast access to boot clock.
 *
 * To keep it NMI safe since we're accessing from tracing, we're not using a
 * separate timekeeper with updates to monotonic clock and boot offset
 * protected with seqcounts. This has the following minor side effects:
 *
 * (1) Its possible that a timestamp be taken after the boot offset is updated
 * but before the timekeeper is updated. If this happens, the new boot offset
 * is added to the old timekeeping making the clock appear to update slightly
 * earlier:
 *    CPU 0                                        CPU 1
 *    timekeeping_inject_sleeptime64()
 *    __timekeeping_inject_sleeptime(tk, delta);
 *                                                 timestamp();
 *    timekeeping_update_staged(tkd, TK_CLEAR_NTP...);
 *
 * (2) On 32-bit systems, the 64-bit boot offset (tk->offs_boot) may be
 * partially updated.  Since the tk->offs_boot update is a rare event, this
 * should be a rare occurrence which postprocessing should be able to handle.
 *
 * The caveats vs. timestamp ordering as documented for ktime_get_mono_fast_ns()
 * apply as well.
 */
u64 notrace ktime_get_boot_fast_ns(void)
{
	struct timekeeper *tk = &tk_core.timekeeper;

	return (ktime_get_mono_fast_ns() + ktime_to_ns(data_race(tk->offs_boot)));
}
EXPORT_SYMBOL_GPL(ktime_get_boot_fast_ns);

/**
 * ktime_get_tai_fast_ns - NMI safe and fast access to tai clock.
 *
 * The same limitations as described for ktime_get_boot_fast_ns() apply. The
 * mono time and the TAI offset are not read atomically which may yield wrong
 * readouts. However, an update of the TAI offset is an rare event e.g., caused
 * by settime or adjtimex with an offset. The user of this function has to deal
 * with the possibility of wrong timestamps in post processing.
 */
u64 notrace ktime_get_tai_fast_ns(void)
{
	struct timekeeper *tk = &tk_core.timekeeper;

	return (ktime_get_mono_fast_ns() + ktime_to_ns(data_race(tk->offs_tai)));
}
EXPORT_SYMBOL_GPL(ktime_get_tai_fast_ns);

/**
 * ktime_get_real_fast_ns: - NMI safe and fast access to clock realtime.
 *
 * See ktime_get_mono_fast_ns() for documentation of the time stamp ordering.
 */
u64 ktime_get_real_fast_ns(void)
{
	struct tk_fast *tkf = &tk_fast_mono;
	struct tk_read_base *tkr;
	u64 baser, delta;
	unsigned int seq;

	do {
		seq = raw_read_seqcount_latch(&tkf->seq);
		tkr = tkf->base + (seq & 0x01);
		baser = ktime_to_ns(tkr->base_real);
		delta = timekeeping_get_ns(tkr);
	} while (raw_read_seqcount_latch_retry(&tkf->seq, seq));

	return baser + delta;
}
EXPORT_SYMBOL_GPL(ktime_get_real_fast_ns);

/**
 * halt_fast_timekeeper - Prevent fast timekeeper from accessing clocksource.
 * @tk: Timekeeper to snapshot.
 *
 * It generally is unsafe to access the clocksource after timekeeping has been
 * suspended, so take a snapshot of the readout base of @tk and use it as the
 * fast timekeeper's readout base while suspended.  It will return the same
 * number of cycles every time until timekeeping is resumed at which time the
 * proper readout base for the fast timekeeper will be restored automatically.
 */
static void halt_fast_timekeeper(const struct timekeeper *tk)
{
	static struct tk_read_base tkr_dummy;
	const struct tk_read_base *tkr = &tk->tkr_mono;

	memcpy(&tkr_dummy, tkr, sizeof(tkr_dummy));
	cycles_at_suspend = tk_clock_read(tkr);
	tkr_dummy.clock = &dummy_clock;
	tkr_dummy.base_real = tkr->base + tk->offs_real;
	update_fast_timekeeper(&tkr_dummy, &tk_fast_mono);

	tkr = &tk->tkr_raw;
	memcpy(&tkr_dummy, tkr, sizeof(tkr_dummy));
	tkr_dummy.clock = &dummy_clock;
	update_fast_timekeeper(&tkr_dummy, &tk_fast_raw);
}

static RAW_NOTIFIER_HEAD(pvclock_gtod_chain);

static void update_pvclock_gtod(struct timekeeper *tk, bool was_set)
{
	raw_notifier_call_chain(&pvclock_gtod_chain, was_set, tk);
}

/**
 * pvclock_gtod_register_notifier - register a pvclock timedata update listener
 * @nb: Pointer to the notifier block to register
 */
int pvclock_gtod_register_notifier(struct notifier_block *nb)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	int ret;

	guard(raw_spinlock_irqsave)(&tk_core.lock);
	ret = raw_notifier_chain_register(&pvclock_gtod_chain, nb);
	update_pvclock_gtod(tk, true);

	return ret;
}
EXPORT_SYMBOL_GPL(pvclock_gtod_register_notifier);

/**
 * pvclock_gtod_unregister_notifier - unregister a pvclock
 * timedata update listener
 * @nb: Pointer to the notifier block to unregister
 */
int pvclock_gtod_unregister_notifier(struct notifier_block *nb)
{
	guard(raw_spinlock_irqsave)(&tk_core.lock);
	return raw_notifier_chain_unregister(&pvclock_gtod_chain, nb);
}
EXPORT_SYMBOL_GPL(pvclock_gtod_unregister_notifier);

/*
 * tk_update_leap_state - helper to update the next_leap_ktime
 */
static inline void tk_update_leap_state(struct timekeeper *tk)
{
	tk->next_leap_ktime = ntp_get_next_leap(tk->id);
	if (tk->next_leap_ktime != KTIME_MAX)
		/* Convert to monotonic time */
		tk->next_leap_ktime = ktime_sub(tk->next_leap_ktime, tk->offs_real);
}

/*
 * Leap state update for both shadow and the real timekeeper
 * Separate to spare a full memcpy() of the timekeeper.
 */
static void tk_update_leap_state_all(struct tk_data *tkd)
{
	write_seqcount_begin(&tkd->seq);
	tk_update_leap_state(&tkd->shadow_timekeeper);
	tkd->timekeeper.next_leap_ktime = tkd->shadow_timekeeper.next_leap_ktime;
	write_seqcount_end(&tkd->seq);
}

/*
 * Update the ktime_t based scalar nsec members of the timekeeper
 */
static inline void tk_update_ktime_data(struct timekeeper *tk)
{
	u64 seconds;
	u32 nsec;

	/*
	 * The xtime based monotonic readout is:
	 *	nsec = (xtime_sec + wtm_sec) * 1e9 + wtm_nsec + now();
	 * The ktime based monotonic readout is:
	 *	nsec = base_mono + now();
	 * ==> base_mono = (xtime_sec + wtm_sec) * 1e9 + wtm_nsec
	 */
	seconds = (u64)(tk->xtime_sec + tk->wall_to_monotonic.tv_sec);
	nsec = (u32) tk->wall_to_monotonic.tv_nsec;
	tk->tkr_mono.base = ns_to_ktime(seconds * NSEC_PER_SEC + nsec);

	/*
	 * The sum of the nanoseconds portions of xtime and
	 * wall_to_monotonic can be greater/equal one second. Take
	 * this into account before updating tk->ktime_sec.
	 */
	nsec += (u32)(tk->tkr_mono.xtime_nsec >> tk->tkr_mono.shift);
	if (nsec >= NSEC_PER_SEC)
		seconds++;
	tk->ktime_sec = seconds;

	/* Update the monotonic raw base */
	tk->tkr_raw.base = ns_to_ktime(tk->raw_sec * NSEC_PER_SEC);
}

/*
 * Restore the shadow timekeeper from the real timekeeper.
 */
static void timekeeping_restore_shadow(struct tk_data *tkd)
{
	lockdep_assert_held(&tkd->lock);
	memcpy(&tkd->shadow_timekeeper, &tkd->timekeeper, sizeof(tkd->timekeeper));
}

static void timekeeping_update_from_shadow(struct tk_data *tkd, unsigned int action)
{
	struct timekeeper *tk = &tkd->shadow_timekeeper;

	lockdep_assert_held(&tkd->lock);

	/*
	 * Block out readers before running the updates below because that
	 * updates VDSO and other time related infrastructure. Not blocking
	 * the readers might let a reader see time going backwards when
	 * reading from the VDSO after the VDSO update and then reading in
	 * the kernel from the timekeeper before that got updated.
	 */
	write_seqcount_begin(&tkd->seq);

	if (action & TK_CLEAR_NTP) {
		tk->ntp_error = 0;
		ntp_clear(tk->id);
	}

	tk_update_leap_state(tk);
	tk_update_ktime_data(tk);
	tk->tkr_mono.base_real = tk->tkr_mono.base + tk->offs_real;

	if (tk->id == TIMEKEEPER_CORE) {
		update_vsyscall(tk);
		update_pvclock_gtod(tk, action & TK_CLOCK_WAS_SET);

		update_fast_timekeeper(&tk->tkr_mono, &tk_fast_mono);
		update_fast_timekeeper(&tk->tkr_raw,  &tk_fast_raw);
	} else if (tk_is_aux(tk)) {
		vdso_time_update_aux(tk);
	}

	if (action & TK_CLOCK_WAS_SET)
		tk->clock_was_set_seq++;

	/*
	 * Update the real timekeeper.
	 *
	 * We could avoid this memcpy() by switching pointers, but that has
	 * the downside that the reader side does not longer benefit from
	 * the cacheline optimized data layout of the timekeeper and requires
	 * another indirection.
	 */
	memcpy(&tkd->timekeeper, tk, sizeof(*tk));
	write_seqcount_end(&tkd->seq);
}

/**
 * timekeeping_forward_now - update clock to the current time
 * @tk:		Pointer to the timekeeper to update
 *
 * Forward the current clock to update its state since the last call to
 * update_wall_time(). This is useful before significant clock changes,
 * as it avoids having to deal with this time offset explicitly.
 */
static void timekeeping_forward_now(struct timekeeper *tk)
{
	u64 cycle_now, delta;

	cycle_now = tk_clock_read(&tk->tkr_mono);
	delta = clocksource_delta(cycle_now, tk->tkr_mono.cycle_last, tk->tkr_mono.mask,
				  tk->tkr_mono.clock->max_raw_delta);
	tk->tkr_mono.cycle_last = cycle_now;
	tk->tkr_raw.cycle_last  = cycle_now;

	while (delta > 0) {
		u64 max = tk->tkr_mono.clock->max_cycles;
		u64 incr = delta < max ? delta : max;

		tk->tkr_mono.xtime_nsec += incr * tk->tkr_mono.mult;
		tk->tkr_raw.xtime_nsec += incr * tk->tkr_raw.mult;
		tk_normalize_xtime(tk);
		delta -= incr;
	}
	tk_update_coarse_nsecs(tk);
}

/**
 * ktime_get_real_ts64 - Returns the time of day in a timespec64.
 * @ts:		pointer to the timespec to be set
 *
 * Returns the time of day in a timespec64 (WARN if suspended).
 */
void ktime_get_real_ts64(struct timespec64 *ts)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	unsigned int seq;
	u64 nsecs;

	WARN_ON(timekeeping_suspended);

	do {
		seq = read_seqcount_begin(&tk_core.seq);

		ts->tv_sec = tk->xtime_sec;
		nsecs = timekeeping_get_ns(&tk->tkr_mono);

	} while (read_seqcount_retry(&tk_core.seq, seq));

	ts->tv_nsec = 0;
	timespec64_add_ns(ts, nsecs);
}
EXPORT_SYMBOL(ktime_get_real_ts64);

ktime_t ktime_get(void)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	unsigned int seq;
	ktime_t base;
	u64 nsecs;

	WARN_ON(timekeeping_suspended);

	do {
		seq = read_seqcount_begin(&tk_core.seq);
		base = tk->tkr_mono.base;
		nsecs = timekeeping_get_ns(&tk->tkr_mono);

	} while (read_seqcount_retry(&tk_core.seq, seq));

	return ktime_add_ns(base, nsecs);
}
EXPORT_SYMBOL_GPL(ktime_get);

u32 ktime_get_resolution_ns(void)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	unsigned int seq;
	u32 nsecs;

	WARN_ON(timekeeping_suspended);

	do {
		seq = read_seqcount_begin(&tk_core.seq);
		nsecs = tk->tkr_mono.mult >> tk->tkr_mono.shift;
	} while (read_seqcount_retry(&tk_core.seq, seq));

	return nsecs;
}
EXPORT_SYMBOL_GPL(ktime_get_resolution_ns);

static ktime_t *offsets[TK_OFFS_MAX] = {
	[TK_OFFS_REAL]	= &tk_core.timekeeper.offs_real,
	[TK_OFFS_BOOT]	= &tk_core.timekeeper.offs_boot,
	[TK_OFFS_TAI]	= &tk_core.timekeeper.offs_tai,
};

ktime_t ktime_get_with_offset(enum tk_offsets offs)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	unsigned int seq;
	ktime_t base, *offset = offsets[offs];
	u64 nsecs;

	WARN_ON(timekeeping_suspended);

	do {
		seq = read_seqcount_begin(&tk_core.seq);
		base = ktime_add(tk->tkr_mono.base, *offset);
		nsecs = timekeeping_get_ns(&tk->tkr_mono);

	} while (read_seqcount_retry(&tk_core.seq, seq));

	return ktime_add_ns(base, nsecs);

}
EXPORT_SYMBOL_GPL(ktime_get_with_offset);

ktime_t ktime_get_coarse_with_offset(enum tk_offsets offs)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	ktime_t base, *offset = offsets[offs];
	unsigned int seq;
	u64 nsecs;

	WARN_ON(timekeeping_suspended);

	do {
		seq = read_seqcount_begin(&tk_core.seq);
		base = ktime_add(tk->tkr_mono.base, *offset);
		nsecs = tk->coarse_nsec;

	} while (read_seqcount_retry(&tk_core.seq, seq));

	return ktime_add_ns(base, nsecs);
}
EXPORT_SYMBOL_GPL(ktime_get_coarse_with_offset);

/**
 * ktime_mono_to_any() - convert monotonic time to any other time
 * @tmono:	time to convert.
 * @offs:	which offset to use
 */
ktime_t ktime_mono_to_any(ktime_t tmono, enum tk_offsets offs)
{
	ktime_t *offset = offsets[offs];
	unsigned int seq;
	ktime_t tconv;

	if (IS_ENABLED(CONFIG_64BIT)) {
		/*
		 * Paired with WRITE_ONCE()s in tk_set_wall_to_mono() and
		 * tk_update_sleep_time().
		 */
		return ktime_add(tmono, READ_ONCE(*offset));
	}

	do {
		seq = read_seqcount_begin(&tk_core.seq);
		tconv = ktime_add(tmono, *offset);
	} while (read_seqcount_retry(&tk_core.seq, seq));

	return tconv;
}
EXPORT_SYMBOL_GPL(ktime_mono_to_any);

/**
 * ktime_get_raw - Returns the raw monotonic time in ktime_t format
 */
ktime_t ktime_get_raw(void)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	unsigned int seq;
	ktime_t base;
	u64 nsecs;

	do {
		seq = read_seqcount_begin(&tk_core.seq);
		base = tk->tkr_raw.base;
		nsecs = timekeeping_get_ns(&tk->tkr_raw);

	} while (read_seqcount_retry(&tk_core.seq, seq));

	return ktime_add_ns(base, nsecs);
}
EXPORT_SYMBOL_GPL(ktime_get_raw);

/**
 * ktime_get_ts64 - get the monotonic clock in timespec64 format
 * @ts:		pointer to timespec variable
 *
 * The function calculates the monotonic clock from the realtime
 * clock and the wall_to_monotonic offset and stores the result
 * in normalized timespec64 format in the variable pointed to by @ts.
 */
void ktime_get_ts64(struct timespec64 *ts)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	struct timespec64 tomono;
	unsigned int seq;
	u64 nsec;

	WARN_ON(timekeeping_suspended);

	do {
		seq = read_seqcount_begin(&tk_core.seq);
		ts->tv_sec = tk->xtime_sec;
		nsec = timekeeping_get_ns(&tk->tkr_mono);
		tomono = tk->wall_to_monotonic;

	} while (read_seqcount_retry(&tk_core.seq, seq));

	ts->tv_sec += tomono.tv_sec;
	ts->tv_nsec = 0;
	timespec64_add_ns(ts, nsec + tomono.tv_nsec);
}
EXPORT_SYMBOL_GPL(ktime_get_ts64);

/**
 * ktime_get_seconds - Get the seconds portion of CLOCK_MONOTONIC
 *
 * Returns the seconds portion of CLOCK_MONOTONIC with a single non
 * serialized read. tk->ktime_sec is of type 'unsigned long' so this
 * works on both 32 and 64 bit systems. On 32 bit systems the readout
 * covers ~136 years of uptime which should be enough to prevent
 * premature wrap arounds.
 */
time64_t ktime_get_seconds(void)
{
	struct timekeeper *tk = &tk_core.timekeeper;

	WARN_ON(timekeeping_suspended);
	return tk->ktime_sec;
}
EXPORT_SYMBOL_GPL(ktime_get_seconds);

/**
 * ktime_get_real_seconds - Get the seconds portion of CLOCK_REALTIME
 *
 * Returns the wall clock seconds since 1970.
 *
 * For 64bit systems the fast access to tk->xtime_sec is preserved. On
 * 32bit systems the access must be protected with the sequence
 * counter to provide "atomic" access to the 64bit tk->xtime_sec
 * value.
 */
time64_t ktime_get_real_seconds(void)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	time64_t seconds;
	unsigned int seq;

	if (IS_ENABLED(CONFIG_64BIT))
		return tk->xtime_sec;

	do {
		seq = read_seqcount_begin(&tk_core.seq);
		seconds = tk->xtime_sec;

	} while (read_seqcount_retry(&tk_core.seq, seq));

	return seconds;
}
EXPORT_SYMBOL_GPL(ktime_get_real_seconds);

/**
 * __ktime_get_real_seconds - Unprotected access to CLOCK_REALTIME seconds
 *
 * The same as ktime_get_real_seconds() but without the sequence counter
 * protection. This function is used in restricted contexts like the x86 MCE
 * handler and in KGDB. It's unprotected on 32-bit vs. concurrent half
 * completed modification and only to be used for such critical contexts.
 *
 * Returns: Racy snapshot of the CLOCK_REALTIME seconds value
 */
noinstr time64_t __ktime_get_real_seconds(void)
{
	struct timekeeper *tk = &tk_core.timekeeper;

	return tk->xtime_sec;
}

/**
 * ktime_get_snapshot - snapshots the realtime/monotonic raw clocks with counter
 * @systime_snapshot:	pointer to struct receiving the system time snapshot
 */
void ktime_get_snapshot(struct system_time_snapshot *systime_snapshot)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	unsigned int seq;
	ktime_t base_raw;
	ktime_t base_real;
	ktime_t base_boot;
	u64 nsec_raw;
	u64 nsec_real;
	u64 now;

	WARN_ON_ONCE(timekeeping_suspended);

	do {
		seq = read_seqcount_begin(&tk_core.seq);
		now = tk_clock_read(&tk->tkr_mono);
		systime_snapshot->cs_id = tk->tkr_mono.clock->id;
		systime_snapshot->cs_was_changed_seq = tk->cs_was_changed_seq;
		systime_snapshot->clock_was_set_seq = tk->clock_was_set_seq;
		base_real = ktime_add(tk->tkr_mono.base,
				      tk_core.timekeeper.offs_real);
		base_boot = ktime_add(tk->tkr_mono.base,
				      tk_core.timekeeper.offs_boot);
		base_raw = tk->tkr_raw.base;
		nsec_real = timekeeping_cycles_to_ns(&tk->tkr_mono, now);
		nsec_raw  = timekeeping_cycles_to_ns(&tk->tkr_raw, now);
	} while (read_seqcount_retry(&tk_core.seq, seq));

	systime_snapshot->cycles = now;
	systime_snapshot->real = ktime_add_ns(base_real, nsec_real);
	systime_snapshot->boot = ktime_add_ns(base_boot, nsec_real);
	systime_snapshot->raw = ktime_add_ns(base_raw, nsec_raw);
}
EXPORT_SYMBOL_GPL(ktime_get_snapshot);

/* Scale base by mult/div checking for overflow */
static int scale64_check_overflow(u64 mult, u64 div, u64 *base)
{
	u64 tmp, rem;

	tmp = div64_u64_rem(*base, div, &rem);

	if (((int)sizeof(u64)*8 - fls64(mult) < fls64(tmp)) ||
	    ((int)sizeof(u64)*8 - fls64(mult) < fls64(rem)))
		return -EOVERFLOW;
	tmp *= mult;

	rem = div64_u64(rem * mult, div);
	*base = tmp + rem;
	return 0;
}

/**
 * adjust_historical_crosststamp - adjust crosstimestamp previous to current interval
 * @history:			Snapshot representing start of history
 * @partial_history_cycles:	Cycle offset into history (fractional part)
 * @total_history_cycles:	Total history length in cycles
 * @discontinuity:		True indicates clock was set on history period
 * @ts:				Cross timestamp that should be adjusted using
 *	partial/total ratio
 *
 * Helper function used by get_device_system_crosststamp() to correct the
 * crosstimestamp corresponding to the start of the current interval to the
 * system counter value (timestamp point) provided by the driver. The
 * total_history_* quantities are the total history starting at the provided
 * reference point and ending at the start of the current interval. The cycle
 * count between the driver timestamp point and the start of the current
 * interval is partial_history_cycles.
 */
static int adjust_historical_crosststamp(struct system_time_snapshot *history,
					 u64 partial_history_cycles,
					 u64 total_history_cycles,
					 bool discontinuity,
					 struct system_device_crosststamp *ts)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	u64 corr_raw, corr_real;
	bool interp_forward;
	int ret;

	if (total_history_cycles == 0 || partial_history_cycles == 0)
		return 0;

	/* Interpolate shortest distance from beginning or end of history */
	interp_forward = partial_history_cycles > total_history_cycles / 2;
	partial_history_cycles = interp_forward ?
		total_history_cycles - partial_history_cycles :
		partial_history_cycles;

	/*
	 * Scale the monotonic raw time delta by:
	 *	partial_history_cycles / total_history_cycles
	 */
	corr_raw = (u64)ktime_to_ns(
		ktime_sub(ts->sys_monoraw, history->raw));
	ret = scale64_check_overflow(partial_history_cycles,
				     total_history_cycles, &corr_raw);
	if (ret)
		return ret;

	/*
	 * If there is a discontinuity in the history, scale monotonic raw
	 *	correction by:
	 *	mult(real)/mult(raw) yielding the realtime correction
	 * Otherwise, calculate the realtime correction similar to monotonic
	 *	raw calculation
	 */
	if (discontinuity) {
		corr_real = mul_u64_u32_div
			(corr_raw, tk->tkr_mono.mult, tk->tkr_raw.mult);
	} else {
		corr_real = (u64)ktime_to_ns(
			ktime_sub(ts->sys_realtime, history->real));
		ret = scale64_check_overflow(partial_history_cycles,
					     total_history_cycles, &corr_real);
		if (ret)
			return ret;
	}

	/* Fixup monotonic raw and real time time values */
	if (interp_forward) {
		ts->sys_monoraw = ktime_add_ns(history->raw, corr_raw);
		ts->sys_realtime = ktime_add_ns(history->real, corr_real);
	} else {
		ts->sys_monoraw = ktime_sub_ns(ts->sys_monoraw, corr_raw);
		ts->sys_realtime = ktime_sub_ns(ts->sys_realtime, corr_real);
	}

	return 0;
}

/*
 * timestamp_in_interval - true if ts is chronologically in [start, end]
 *
 * True if ts occurs chronologically at or after start, and before or at end.
 */
static bool timestamp_in_interval(u64 start, u64 end, u64 ts)
{
	if (ts >= start && ts <= end)
		return true;
	if (start > end && (ts >= start || ts <= end))
		return true;
	return false;
}

static bool convert_clock(u64 *val, u32 numerator, u32 denominator)
{
	u64 rem, res;

	if (!numerator || !denominator)
		return false;

	res = div64_u64_rem(*val, denominator, &rem) * numerator;
	*val = res + div_u64(rem * numerator, denominator);
	return true;
}

static bool convert_base_to_cs(struct system_counterval_t *scv)
{
	struct clocksource *cs = tk_core.timekeeper.tkr_mono.clock;
	struct clocksource_base *base;
	u32 num, den;

	/* The timestamp was taken from the time keeper clock source */
	if (cs->id == scv->cs_id)
		return true;

	/*
	 * Check whether cs_id matches the base clock. Prevent the compiler from
	 * re-evaluating @base as the clocksource might change concurrently.
	 */
	base = READ_ONCE(cs->base);
	if (!base || base->id != scv->cs_id)
		return false;

	num = scv->use_nsecs ? cs->freq_khz : base->numerator;
	den = scv->use_nsecs ? USEC_PER_SEC : base->denominator;

	if (!convert_clock(&scv->cycles, num, den))
		return false;

	scv->cycles += base->offset;
	return true;
}

static bool convert_cs_to_base(u64 *cycles, enum clocksource_ids base_id)
{
	struct clocksource *cs = tk_core.timekeeper.tkr_mono.clock;
	struct clocksource_base *base;

	/*
	 * Check whether base_id matches the base clock. Prevent the compiler from
	 * re-evaluating @base as the clocksource might change concurrently.
	 */
	base = READ_ONCE(cs->base);
	if (!base || base->id != base_id)
		return false;

	*cycles -= base->offset;
	if (!convert_clock(cycles, base->denominator, base->numerator))
		return false;
	return true;
}

static bool convert_ns_to_cs(u64 *delta)
{
	struct tk_read_base *tkr = &tk_core.timekeeper.tkr_mono;

	if (BITS_TO_BYTES(fls64(*delta) + tkr->shift) >= sizeof(*delta))
		return false;

	*delta = div_u64((*delta << tkr->shift) - tkr->xtime_nsec, tkr->mult);
	return true;
}

/**
 * ktime_real_to_base_clock() - Convert CLOCK_REALTIME timestamp to a base clock timestamp
 * @treal:	CLOCK_REALTIME timestamp to convert
 * @base_id:	base clocksource id
 * @cycles:	pointer to store the converted base clock timestamp
 *
 * Converts a supplied, future realtime clock value to the corresponding base clock value.
 *
 * Return:  true if the conversion is successful, false otherwise.
 */
bool ktime_real_to_base_clock(ktime_t treal, enum clocksource_ids base_id, u64 *cycles)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	unsigned int seq;
	u64 delta;

	do {
		seq = read_seqcount_begin(&tk_core.seq);
		if ((u64)treal < tk->tkr_mono.base_real)
			return false;
		delta = (u64)treal - tk->tkr_mono.base_real;
		if (!convert_ns_to_cs(&delta))
			return false;
		*cycles = tk->tkr_mono.cycle_last + delta;
		if (!convert_cs_to_base(cycles, base_id))
			return false;
	} while (read_seqcount_retry(&tk_core.seq, seq));

	return true;
}
EXPORT_SYMBOL_GPL(ktime_real_to_base_clock);

/**
 * get_device_system_crosststamp - Synchronously capture system/device timestamp
 * @get_time_fn:	Callback to get simultaneous device time and
 *	system counter from the device driver
 * @ctx:		Context passed to get_time_fn()
 * @history_begin:	Historical reference point used to interpolate system
 *	time when counter provided by the driver is before the current interval
 * @xtstamp:		Receives simultaneously captured system and device time
 *
 * Reads a timestamp from a device and correlates it to system time
 */
int get_device_system_crosststamp(int (*get_time_fn)
				  (ktime_t *device_time,
				   struct system_counterval_t *sys_counterval,
				   void *ctx),
				  void *ctx,
				  struct system_time_snapshot *history_begin,
				  struct system_device_crosststamp *xtstamp)
{
	struct system_counterval_t system_counterval = {};
	struct timekeeper *tk = &tk_core.timekeeper;
	u64 cycles, now, interval_start;
	unsigned int clock_was_set_seq = 0;
	ktime_t base_real, base_raw;
	u64 nsec_real, nsec_raw;
	u8 cs_was_changed_seq;
	unsigned int seq;
	bool do_interp;
	int ret;

	do {
		seq = read_seqcount_begin(&tk_core.seq);
		/*
		 * Try to synchronously capture device time and a system
		 * counter value calling back into the device driver
		 */
		ret = get_time_fn(&xtstamp->device, &system_counterval, ctx);
		if (ret)
			return ret;

		/*
		 * Verify that the clocksource ID associated with the captured
		 * system counter value is the same as for the currently
		 * installed timekeeper clocksource
		 */
		if (system_counterval.cs_id == CSID_GENERIC ||
		    !convert_base_to_cs(&system_counterval))
			return -ENODEV;
		cycles = system_counterval.cycles;

		/*
		 * Check whether the system counter value provided by the
		 * device driver is on the current timekeeping interval.
		 */
		now = tk_clock_read(&tk->tkr_mono);
		interval_start = tk->tkr_mono.cycle_last;
		if (!timestamp_in_interval(interval_start, now, cycles)) {
			clock_was_set_seq = tk->clock_was_set_seq;
			cs_was_changed_seq = tk->cs_was_changed_seq;
			cycles = interval_start;
			do_interp = true;
		} else {
			do_interp = false;
		}

		base_real = ktime_add(tk->tkr_mono.base,
				      tk_core.timekeeper.offs_real);
		base_raw = tk->tkr_raw.base;

		nsec_real = timekeeping_cycles_to_ns(&tk->tkr_mono, cycles);
		nsec_raw = timekeeping_cycles_to_ns(&tk->tkr_raw, cycles);
	} while (read_seqcount_retry(&tk_core.seq, seq));

	xtstamp->sys_realtime = ktime_add_ns(base_real, nsec_real);
	xtstamp->sys_monoraw = ktime_add_ns(base_raw, nsec_raw);

	/*
	 * Interpolate if necessary, adjusting back from the start of the
	 * current interval
	 */
	if (do_interp) {
		u64 partial_history_cycles, total_history_cycles;
		bool discontinuity;

		/*
		 * Check that the counter value is not before the provided
		 * history reference and that the history doesn't cross a
		 * clocksource change
		 */
		if (!history_begin ||
		    !timestamp_in_interval(history_begin->cycles,
					   cycles, system_counterval.cycles) ||
		    history_begin->cs_was_changed_seq != cs_was_changed_seq)
			return -EINVAL;
		partial_history_cycles = cycles - system_counterval.cycles;
		total_history_cycles = cycles - history_begin->cycles;
		discontinuity =
			history_begin->clock_was_set_seq != clock_was_set_seq;

		ret = adjust_historical_crosststamp(history_begin,
						    partial_history_cycles,
						    total_history_cycles,
						    discontinuity, xtstamp);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(get_device_system_crosststamp);

/**
 * timekeeping_clocksource_has_base - Check whether the current clocksource
 *				      is based on given a base clock
 * @id:		base clocksource ID
 *
 * Note:	The return value is a snapshot which can become invalid right
 *		after the function returns.
 *
 * Return:	true if the timekeeper clocksource has a base clock with @id,
 *		false otherwise
 */
bool timekeeping_clocksource_has_base(enum clocksource_ids id)
{
	/*
	 * This is a snapshot, so no point in using the sequence
	 * count. Just prevent the compiler from re-evaluating @base as the
	 * clocksource might change concurrently.
	 */
	struct clocksource_base *base = READ_ONCE(tk_core.timekeeper.tkr_mono.clock->base);

	return base ? base->id == id : false;
}
EXPORT_SYMBOL_GPL(timekeeping_clocksource_has_base);

/**
 * do_settimeofday64 - Sets the time of day.
 * @ts:     pointer to the timespec64 variable containing the new time
 *
 * Sets the time of day to the new time and update NTP and notify hrtimers
 */
int do_settimeofday64(const struct timespec64 *ts)
{
	struct timespec64 ts_delta, xt;

	if (!timespec64_valid_settod(ts))
		return -EINVAL;

	scoped_guard (raw_spinlock_irqsave, &tk_core.lock) {
		struct timekeeper *tks = &tk_core.shadow_timekeeper;

		timekeeping_forward_now(tks);

		xt = tk_xtime(tks);
		ts_delta = timespec64_sub(*ts, xt);

		if (timespec64_compare(&tks->wall_to_monotonic, &ts_delta) > 0) {
			timekeeping_restore_shadow(&tk_core);
			return -EINVAL;
		}

		tk_set_wall_to_mono(tks, timespec64_sub(tks->wall_to_monotonic, ts_delta));
		tk_set_xtime(tks, ts);
		timekeeping_update_from_shadow(&tk_core, TK_UPDATE_ALL);
	}

	/* Signal hrtimers about time change */
	clock_was_set(CLOCK_SET_WALL);

	audit_tk_injoffset(ts_delta);
	add_device_randomness(ts, sizeof(*ts));
	return 0;
}
EXPORT_SYMBOL(do_settimeofday64);

static inline bool timekeeper_is_core_tk(struct timekeeper *tk)
{
	return !IS_ENABLED(CONFIG_POSIX_AUX_CLOCKS) || tk->id == TIMEKEEPER_CORE;
}

/**
 * __timekeeping_inject_offset - Adds or subtracts from the current time.
 * @tkd:	Pointer to the timekeeper to modify
 * @ts:		Pointer to the timespec variable containing the offset
 *
 * Adds or subtracts an offset value from the current time.
 */
static int __timekeeping_inject_offset(struct tk_data *tkd, const struct timespec64 *ts)
{
	struct timekeeper *tks = &tkd->shadow_timekeeper;
	struct timespec64 tmp;

	if (ts->tv_nsec < 0 || ts->tv_nsec >= NSEC_PER_SEC)
		return -EINVAL;

	timekeeping_forward_now(tks);

	if (timekeeper_is_core_tk(tks)) {
		/* Make sure the proposed value is valid */
		tmp = timespec64_add(tk_xtime(tks), *ts);
		if (timespec64_compare(&tks->wall_to_monotonic, ts) > 0 ||
		    !timespec64_valid_settod(&tmp)) {
			timekeeping_restore_shadow(tkd);
			return -EINVAL;
		}

		tk_xtime_add(tks, ts);
		tk_set_wall_to_mono(tks, timespec64_sub(tks->wall_to_monotonic, *ts));
	} else {
		struct tk_read_base *tkr_mono = &tks->tkr_mono;
		ktime_t now, offs;

		/* Get the current time */
		now = ktime_add_ns(tkr_mono->base, timekeeping_get_ns(tkr_mono));
		/* Add the relative offset change */
		offs = ktime_add(tks->offs_aux, timespec64_to_ktime(*ts));

		/* Prevent that the resulting time becomes negative */
		if (ktime_add(now, offs) < 0) {
			timekeeping_restore_shadow(tkd);
			return -EINVAL;
		}
		tks->offs_aux = offs;
	}

	timekeeping_update_from_shadow(tkd, TK_UPDATE_ALL);
	return 0;
}

static int timekeeping_inject_offset(const struct timespec64 *ts)
{
	int ret;

	scoped_guard (raw_spinlock_irqsave, &tk_core.lock)
		ret = __timekeeping_inject_offset(&tk_core, ts);

	/* Signal hrtimers about time change */
	if (!ret)
		clock_was_set(CLOCK_SET_WALL);
	return ret;
}

/*
 * Indicates if there is an offset between the system clock and the hardware
 * clock/persistent clock/rtc.
 */
int persistent_clock_is_local;

/*
 * Adjust the time obtained from the CMOS to be UTC time instead of
 * local time.
 *
 * This is ugly, but preferable to the alternatives.  Otherwise we
 * would either need to write a program to do it in /etc/rc (and risk
 * confusion if the program gets run more than once; it would also be
 * hard to make the program warp the clock precisely n hours)  or
 * compile in the timezone information into the kernel.  Bad, bad....
 *
 *						- TYT, 1992-01-01
 *
 * The best thing to do is to keep the CMOS clock in universal time (UTC)
 * as real UNIX machines always do it. This avoids all headaches about
 * daylight saving times and warping kernel clocks.
 */
void timekeeping_warp_clock(void)
{
	if (sys_tz.tz_minuteswest != 0) {
		struct timespec64 adjust;

		persistent_clock_is_local = 1;
		adjust.tv_sec = sys_tz.tz_minuteswest * 60;
		adjust.tv_nsec = 0;
		timekeeping_inject_offset(&adjust);
	}
}

/*
 * __timekeeping_set_tai_offset - Sets the TAI offset from UTC and monotonic
 */
static void __timekeeping_set_tai_offset(struct timekeeper *tk, s32 tai_offset)
{
	tk->tai_offset = tai_offset;
	tk->offs_tai = ktime_add(tk->offs_real, ktime_set(tai_offset, 0));
}

/*
 * change_clocksource - Swaps clocksources if a new one is available
 *
 * Accumulates current time interval and initializes new clocksource
 */
static int change_clocksource(void *data)
{
	struct clocksource *new = data, *old = NULL;

	/*
	 * If the clocksource is in a module, get a module reference.
	 * Succeeds for built-in code (owner == NULL) as well. Abort if the
	 * reference can't be acquired.
	 */
	if (!try_module_get(new->owner))
		return 0;

	/* Abort if the device can't be enabled */
	if (new->enable && new->enable(new) != 0) {
		module_put(new->owner);
		return 0;
	}

	scoped_guard (raw_spinlock_irqsave, &tk_core.lock) {
		struct timekeeper *tks = &tk_core.shadow_timekeeper;

		timekeeping_forward_now(tks);
		old = tks->tkr_mono.clock;
		tk_setup_internals(tks, new);
		timekeeping_update_from_shadow(&tk_core, TK_UPDATE_ALL);
	}

	tk_aux_update_clocksource();

	if (old) {
		if (old->disable)
			old->disable(old);
		module_put(old->owner);
	}

	return 0;
}

/**
 * timekeeping_notify - Install a new clock source
 * @clock:		pointer to the clock source
 *
 * This function is called from clocksource.c after a new, better clock
 * source has been registered. The caller holds the clocksource_mutex.
 */
int timekeeping_notify(struct clocksource *clock)
{
	struct timekeeper *tk = &tk_core.timekeeper;

	if (tk->tkr_mono.clock == clock)
		return 0;
	stop_machine(change_clocksource, clock, NULL);
	tick_clock_notify();
	return tk->tkr_mono.clock == clock ? 0 : -1;
}

/**
 * ktime_get_raw_ts64 - Returns the raw monotonic time in a timespec
 * @ts:		pointer to the timespec64 to be set
 *
 * Returns the raw monotonic time (completely un-modified by ntp)
 */
void ktime_get_raw_ts64(struct timespec64 *ts)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	unsigned int seq;
	u64 nsecs;

	do {
		seq = read_seqcount_begin(&tk_core.seq);
		ts->tv_sec = tk->raw_sec;
		nsecs = timekeeping_get_ns(&tk->tkr_raw);

	} while (read_seqcount_retry(&tk_core.seq, seq));

	ts->tv_nsec = 0;
	timespec64_add_ns(ts, nsecs);
}
EXPORT_SYMBOL(ktime_get_raw_ts64);

/**
 * ktime_get_clock_ts64 - Returns time of a clock in a timespec
 * @id:		POSIX clock ID of the clock to read
 * @ts:		Pointer to the timespec64 to be set
 *
 * The timestamp is invalidated (@ts->sec is set to -1) if the
 * clock @id is not available.
 */
void ktime_get_clock_ts64(clockid_t id, struct timespec64 *ts)
{
	/* Invalidate time stamp */
	ts->tv_sec = -1;
	ts->tv_nsec = 0;

	switch (id) {
	case CLOCK_REALTIME:
		ktime_get_real_ts64(ts);
		return;
	case CLOCK_MONOTONIC:
		ktime_get_ts64(ts);
		return;
	case CLOCK_MONOTONIC_RAW:
		ktime_get_raw_ts64(ts);
		return;
	case CLOCK_AUX ... CLOCK_AUX_LAST:
		if (IS_ENABLED(CONFIG_POSIX_AUX_CLOCKS))
			ktime_get_aux_ts64(id, ts);
		return;
	default:
		WARN_ON_ONCE(1);
	}
}
EXPORT_SYMBOL_GPL(ktime_get_clock_ts64);

/**
 * timekeeping_valid_for_hres - Check if timekeeping is suitable for hres
 */
int timekeeping_valid_for_hres(void)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	unsigned int seq;
	int ret;

	do {
		seq = read_seqcount_begin(&tk_core.seq);

		ret = tk->tkr_mono.clock->flags & CLOCK_SOURCE_VALID_FOR_HRES;

	} while (read_seqcount_retry(&tk_core.seq, seq));

	return ret;
}

/**
 * timekeeping_max_deferment - Returns max time the clocksource can be deferred
 */
u64 timekeeping_max_deferment(void)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	unsigned int seq;
	u64 ret;

	do {
		seq = read_seqcount_begin(&tk_core.seq);

		ret = tk->tkr_mono.clock->max_idle_ns;

	} while (read_seqcount_retry(&tk_core.seq, seq));

	return ret;
}

/**
 * read_persistent_clock64 -  Return time from the persistent clock.
 * @ts: Pointer to the storage for the readout value
 *
 * Weak dummy function for arches that do not yet support it.
 * Reads the time from the battery backed persistent clock.
 * Returns a timespec with tv_sec=0 and tv_nsec=0 if unsupported.
 *
 *  XXX - Do be sure to remove it once all arches implement it.
 */
void __weak read_persistent_clock64(struct timespec64 *ts)
{
	ts->tv_sec = 0;
	ts->tv_nsec = 0;
}

/**
 * read_persistent_wall_and_boot_offset - Read persistent clock, and also offset
 *                                        from the boot.
 * @wall_time:	  current time as returned by persistent clock
 * @boot_offset:  offset that is defined as wall_time - boot_time
 *
 * Weak dummy function for arches that do not yet support it.
 *
 * The default function calculates offset based on the current value of
 * local_clock(). This way architectures that support sched_clock() but don't
 * support dedicated boot time clock will provide the best estimate of the
 * boot time.
 */
void __weak __init
read_persistent_wall_and_boot_offset(struct timespec64 *wall_time,
				     struct timespec64 *boot_offset)
{
	read_persistent_clock64(wall_time);
	*boot_offset = ns_to_timespec64(local_clock());
}

static __init void tkd_basic_setup(struct tk_data *tkd, enum timekeeper_ids tk_id, bool valid)
{
	raw_spin_lock_init(&tkd->lock);
	seqcount_raw_spinlock_init(&tkd->seq, &tkd->lock);
	tkd->timekeeper.id = tkd->shadow_timekeeper.id = tk_id;
	tkd->timekeeper.clock_valid = tkd->shadow_timekeeper.clock_valid = valid;
}

/*
 * Flag reflecting whether timekeeping_resume() has injected sleeptime.
 *
 * The flag starts of false and is only set when a suspend reaches
 * timekeeping_suspend(), timekeeping_resume() sets it to false when the
 * timekeeper clocksource is not stopping across suspend and has been
 * used to update sleep time. If the timekeeper clocksource has stopped
 * then the flag stays true and is used by the RTC resume code to decide
 * whether sleeptime must be injected and if so the flag gets false then.
 *
 * If a suspend fails before reaching timekeeping_resume() then the flag
 * stays false and prevents erroneous sleeptime injection.
 */
static bool suspend_timing_needed;

/* Flag for if there is a persistent clock on this platform */
static bool persistent_clock_exists;

/*
 * timekeeping_init - Initializes the clocksource and common timekeeping values
 */
void __init timekeeping_init(void)
{
	struct timespec64 wall_time, boot_offset, wall_to_mono;
	struct timekeeper *tks = &tk_core.shadow_timekeeper;
	struct clocksource *clock;

	tkd_basic_setup(&tk_core, TIMEKEEPER_CORE, true);
	tk_aux_setup();

	read_persistent_wall_and_boot_offset(&wall_time, &boot_offset);
	if (timespec64_valid_settod(&wall_time) &&
	    timespec64_to_ns(&wall_time) > 0) {
		persistent_clock_exists = true;
	} else if (timespec64_to_ns(&wall_time) != 0) {
		pr_warn("Persistent clock returned invalid value");
		wall_time = (struct timespec64){0};
	}

	if (timespec64_compare(&wall_time, &boot_offset) < 0)
		boot_offset = (struct timespec64){0};

	/*
	 * We want set wall_to_mono, so the following is true:
	 * wall time + wall_to_mono = boot time
	 */
	wall_to_mono = timespec64_sub(boot_offset, wall_time);

	guard(raw_spinlock_irqsave)(&tk_core.lock);

	ntp_init();

	clock = clocksource_default_clock();
	if (clock->enable)
		clock->enable(clock);
	tk_setup_internals(tks, clock);

	tk_set_xtime(tks, &wall_time);
	tks->raw_sec = 0;

	tk_set_wall_to_mono(tks, wall_to_mono);

	timekeeping_update_from_shadow(&tk_core, TK_CLOCK_WAS_SET);
}

/* time in seconds when suspend began for persistent clock */
static struct timespec64 timekeeping_suspend_time;

/**
 * __timekeeping_inject_sleeptime - Internal function to add sleep interval
 * @tk:		Pointer to the timekeeper to be updated
 * @delta:	Pointer to the delta value in timespec64 format
 *
 * Takes a timespec offset measuring a suspend interval and properly
 * adds the sleep offset to the timekeeping variables.
 */
static void __timekeeping_inject_sleeptime(struct timekeeper *tk,
					   const struct timespec64 *delta)
{
	if (!timespec64_valid_strict(delta)) {
		printk_deferred(KERN_WARNING
				"__timekeeping_inject_sleeptime: Invalid "
				"sleep delta value!\n");
		return;
	}
	tk_xtime_add(tk, delta);
	tk_set_wall_to_mono(tk, timespec64_sub(tk->wall_to_monotonic, *delta));
	tk_update_sleep_time(tk, timespec64_to_ktime(*delta));
	tk_debug_account_sleep_time(delta);
}

#if defined(CONFIG_PM_SLEEP) && defined(CONFIG_RTC_HCTOSYS_DEVICE)
/*
 * We have three kinds of time sources to use for sleep time
 * injection, the preference order is:
 * 1) non-stop clocksource
 * 2) persistent clock (ie: RTC accessible when irqs are off)
 * 3) RTC
 *
 * 1) and 2) are used by timekeeping, 3) by RTC subsystem.
 * If system has neither 1) nor 2), 3) will be used finally.
 *
 *
 * If timekeeping has injected sleeptime via either 1) or 2),
 * 3) becomes needless, so in this case we don't need to call
 * rtc_resume(), and this is what timekeeping_rtc_skipresume()
 * means.
 */
bool timekeeping_rtc_skipresume(void)
{
	return !suspend_timing_needed;
}

/*
 * 1) can be determined whether to use or not only when doing
 * timekeeping_resume() which is invoked after rtc_suspend(),
 * so we can't skip rtc_suspend() surely if system has 1).
 *
 * But if system has 2), 2) will definitely be used, so in this
 * case we don't need to call rtc_suspend(), and this is what
 * timekeeping_rtc_skipsuspend() means.
 */
bool timekeeping_rtc_skipsuspend(void)
{
	return persistent_clock_exists;
}

/**
 * timekeeping_inject_sleeptime64 - Adds suspend interval to timeekeeping values
 * @delta: pointer to a timespec64 delta value
 *
 * This hook is for architectures that cannot support read_persistent_clock64
 * because their RTC/persistent clock is only accessible when irqs are enabled.
 * and also don't have an effective nonstop clocksource.
 *
 * This function should only be called by rtc_resume(), and allows
 * a suspend offset to be injected into the timekeeping values.
 */
void timekeeping_inject_sleeptime64(const struct timespec64 *delta)
{
	scoped_guard(raw_spinlock_irqsave, &tk_core.lock) {
		struct timekeeper *tks = &tk_core.shadow_timekeeper;

		suspend_timing_needed = false;
		timekeeping_forward_now(tks);
		__timekeeping_inject_sleeptime(tks, delta);
		timekeeping_update_from_shadow(&tk_core, TK_UPDATE_ALL);
	}

	/* Signal hrtimers about time change */
	clock_was_set(CLOCK_SET_WALL | CLOCK_SET_BOOT);
}
#endif

/**
 * timekeeping_resume - Resumes the generic timekeeping subsystem.
 */
void timekeeping_resume(void)
{
	struct timekeeper *tks = &tk_core.shadow_timekeeper;
	struct clocksource *clock = tks->tkr_mono.clock;
	struct timespec64 ts_new, ts_delta;
	bool inject_sleeptime = false;
	u64 cycle_now, nsec;
	unsigned long flags;

	read_persistent_clock64(&ts_new);

	clockevents_resume();
	clocksource_resume();

	raw_spin_lock_irqsave(&tk_core.lock, flags);

	/*
	 * After system resumes, we need to calculate the suspended time and
	 * compensate it for the OS time. There are 3 sources that could be
	 * used: Nonstop clocksource during suspend, persistent clock and rtc
	 * device.
	 *
	 * One specific platform may have 1 or 2 or all of them, and the
	 * preference will be:
	 *	suspend-nonstop clocksource -> persistent clock -> rtc
	 * The less preferred source will only be tried if there is no better
	 * usable source. The rtc part is handled separately in rtc core code.
	 */
	cycle_now = tk_clock_read(&tks->tkr_mono);
	nsec = clocksource_stop_suspend_timing(clock, cycle_now);
	if (nsec > 0) {
		ts_delta = ns_to_timespec64(nsec);
		inject_sleeptime = true;
	} else if (timespec64_compare(&ts_new, &timekeeping_suspend_time) > 0) {
		ts_delta = timespec64_sub(ts_new, timekeeping_suspend_time);
		inject_sleeptime = true;
	}

	if (inject_sleeptime) {
		suspend_timing_needed = false;
		__timekeeping_inject_sleeptime(tks, &ts_delta);
	}

	/* Re-base the last cycle value */
	tks->tkr_mono.cycle_last = cycle_now;
	tks->tkr_raw.cycle_last  = cycle_now;

	tks->ntp_error = 0;
	timekeeping_suspended = 0;
	timekeeping_update_from_shadow(&tk_core, TK_CLOCK_WAS_SET);
	raw_spin_unlock_irqrestore(&tk_core.lock, flags);

	touch_softlockup_watchdog();

	/* Resume the clockevent device(s) and hrtimers */
	tick_resume();
	/* Notify timerfd as resume is equivalent to clock_was_set() */
	timerfd_resume();
}

int timekeeping_suspend(void)
{
	struct timekeeper *tks = &tk_core.shadow_timekeeper;
	struct timespec64 delta, delta_delta;
	static struct timespec64 old_delta;
	struct clocksource *curr_clock;
	unsigned long flags;
	u64 cycle_now;

	read_persistent_clock64(&timekeeping_suspend_time);

	/*
	 * On some systems the persistent_clock can not be detected at
	 * timekeeping_init by its return value, so if we see a valid
	 * value returned, update the persistent_clock_exists flag.
	 */
	if (timekeeping_suspend_time.tv_sec || timekeeping_suspend_time.tv_nsec)
		persistent_clock_exists = true;

	suspend_timing_needed = true;

	raw_spin_lock_irqsave(&tk_core.lock, flags);
	timekeeping_forward_now(tks);
	timekeeping_suspended = 1;

	/*
	 * Since we've called forward_now, cycle_last stores the value
	 * just read from the current clocksource. Save this to potentially
	 * use in suspend timing.
	 */
	curr_clock = tks->tkr_mono.clock;
	cycle_now = tks->tkr_mono.cycle_last;
	clocksource_start_suspend_timing(curr_clock, cycle_now);

	if (persistent_clock_exists) {
		/*
		 * To avoid drift caused by repeated suspend/resumes,
		 * which each can add ~1 second drift error,
		 * try to compensate so the difference in system time
		 * and persistent_clock time stays close to constant.
		 */
		delta = timespec64_sub(tk_xtime(tks), timekeeping_suspend_time);
		delta_delta = timespec64_sub(delta, old_delta);
		if (abs(delta_delta.tv_sec) >= 2) {
			/*
			 * if delta_delta is too large, assume time correction
			 * has occurred and set old_delta to the current delta.
			 */
			old_delta = delta;
		} else {
			/* Otherwise try to adjust old_system to compensate */
			timekeeping_suspend_time =
				timespec64_add(timekeeping_suspend_time, delta_delta);
		}
	}

	timekeeping_update_from_shadow(&tk_core, 0);
	halt_fast_timekeeper(tks);
	raw_spin_unlock_irqrestore(&tk_core.lock, flags);

	tick_suspend();
	clocksource_suspend();
	clockevents_suspend();

	return 0;
}

/* sysfs resume/suspend bits for timekeeping */
static struct syscore_ops timekeeping_syscore_ops = {
	.resume		= timekeeping_resume,
	.suspend	= timekeeping_suspend,
};

static int __init timekeeping_init_ops(void)
{
	register_syscore_ops(&timekeeping_syscore_ops);
	return 0;
}
device_initcall(timekeeping_init_ops);

/*
 * Apply a multiplier adjustment to the timekeeper
 */
static __always_inline void timekeeping_apply_adjustment(struct timekeeper *tk,
							 s64 offset,
							 s32 mult_adj)
{
	s64 interval = tk->cycle_interval;

	if (mult_adj == 0) {
		return;
	} else if (mult_adj == -1) {
		interval = -interval;
		offset = -offset;
	} else if (mult_adj != 1) {
		interval *= mult_adj;
		offset *= mult_adj;
	}

	/*
	 * So the following can be confusing.
	 *
	 * To keep things simple, lets assume mult_adj == 1 for now.
	 *
	 * When mult_adj != 1, remember that the interval and offset values
	 * have been appropriately scaled so the math is the same.
	 *
	 * The basic idea here is that we're increasing the multiplier
	 * by one, this causes the xtime_interval to be incremented by
	 * one cycle_interval. This is because:
	 *	xtime_interval = cycle_interval * mult
	 * So if mult is being incremented by one:
	 *	xtime_interval = cycle_interval * (mult + 1)
	 * Its the same as:
	 *	xtime_interval = (cycle_interval * mult) + cycle_interval
	 * Which can be shortened to:
	 *	xtime_interval += cycle_interval
	 *
	 * So offset stores the non-accumulated cycles. Thus the current
	 * time (in shifted nanoseconds) is:
	 *	now = (offset * adj) + xtime_nsec
	 * Now, even though we're adjusting the clock frequency, we have
	 * to keep time consistent. In other words, we can't jump back
	 * in time, and we also want to avoid jumping forward in time.
	 *
	 * So given the same offset value, we need the time to be the same
	 * both before and after the freq adjustment.
	 *	now = (offset * adj_1) + xtime_nsec_1
	 *	now = (offset * adj_2) + xtime_nsec_2
	 * So:
	 *	(offset * adj_1) + xtime_nsec_1 =
	 *		(offset * adj_2) + xtime_nsec_2
	 * And we know:
	 *	adj_2 = adj_1 + 1
	 * So:
	 *	(offset * adj_1) + xtime_nsec_1 =
	 *		(offset * (adj_1+1)) + xtime_nsec_2
	 *	(offset * adj_1) + xtime_nsec_1 =
	 *		(offset * adj_1) + offset + xtime_nsec_2
	 * Canceling the sides:
	 *	xtime_nsec_1 = offset + xtime_nsec_2
	 * Which gives us:
	 *	xtime_nsec_2 = xtime_nsec_1 - offset
	 * Which simplifies to:
	 *	xtime_nsec -= offset
	 */
	if ((mult_adj > 0) && (tk->tkr_mono.mult + mult_adj < mult_adj)) {
		/* NTP adjustment caused clocksource mult overflow */
		WARN_ON_ONCE(1);
		return;
	}

	tk->tkr_mono.mult += mult_adj;
	tk->xtime_interval += interval;
	tk->tkr_mono.xtime_nsec -= offset;
}

/*
 * Adjust the timekeeper's multiplier to the correct frequency
 * and also to reduce the accumulated error value.
 */
static void timekeeping_adjust(struct timekeeper *tk, s64 offset)
{
	u64 ntp_tl = ntp_tick_length(tk->id);
	u32 mult;

	/*
	 * Determine the multiplier from the current NTP tick length.
	 * Avoid expensive division when the tick length doesn't change.
	 */
	if (likely(tk->ntp_tick == ntp_tl)) {
		mult = tk->tkr_mono.mult - tk->ntp_err_mult;
	} else {
		tk->ntp_tick = ntp_tl;
		mult = div64_u64((tk->ntp_tick >> tk->ntp_error_shift) -
				 tk->xtime_remainder, tk->cycle_interval);
	}

	/*
	 * If the clock is behind the NTP time, increase the multiplier by 1
	 * to catch up with it. If it's ahead and there was a remainder in the
	 * tick division, the clock will slow down. Otherwise it will stay
	 * ahead until the tick length changes to a non-divisible value.
	 */
	tk->ntp_err_mult = tk->ntp_error > 0 ? 1 : 0;
	mult += tk->ntp_err_mult;

	timekeeping_apply_adjustment(tk, offset, mult - tk->tkr_mono.mult);

	if (unlikely(tk->tkr_mono.clock->maxadj &&
		(abs(tk->tkr_mono.mult - tk->tkr_mono.clock->mult)
			> tk->tkr_mono.clock->maxadj))) {
		printk_once(KERN_WARNING
			"Adjusting %s more than 11%% (%ld vs %ld)\n",
			tk->tkr_mono.clock->name, (long)tk->tkr_mono.mult,
			(long)tk->tkr_mono.clock->mult + tk->tkr_mono.clock->maxadj);
	}

	/*
	 * It may be possible that when we entered this function, xtime_nsec
	 * was very small.  Further, if we're slightly speeding the clocksource
	 * in the code above, its possible the required corrective factor to
	 * xtime_nsec could cause it to underflow.
	 *
	 * Now, since we have already accumulated the second and the NTP
	 * subsystem has been notified via second_overflow(), we need to skip
	 * the next update.
	 */
	if (unlikely((s64)tk->tkr_mono.xtime_nsec < 0)) {
		tk->tkr_mono.xtime_nsec += (u64)NSEC_PER_SEC <<
							tk->tkr_mono.shift;
		tk->xtime_sec--;
		tk->skip_second_overflow = 1;
	}
}

/*
 * accumulate_nsecs_to_secs - Accumulates nsecs into secs
 *
 * Helper function that accumulates the nsecs greater than a second
 * from the xtime_nsec field to the xtime_secs field.
 * It also calls into the NTP code to handle leapsecond processing.
 */
static inline unsigned int accumulate_nsecs_to_secs(struct timekeeper *tk)
{
	u64 nsecps = (u64)NSEC_PER_SEC << tk->tkr_mono.shift;
	unsigned int clock_set = 0;

	while (tk->tkr_mono.xtime_nsec >= nsecps) {
		int leap;

		tk->tkr_mono.xtime_nsec -= nsecps;
		tk->xtime_sec++;

		/*
		 * Skip NTP update if this second was accumulated before,
		 * i.e. xtime_nsec underflowed in timekeeping_adjust()
		 */
		if (unlikely(tk->skip_second_overflow)) {
			tk->skip_second_overflow = 0;
			continue;
		}

		/* Figure out if its a leap sec and apply if needed */
		leap = second_overflow(tk->id, tk->xtime_sec);
		if (unlikely(leap)) {
			struct timespec64 ts;

			tk->xtime_sec += leap;

			ts.tv_sec = leap;
			ts.tv_nsec = 0;
			tk_set_wall_to_mono(tk,
				timespec64_sub(tk->wall_to_monotonic, ts));

			__timekeeping_set_tai_offset(tk, tk->tai_offset - leap);

			clock_set = TK_CLOCK_WAS_SET;
		}
	}
	return clock_set;
}

/*
 * logarithmic_accumulation - shifted accumulation of cycles
 *
 * This functions accumulates a shifted interval of cycles into
 * a shifted interval nanoseconds. Allows for O(log) accumulation
 * loop.
 *
 * Returns the unconsumed cycles.
 */
static u64 logarithmic_accumulation(struct timekeeper *tk, u64 offset,
				    u32 shift, unsigned int *clock_set)
{
	u64 interval = tk->cycle_interval << shift;
	u64 snsec_per_sec;

	/* If the offset is smaller than a shifted interval, do nothing */
	if (offset < interval)
		return offset;

	/* Accumulate one shifted interval */
	offset -= interval;
	tk->tkr_mono.cycle_last += interval;
	tk->tkr_raw.cycle_last  += interval;

	tk->tkr_mono.xtime_nsec += tk->xtime_interval << shift;
	*clock_set |= accumulate_nsecs_to_secs(tk);

	/* Accumulate raw time */
	tk->tkr_raw.xtime_nsec += tk->raw_interval << shift;
	snsec_per_sec = (u64)NSEC_PER_SEC << tk->tkr_raw.shift;
	while (tk->tkr_raw.xtime_nsec >= snsec_per_sec) {
		tk->tkr_raw.xtime_nsec -= snsec_per_sec;
		tk->raw_sec++;
	}

	/* Accumulate error between NTP and clock interval */
	tk->ntp_error += tk->ntp_tick << shift;
	tk->ntp_error -= (tk->xtime_interval + tk->xtime_remainder) <<
						(tk->ntp_error_shift + shift);

	return offset;
}

/*
 * timekeeping_advance - Updates the timekeeper to the current time and
 * current NTP tick length
 */
static bool __timekeeping_advance(struct tk_data *tkd, enum timekeeping_adv_mode mode)
{
	struct timekeeper *tk = &tkd->shadow_timekeeper;
	struct timekeeper *real_tk = &tkd->timekeeper;
	unsigned int clock_set = 0;
	int shift = 0, maxshift;
	u64 offset, orig_offset;

	/* Make sure we're fully resumed: */
	if (unlikely(timekeeping_suspended))
		return false;

	offset = clocksource_delta(tk_clock_read(&tk->tkr_mono),
				   tk->tkr_mono.cycle_last, tk->tkr_mono.mask,
				   tk->tkr_mono.clock->max_raw_delta);
	orig_offset = offset;
	/* Check if there's really nothing to do */
	if (offset < real_tk->cycle_interval && mode == TK_ADV_TICK)
		return false;

	/*
	 * With NO_HZ we may have to accumulate many cycle_intervals
	 * (think "ticks") worth of time at once. To do this efficiently,
	 * we calculate the largest doubling multiple of cycle_intervals
	 * that is smaller than the offset.  We then accumulate that
	 * chunk in one go, and then try to consume the next smaller
	 * doubled multiple.
	 */
	shift = ilog2(offset) - ilog2(tk->cycle_interval);
	shift = max(0, shift);
	/* Bound shift to one less than what overflows tick_length */
	maxshift = (64 - (ilog2(ntp_tick_length(tk->id)) + 1)) - 1;
	shift = min(shift, maxshift);
	while (offset >= tk->cycle_interval) {
		offset = logarithmic_accumulation(tk, offset, shift, &clock_set);
		if (offset < tk->cycle_interval<<shift)
			shift--;
	}

	/* Adjust the multiplier to correct NTP error */
	timekeeping_adjust(tk, offset);

	/*
	 * Finally, make sure that after the rounding
	 * xtime_nsec isn't larger than NSEC_PER_SEC
	 */
	clock_set |= accumulate_nsecs_to_secs(tk);

	/*
	 * To avoid inconsistencies caused adjtimex TK_ADV_FREQ calls
	 * making small negative adjustments to the base xtime_nsec
	 * value, only update the coarse clocks if we accumulated time
	 */
	if (orig_offset != offset)
		tk_update_coarse_nsecs(tk);

	timekeeping_update_from_shadow(tkd, clock_set);

	return !!clock_set;
}

static bool timekeeping_advance(enum timekeeping_adv_mode mode)
{
	guard(raw_spinlock_irqsave)(&tk_core.lock);
	return __timekeeping_advance(&tk_core, mode);
}

/**
 * update_wall_time - Uses the current clocksource to increment the wall time
 *
 * It also updates the enabled auxiliary clock timekeepers
 */
void update_wall_time(void)
{
	if (timekeeping_advance(TK_ADV_TICK))
		clock_was_set_delayed();
	tk_aux_advance();
}

/**
 * getboottime64 - Return the real time of system boot.
 * @ts:		pointer to the timespec64 to be set
 *
 * Returns the wall-time of boot in a timespec64.
 *
 * This is based on the wall_to_monotonic offset and the total suspend
 * time. Calls to settimeofday will affect the value returned (which
 * basically means that however wrong your real time clock is at boot time,
 * you get the right time here).
 */
void getboottime64(struct timespec64 *ts)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	ktime_t t = ktime_sub(tk->offs_real, tk->offs_boot);

	*ts = ktime_to_timespec64(t);
}
EXPORT_SYMBOL_GPL(getboottime64);

void ktime_get_coarse_real_ts64(struct timespec64 *ts)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	unsigned int seq;

	do {
		seq = read_seqcount_begin(&tk_core.seq);

		*ts = tk_xtime_coarse(tk);
	} while (read_seqcount_retry(&tk_core.seq, seq));
}
EXPORT_SYMBOL(ktime_get_coarse_real_ts64);

/**
 * ktime_get_coarse_real_ts64_mg - return latter of coarse grained time or floor
 * @ts:		timespec64 to be filled
 *
 * Fetch the global mg_floor value, convert it to realtime and compare it
 * to the current coarse-grained time. Fill @ts with whichever is
 * latest. Note that this is a filesystem-specific interface and should be
 * avoided outside of that context.
 */
void ktime_get_coarse_real_ts64_mg(struct timespec64 *ts)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	u64 floor = atomic64_read(&mg_floor);
	ktime_t f_real, offset, coarse;
	unsigned int seq;

	do {
		seq = read_seqcount_begin(&tk_core.seq);
		*ts = tk_xtime_coarse(tk);
		offset = tk_core.timekeeper.offs_real;
	} while (read_seqcount_retry(&tk_core.seq, seq));

	coarse = timespec64_to_ktime(*ts);
	f_real = ktime_add(floor, offset);
	if (ktime_after(f_real, coarse))
		*ts = ktime_to_timespec64(f_real);
}

/**
 * ktime_get_real_ts64_mg - attempt to update floor value and return result
 * @ts:		pointer to the timespec to be set
 *
 * Get a monotonic fine-grained time value and attempt to swap it into
 * mg_floor. If that succeeds then accept the new floor value. If it fails
 * then another task raced in during the interim time and updated the
 * floor.  Since any update to the floor must be later than the previous
 * floor, either outcome is acceptable.
 *
 * Typically this will be called after calling ktime_get_coarse_real_ts64_mg(),
 * and determining that the resulting coarse-grained timestamp did not effect
 * a change in ctime. Any more recent floor value would effect a change to
 * ctime, so there is no need to retry the atomic64_try_cmpxchg() on failure.
 *
 * @ts will be filled with the latest floor value, regardless of the outcome of
 * the cmpxchg. Note that this is a filesystem specific interface and should be
 * avoided outside of that context.
 */
void ktime_get_real_ts64_mg(struct timespec64 *ts)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	ktime_t old = atomic64_read(&mg_floor);
	ktime_t offset, mono;
	unsigned int seq;
	u64 nsecs;

	do {
		seq = read_seqcount_begin(&tk_core.seq);

		ts->tv_sec = tk->xtime_sec;
		mono = tk->tkr_mono.base;
		nsecs = timekeeping_get_ns(&tk->tkr_mono);
		offset = tk_core.timekeeper.offs_real;
	} while (read_seqcount_retry(&tk_core.seq, seq));

	mono = ktime_add_ns(mono, nsecs);

	/*
	 * Attempt to update the floor with the new time value. As any
	 * update must be later then the existing floor, and would effect
	 * a change to ctime from the perspective of the current task,
	 * accept the resulting floor value regardless of the outcome of
	 * the swap.
	 */
	if (atomic64_try_cmpxchg(&mg_floor, &old, mono)) {
		ts->tv_nsec = 0;
		timespec64_add_ns(ts, nsecs);
		timekeeping_inc_mg_floor_swaps();
	} else {
		/*
		 * Another task changed mg_floor since "old" was fetched.
		 * "old" has been updated with the latest value of "mg_floor".
		 * That value is newer than the previous floor value, which
		 * is enough to effect a change to ctime. Accept it.
		 */
		*ts = ktime_to_timespec64(ktime_add(old, offset));
	}
}

void ktime_get_coarse_ts64(struct timespec64 *ts)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	struct timespec64 now, mono;
	unsigned int seq;

	do {
		seq = read_seqcount_begin(&tk_core.seq);

		now = tk_xtime_coarse(tk);
		mono = tk->wall_to_monotonic;
	} while (read_seqcount_retry(&tk_core.seq, seq));

	set_normalized_timespec64(ts, now.tv_sec + mono.tv_sec,
				  now.tv_nsec + mono.tv_nsec);
}
EXPORT_SYMBOL(ktime_get_coarse_ts64);

/*
 * Must hold jiffies_lock
 */
void do_timer(unsigned long ticks)
{
	jiffies_64 += ticks;
	calc_global_load();
}

/**
 * ktime_get_update_offsets_now - hrtimer helper
 * @cwsseq:	pointer to check and store the clock was set sequence number
 * @offs_real:	pointer to storage for monotonic -> realtime offset
 * @offs_boot:	pointer to storage for monotonic -> boottime offset
 * @offs_tai:	pointer to storage for monotonic -> clock tai offset
 *
 * Returns current monotonic time and updates the offsets if the
 * sequence number in @cwsseq and timekeeper.clock_was_set_seq are
 * different.
 *
 * Called from hrtimer_interrupt() or retrigger_next_event()
 */
ktime_t ktime_get_update_offsets_now(unsigned int *cwsseq, ktime_t *offs_real,
				     ktime_t *offs_boot, ktime_t *offs_tai)
{
	struct timekeeper *tk = &tk_core.timekeeper;
	unsigned int seq;
	ktime_t base;
	u64 nsecs;

	do {
		seq = read_seqcount_begin(&tk_core.seq);

		base = tk->tkr_mono.base;
		nsecs = timekeeping_get_ns(&tk->tkr_mono);
		base = ktime_add_ns(base, nsecs);

		if (*cwsseq != tk->clock_was_set_seq) {
			*cwsseq = tk->clock_was_set_seq;
			*offs_real = tk->offs_real;
			*offs_boot = tk->offs_boot;
			*offs_tai = tk->offs_tai;
		}

		/* Handle leapsecond insertion adjustments */
		if (unlikely(base >= tk->next_leap_ktime))
			*offs_real = ktime_sub(tk->offs_real, ktime_set(1, 0));

	} while (read_seqcount_retry(&tk_core.seq, seq));

	return base;
}

/*
 * timekeeping_validate_timex - Ensures the timex is ok for use in do_adjtimex
 */
static int timekeeping_validate_timex(const struct __kernel_timex *txc, bool aux_clock)
{
	if (txc->modes & ADJ_ADJTIME) {
		/* singleshot must not be used with any other mode bits */
		if (!(txc->modes & ADJ_OFFSET_SINGLESHOT))
			return -EINVAL;
		if (!(txc->modes & ADJ_OFFSET_READONLY) &&
		    !capable(CAP_SYS_TIME))
			return -EPERM;
	} else {
		/* In order to modify anything, you gotta be super-user! */
		if (txc->modes && !capable(CAP_SYS_TIME))
			return -EPERM;
		/*
		 * if the quartz is off by more than 10% then
		 * something is VERY wrong!
		 */
		if (txc->modes & ADJ_TICK &&
		    (txc->tick <  900000/USER_HZ ||
		     txc->tick > 1100000/USER_HZ))
			return -EINVAL;
	}

	if (txc->modes & ADJ_SETOFFSET) {
		/* In order to inject time, you gotta be super-user! */
		if (!capable(CAP_SYS_TIME))
			return -EPERM;

		/*
		 * Validate if a timespec/timeval used to inject a time
		 * offset is valid.  Offsets can be positive or negative, so
		 * we don't check tv_sec. The value of the timeval/timespec
		 * is the sum of its fields,but *NOTE*:
		 * The field tv_usec/tv_nsec must always be non-negative and
		 * we can't have more nanoseconds/microseconds than a second.
		 */
		if (txc->time.tv_usec < 0)
			return -EINVAL;

		if (txc->modes & ADJ_NANO) {
			if (txc->time.tv_usec >= NSEC_PER_SEC)
				return -EINVAL;
		} else {
			if (txc->time.tv_usec >= USEC_PER_SEC)
				return -EINVAL;
		}
	}

	/*
	 * Check for potential multiplication overflows that can
	 * only happen on 64-bit systems:
	 */
	if ((txc->modes & ADJ_FREQUENCY) && (BITS_PER_LONG == 64)) {
		if (LLONG_MIN / PPM_SCALE > txc->freq)
			return -EINVAL;
		if (LLONG_MAX / PPM_SCALE < txc->freq)
			return -EINVAL;
	}

	if (aux_clock) {
		/* Auxiliary clocks are similar to TAI and do not have leap seconds */
		if (txc->status & (STA_INS | STA_DEL))
			return -EINVAL;

		/* No TAI offset setting */
		if (txc->modes & ADJ_TAI)
			return -EINVAL;

		/* No PPS support either */
		if (txc->status & (STA_PPSFREQ | STA_PPSTIME))
			return -EINVAL;
	}

	return 0;
}

/**
 * random_get_entropy_fallback - Returns the raw clock source value,
 * used by random.c for platforms with no valid random_get_entropy().
 */
unsigned long random_get_entropy_fallback(void)
{
	struct tk_read_base *tkr = &tk_core.timekeeper.tkr_mono;
	struct clocksource *clock = READ_ONCE(tkr->clock);

	if (unlikely(timekeeping_suspended || !clock))
		return 0;
	return clock->read(clock);
}
EXPORT_SYMBOL_GPL(random_get_entropy_fallback);

struct adjtimex_result {
	struct audit_ntp_data	ad;
	struct timespec64	delta;
	bool			clock_set;
};

static int __do_adjtimex(struct tk_data *tkd, struct __kernel_timex *txc,
			 struct adjtimex_result *result)
{
	struct timekeeper *tks = &tkd->shadow_timekeeper;
	bool aux_clock = !timekeeper_is_core_tk(tks);
	struct timespec64 ts;
	s32 orig_tai, tai;
	int ret;

	/* Validate the data before disabling interrupts */
	ret = timekeeping_validate_timex(txc, aux_clock);
	if (ret)
		return ret;
	add_device_randomness(txc, sizeof(*txc));

	if (!aux_clock)
		ktime_get_real_ts64(&ts);
	else
		tk_get_aux_ts64(tkd->timekeeper.id, &ts);

	add_device_randomness(&ts, sizeof(ts));

	guard(raw_spinlock_irqsave)(&tkd->lock);

	if (!tks->clock_valid)
		return -ENODEV;

	if (txc->modes & ADJ_SETOFFSET) {
		result->delta.tv_sec  = txc->time.tv_sec;
		result->delta.tv_nsec = txc->time.tv_usec;
		if (!(txc->modes & ADJ_NANO))
			result->delta.tv_nsec *= 1000;
		ret = __timekeeping_inject_offset(tkd, &result->delta);
		if (ret)
			return ret;
		result->clock_set = true;
	}

	orig_tai = tai = tks->tai_offset;
	ret = ntp_adjtimex(tks->id, txc, &ts, &tai, &result->ad);

	if (tai != orig_tai) {
		__timekeeping_set_tai_offset(tks, tai);
		timekeeping_update_from_shadow(tkd, TK_CLOCK_WAS_SET);
		result->clock_set = true;
	} else {
		tk_update_leap_state_all(&tk_core);
	}

	/* Update the multiplier immediately if frequency was set directly */
	if (txc->modes & (ADJ_FREQUENCY | ADJ_TICK))
		result->clock_set |= __timekeeping_advance(tkd, TK_ADV_FREQ);

	return ret;
}

/**
 * do_adjtimex() - Accessor function to NTP __do_adjtimex function
 * @txc:	Pointer to kernel_timex structure containing NTP parameters
 */
int do_adjtimex(struct __kernel_timex *txc)
{
	struct adjtimex_result result = { };
	int ret;

	ret = __do_adjtimex(&tk_core, txc, &result);
	if (ret < 0)
		return ret;

	if (txc->modes & ADJ_SETOFFSET)
		audit_tk_injoffset(result.delta);

	audit_ntp_log(&result.ad);

	if (result.clock_set)
		clock_was_set(CLOCK_SET_WALL);

	ntp_notify_cmos_timer(result.delta.tv_sec != 0);

	return ret;
}

/*
 * Invoked from NTP with the time keeper lock held, so lockless access is
 * fine.
 */
long ktime_get_ntp_seconds(unsigned int id)
{
	return timekeeper_data[id].timekeeper.xtime_sec;
}

#ifdef CONFIG_NTP_PPS
/**
 * hardpps() - Accessor function to NTP __hardpps function
 * @phase_ts:	Pointer to timespec64 structure representing phase timestamp
 * @raw_ts:	Pointer to timespec64 structure representing raw timestamp
 */
void hardpps(const struct timespec64 *phase_ts, const struct timespec64 *raw_ts)
{
	guard(raw_spinlock_irqsave)(&tk_core.lock);
	__hardpps(phase_ts, raw_ts);
}
EXPORT_SYMBOL(hardpps);
#endif /* CONFIG_NTP_PPS */

#ifdef CONFIG_POSIX_AUX_CLOCKS
#include "posix-timers.h"

/*
 * Bitmap for the activated auxiliary timekeepers to allow lockless quick
 * checks in the hot paths without touching extra cache lines. If set, then
 * the state of the corresponding timekeeper has to be re-checked under
 * timekeeper::lock.
 */
static unsigned long aux_timekeepers;

static inline unsigned int clockid_to_tkid(unsigned int id)
{
	return TIMEKEEPER_AUX_FIRST + id - CLOCK_AUX;
}

static inline struct tk_data *aux_get_tk_data(clockid_t id)
{
	if (!clockid_aux_valid(id))
		return NULL;
	return &timekeeper_data[clockid_to_tkid(id)];
}

/* Invoked from timekeeping after a clocksource change */
static void tk_aux_update_clocksource(void)
{
	unsigned long active = READ_ONCE(aux_timekeepers);
	unsigned int id;

	for_each_set_bit(id, &active, BITS_PER_LONG) {
		struct tk_data *tkd = &timekeeper_data[id + TIMEKEEPER_AUX_FIRST];
		struct timekeeper *tks = &tkd->shadow_timekeeper;

		guard(raw_spinlock_irqsave)(&tkd->lock);
		if (!tks->clock_valid)
			continue;

		timekeeping_forward_now(tks);
		tk_setup_internals(tks, tk_core.timekeeper.tkr_mono.clock);
		timekeeping_update_from_shadow(tkd, TK_UPDATE_ALL);
	}
}

static void tk_aux_advance(void)
{
	unsigned long active = READ_ONCE(aux_timekeepers);
	unsigned int id;

	/* Lockless quick check to avoid extra cache lines */
	for_each_set_bit(id, &active, BITS_PER_LONG) {
		struct tk_data *aux_tkd = &timekeeper_data[id + TIMEKEEPER_AUX_FIRST];

		guard(raw_spinlock)(&aux_tkd->lock);
		if (aux_tkd->shadow_timekeeper.clock_valid)
			__timekeeping_advance(aux_tkd, TK_ADV_TICK);
	}
}

/**
 * ktime_get_aux - Get time for a AUX clock
 * @id:	ID of the clock to read (CLOCK_AUX...)
 * @kt:	Pointer to ktime_t to store the time stamp
 *
 * Returns: True if the timestamp is valid, false otherwise
 */
bool ktime_get_aux(clockid_t id, ktime_t *kt)
{
	struct tk_data *aux_tkd = aux_get_tk_data(id);
	struct timekeeper *aux_tk;
	unsigned int seq;
	ktime_t base;
	u64 nsecs;

	WARN_ON(timekeeping_suspended);

	if (!aux_tkd)
		return false;

	aux_tk = &aux_tkd->timekeeper;
	do {
		seq = read_seqcount_begin(&aux_tkd->seq);
		if (!aux_tk->clock_valid)
			return false;

		base = ktime_add(aux_tk->tkr_mono.base, aux_tk->offs_aux);
		nsecs = timekeeping_get_ns(&aux_tk->tkr_mono);
	} while (read_seqcount_retry(&aux_tkd->seq, seq));

	*kt = ktime_add_ns(base, nsecs);
	return true;
}
EXPORT_SYMBOL_GPL(ktime_get_aux);

/**
 * ktime_get_aux_ts64 - Get time for a AUX clock
 * @id:	ID of the clock to read (CLOCK_AUX...)
 * @ts:	Pointer to timespec64 to store the time stamp
 *
 * Returns: True if the timestamp is valid, false otherwise
 */
bool ktime_get_aux_ts64(clockid_t id, struct timespec64 *ts)
{
	ktime_t now;

	if (!ktime_get_aux(id, &now))
		return false;
	*ts = ktime_to_timespec64(now);
	return true;
}
EXPORT_SYMBOL_GPL(ktime_get_aux_ts64);

static int aux_get_res(clockid_t id, struct timespec64 *tp)
{
	if (!clockid_aux_valid(id))
		return -ENODEV;

	tp->tv_sec = aux_clock_resolution_ns() / NSEC_PER_SEC;
	tp->tv_nsec = aux_clock_resolution_ns() % NSEC_PER_SEC;
	return 0;
}

static int aux_get_timespec(clockid_t id, struct timespec64 *tp)
{
	return ktime_get_aux_ts64(id, tp) ? 0 : -ENODEV;
}

static int aux_clock_set(const clockid_t id, const struct timespec64 *tnew)
{
	struct tk_data *aux_tkd = aux_get_tk_data(id);
	struct timekeeper *aux_tks;
	ktime_t tnow, nsecs;

	if (!timespec64_valid_settod(tnew))
		return -EINVAL;
	if (!aux_tkd)
		return -ENODEV;

	aux_tks = &aux_tkd->shadow_timekeeper;

	guard(raw_spinlock_irq)(&aux_tkd->lock);
	if (!aux_tks->clock_valid)
		return -ENODEV;

	/* Forward the timekeeper base time */
	timekeeping_forward_now(aux_tks);
	/*
	 * Get the updated base time. tkr_mono.base has not been
	 * updated yet, so do that first. That makes the update
	 * in timekeeping_update_from_shadow() redundant, but
	 * that's harmless. After that @tnow can be calculated
	 * by using tkr_mono::cycle_last, which has been set
	 * by timekeeping_forward_now().
	 */
	tk_update_ktime_data(aux_tks);
	nsecs = timekeeping_cycles_to_ns(&aux_tks->tkr_mono, aux_tks->tkr_mono.cycle_last);
	tnow = ktime_add(aux_tks->tkr_mono.base, nsecs);

	/*
	 * Calculate the new AUX offset as delta to @tnow ("monotonic").
	 * That avoids all the tk::xtime back and forth conversions as
	 * xtime ("realtime") is not applicable for auxiliary clocks and
	 * kept in sync with "monotonic".
	 */
	aux_tks->offs_aux = ktime_sub(timespec64_to_ktime(*tnew), tnow);

	timekeeping_update_from_shadow(aux_tkd, TK_UPDATE_ALL);
	return 0;
}

static int aux_clock_adj(const clockid_t id, struct __kernel_timex *txc)
{
	struct tk_data *aux_tkd = aux_get_tk_data(id);
	struct adjtimex_result result = { };

	if (!aux_tkd)
		return -ENODEV;

	/*
	 * @result is ignored for now as there are neither hrtimers nor a
	 * RTC related to auxiliary clocks for now.
	 */
	return __do_adjtimex(aux_tkd, txc, &result);
}

const struct k_clock clock_aux = {
	.clock_getres		= aux_get_res,
	.clock_get_timespec	= aux_get_timespec,
	.clock_set		= aux_clock_set,
	.clock_adj		= aux_clock_adj,
};

static void aux_clock_enable(clockid_t id)
{
	struct tk_read_base *tkr_raw = &tk_core.timekeeper.tkr_raw;
	struct tk_data *aux_tkd = aux_get_tk_data(id);
	struct timekeeper *aux_tks = &aux_tkd->shadow_timekeeper;

	/* Prevent the core timekeeper from changing. */
	guard(raw_spinlock_irq)(&tk_core.lock);

	/*
	 * Setup the auxiliary clock assuming that the raw core timekeeper
	 * clock frequency conversion is close enough. Userspace has to
	 * adjust for the deviation via clock_adjtime(2).
	 */
	guard(raw_spinlock_nested)(&aux_tkd->lock);

	/* Remove leftovers of a previous registration */
	memset(aux_tks, 0, sizeof(*aux_tks));
	/* Restore the timekeeper id */
	aux_tks->id = aux_tkd->timekeeper.id;
	/* Setup the timekeeper based on the current system clocksource */
	tk_setup_internals(aux_tks, tkr_raw->clock);

	/* Mark it valid and set it live */
	aux_tks->clock_valid = true;
	timekeeping_update_from_shadow(aux_tkd, TK_UPDATE_ALL);
}

static void aux_clock_disable(clockid_t id)
{
	struct tk_data *aux_tkd = aux_get_tk_data(id);

	guard(raw_spinlock_irq)(&aux_tkd->lock);
	aux_tkd->shadow_timekeeper.clock_valid = false;
	timekeeping_update_from_shadow(aux_tkd, TK_UPDATE_ALL);
}

static DEFINE_MUTEX(aux_clock_mutex);

static ssize_t aux_clock_enable_store(struct kobject *kobj, struct kobj_attribute *attr,
				      const char *buf, size_t count)
{
	/* Lazy atoi() as name is "0..7" */
	int id = kobj->name[0] & 0x7;
	bool enable;

	if (!capable(CAP_SYS_TIME))
		return -EPERM;

	if (kstrtobool(buf, &enable) < 0)
		return -EINVAL;

	guard(mutex)(&aux_clock_mutex);
	if (enable == test_bit(id, &aux_timekeepers))
		return count;

	if (enable) {
		aux_clock_enable(CLOCK_AUX + id);
		set_bit(id, &aux_timekeepers);
	} else {
		aux_clock_disable(CLOCK_AUX + id);
		clear_bit(id, &aux_timekeepers);
	}
	return count;
}

static ssize_t aux_clock_enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	unsigned long active = READ_ONCE(aux_timekeepers);
	/* Lazy atoi() as name is "0..7" */
	int id = kobj->name[0] & 0x7;

	return sysfs_emit(buf, "%d\n", test_bit(id, &active));
}

static struct kobj_attribute aux_clock_enable_attr = __ATTR_RW(aux_clock_enable);

static struct attribute *aux_clock_enable_attrs[] = {
	&aux_clock_enable_attr.attr,
	NULL
};

static const struct attribute_group aux_clock_enable_attr_group = {
	.attrs = aux_clock_enable_attrs,
};

static int __init tk_aux_sysfs_init(void)
{
	struct kobject *auxo, *tko = kobject_create_and_add("time", kernel_kobj);

	if (!tko)
		return -ENOMEM;

	auxo = kobject_create_and_add("aux_clocks", tko);
	if (!auxo) {
		kobject_put(tko);
		return -ENOMEM;
	}

	for (int i = 0; i <= MAX_AUX_CLOCKS; i++) {
		char id[2] = { [0] = '0' + i, };
		struct kobject *clk = kobject_create_and_add(id, auxo);

		if (!clk)
			return -ENOMEM;

		int ret = sysfs_create_group(clk, &aux_clock_enable_attr_group);

		if (ret)
			return ret;
	}
	return 0;
}
late_initcall(tk_aux_sysfs_init);

static __init void tk_aux_setup(void)
{
	for (int i = TIMEKEEPER_AUX_FIRST; i <= TIMEKEEPER_AUX_LAST; i++)
		tkd_basic_setup(&timekeeper_data[i], i, false);
}
#endif /* CONFIG_POSIX_AUX_CLOCKS */
