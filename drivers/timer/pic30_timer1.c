/*
 * Copyright (c) 2014-2015 Wind River Systems, Inc.
 * Copyright (c) 2018 Synopsys Inc, Inc.
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_pic30_timer1

#include <drivers/timer/system_timer.h>
#include <irq.h>
#include <sys_clock.h>
#include <spinlock.h>
#include <soc.h>

#define PIC30_TIMER1_PRESCALER						\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(0, prescaler),		\
		    (DT_INST_PROP(0, prescaler)), (1))

#define PIC30_TIMER1_BASE		DT_INST_REG_ADDR(0)
#if defined (CONFIG_CPU_DSPIC33C)
#define PIC30_TIMER1_T1CON		((PIC30_TIMER1_BASE) + 0x00)
#define PIC30_TIMER1_TMR1 		((PIC30_TIMER1_BASE) + 0x04)
#define PIC30_TIMER1_PR1 		((PIC30_TIMER1_BASE) + 0x08)
#define PIC30_TIMER1_T1CON_TON		(1U << 15)
#define PIC30_TIMER1_T1CON_TECS(x)	(((x)&(0x3U))<<8)
#define PIC30_TIMER1_T1CON_TGATE	(1U << 7)
#define PIC30_TIMER1_T1CON_TCKPS(x)	(((x)&(0x3U))<<4)
#define PIC30_TIMER1_T1CON_TSYNC	(1U << 2)
#define PIC30_TIMER1_T1CON_TCS		(1U << 1)
#else
#define PIC30_TIMER1_T1CON		((PIC30_TIMER1_BASE) + 0x04)
#define PIC30_TIMER1_TMR1 		((PIC30_TIMER1_BASE) + 0x00)
#define PIC30_TIMER1_PR1 		((PIC30_TIMER1_BASE) + 0x02)
#define PIC30_TIMER1_T1CON_TON		(1U << 15)
#define PIC30_TIMER1_T1CON_TGATE	(1U << 6)
#define PIC30_TIMER1_T1CON_TCKPS(x)	(((x)&(0x3U))<<4)
#define PIC30_TIMER1_T1CON_TSYNC	(1U << 2)
#define PIC30_TIMER1_T1CON_TCS		(1U << 1)
#endif


#define COUNTER_MAX		0xFFFFu
#define TIMER_STOPPED		0x0
#define CYC_PER_TICK		(sys_clock_hw_cycles_per_sec()	\
					/ CONFIG_SYS_CLOCK_TICKS_PER_SEC \
					/ PIC30_TIMER1_PRESCALER)

#define MAX_TICKS		((COUNTER_MAX / CYC_PER_TICK) - 1)
#define MAX_CYCLES		(MAX_TICKS * CYC_PER_TICK)

/* Minimum cycles in the future to try to program.  Note that this is
 * NOT simply "enough cycles to get the counter read and reprogrammed
 * reliably" -- it becomes the minimum value of the LOAD register, and
 * thus reflects how much time we can reliably see expire between
 * calls to elapsed() to read the COUNTFLAG bit.  So it needs to be
 * set to be larger than the maximum time the interrupt might be
 * masked.  Choosing a fraction of a tick is probably a good enough
 * default, with an absolute minimum of 1k cyc.
 */
#define MIN_DELAY		MAX(1024, (CYC_PER_TICK/16))

#define TICKLESS		(IS_ENABLED(CONFIG_TICKLESS_KERNEL))

static struct k_spinlock lock;

static uint16_t last_load;

/*
 * This local variable holds the amount of timer cycles elapsed
 * and it is updated in z_timer_int_handler and z_clock_set_timeout().
 *
 * Note:
 *  At an arbitrary point in time the "current" value of the
 *  HW timer is calculated as:
 *
 * t = cycle_counter + elapsed();
 */
static uint32_t cycle_count;

/*
 * This local variable holds the amount of elapsed HW cycles
 * that have been announced to the kernel.
 */
static uint32_t announced_cycles;

/**
 *
 * @brief Get contents of Timer1 count register
 *
 * @return Current Timer1 count
 */
static ALWAYS_INLINE uint16_t timer1_count_register_get(void)
{
	return sys_read16(PIC30_TIMER1_TMR1);
}

/**
 *
 * @brief Set Timer1 count register to the specified value
 *
 * @return N/A
 */
static ALWAYS_INLINE void timer1_count_register_set(uint16_t value)
{
	sys_write16(value, PIC30_TIMER1_TMR1);
}

/**
 *
 * @brief Get contents of Timer1 control register
 *
 * @return N/A
 */
static ALWAYS_INLINE uint16_t timer1_control_register_get(void)
{
	return sys_read16(PIC30_TIMER1_T1CON);
}

/**
 *
 * @brief Set Timer1 control register to the specified value
 *
 * @return N/A
 */
static ALWAYS_INLINE void timer1_control_register_set(uint16_t value)
{
	sys_write16(value, PIC30_TIMER1_T1CON);
}

/**
 *
 * @brief Get contents of Timer1 period register
 *
 * @return N/A
 */
static ALWAYS_INLINE uint16_t timer1_period_register_get(void)
{
	return sys_read16(PIC30_TIMER1_PR1);
}

/**
 *
 * @brief Set Timer1 period register to the specified value
 *
 * @return N/A
 */
static ALWAYS_INLINE void timer1_period_register_set(uint16_t count)
{
	sys_write16(count, PIC30_TIMER1_PR1);
}

/**
 *
 * @brief Enable Timer1
 *
 * @return N/A
 */
static ALWAYS_INLINE void timer1_enable(void)
{
	uint16_t reg = sys_read16(PIC30_TIMER1_T1CON);
	sys_write16(reg | PIC30_TIMER1_T1CON_TON, PIC30_TIMER1_T1CON);
}

/**
 *
 * @brief Disable Timer1
 *
 * @return N/A
 */
static ALWAYS_INLINE void timer1_disable(void)
{
	uint16_t reg = sys_read16(PIC30_TIMER1_T1CON);
	sys_write16(reg & ~PIC30_TIMER1_T1CON_TON, PIC30_TIMER1_T1CON);
}

static void pic30_timer_irq_handler(const void *device)
{
	/* timer_int_handler may be triggered by timer irq or
	 * software helper irq
	 */

	ARG_UNUSED(device);

	if (TICKLESS) {
		uint32_t dticks;

		/* In TICKLESS mode, the LOAD is re-programmed
		 * in z_clock_set_timeout(), followed by resetting of
		 * the counter (VAL = 0).
		 *
		 * If a timer wrap occurs right when we re-program LOAD,
		 * the ISR is triggered immediately after z_clock_set_timeout()
		 * returns; in that case we shall not increment the cycle_count
		 * because the value has been updated before LOAD re-program.
		 *
		 * We can assess if this is the case by inspecting COUNTFLAG.
		 */
		/*
		 * Use builtin function instead of generic C version:
		 * dticks = (cycle_count - announced_cycles) / CYC_PER_TICK;
		 */
		dticks = __builtin_divud((cycle_count - announced_cycles),
				CYC_PER_TICK);
		announced_cycles += dticks * CYC_PER_TICK;
		z_clock_announce(dticks);
	} else {
		z_clock_announce(1);
	}
}

void z_clock_set_timeout(int32_t ticks, bool idle)
{
	/* Fast CPUs and a 16 bit counter mean that even idle systems
	 * need to wake up multiple times per second.  If the kernel
	 * allows us to miss tick announcements in idle, then shut off
	 * the counter. (Note: we can assume if idle==true that
	 * interrupts are already disabled)
	 */
	if (IS_ENABLED(CONFIG_TICKLESS_IDLE) && idle
		&& ticks == K_TICKS_FOREVER) {
		timer1_disable();
		last_load = TIMER_STOPPED;
		return;
	}

#if defined(CONFIG_TICKLESS_KERNEL)
	uint32_t delay;
	uint32_t unannounced;

	ticks = (ticks == K_TICKS_FOREVER) ? MAX_TICKS : ticks;
	ticks = CLAMP(ticks - 1, 0, (int32_t)MAX_TICKS);

	k_spinlock_key_t key = k_spin_lock(&lock);

	cycle_count += timer1_count_register_get();
	/* clear counter early to avoid cycle loss as few as possible,
	 * between cycle_count and clearing 0, few cycles are possible
	 * to loss
	 */
	timer1_disable();
	timer1_count_register_set(0);

	/* normal case */
	unannounced = cycle_count - announced_cycles;

	if ((int32_t)unannounced < 0) {
		/* We haven't announced for more than half the 16-bit
		 * wrap duration, because new timeouts keep being set
		 * before the existing one fires. Force an announce
		 * to avoid loss of a wrap event, making sure the
		 * delay is at least the minimum delay possible.
		 */
		last_load = MIN_DELAY;
	} else {
		/* Desired delay in the future */
		delay = ticks * CYC_PER_TICK;

		/* Round delay up to next tick boundary */
		delay += unannounced;
		delay =
		 ((delay + CYC_PER_TICK - 1) / CYC_PER_TICK) * CYC_PER_TICK;

		delay -= unannounced;
		delay = MAX(delay, MIN_DELAY);

		last_load = MIN(delay, MAX_CYCLES);
	}

	timer1_period_register_set(last_load - 1);
	timer1_enable();

	k_spin_unlock(&lock, key);
#endif
}

uint32_t z_clock_elapsed(void)
{
	if (!TICKLESS) {
		return 0;
	}

	uint32_t cyc;

	k_spinlock_key_t key = k_spin_lock(&lock);

	cyc = timer1_count_register_get() + cycle_count - announced_cycles;

	k_spin_unlock(&lock, key);

	/*
	 * cyc / CYC_PER_TICK
	 */
	return __builtin_divud(cyc, CYC_PER_TICK);
}

uint32_t z_timer_cycle_get_32(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t ret = timer1_count_register_get() + cycle_count;

	k_spin_unlock(&lock, key);
	return ret;
}

int z_clock_driver_init(const struct device *device)
{
	ARG_UNUSED(device);
	uint16_t ctrl;

	last_load = CYC_PER_TICK;
	announced_cycles = 0;

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority),
			pic30_timer_irq_handler, NULL, 0);

	/* Disable timer */
	timer1_control_register_set(0);
	timer1_period_register_set(last_load - 1);
	timer1_count_register_set(0);

	ctrl = PIC30_TIMER1_T1CON_TECS(1) |
		PIC30_TIMER1_T1CON_TCKPS(PIC30_TIMER1_PRESCALER / 8);
	timer1_control_register_set(ctrl);

	irq_enable(DT_INST_IRQN(0));

	/* Enable timer */
	timer1_enable();

	return 0;
}
