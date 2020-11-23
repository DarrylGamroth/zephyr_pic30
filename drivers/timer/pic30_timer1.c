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

#define TIMER1_BASE	        DT_INST_REG_ADDR(0)
#define TIMER1_T1CON		((TIMER1_BASE) + 0x00)
#define TIMER1_TMR1 		((TIMER1_BASE) + 0x20)
#define TIMER1_PR1 		((TIMER1_BASE) + 0x40)

#define TIMER1_T1CON_TON    (1U << 15)
#define TIMER1_T1CON_SIDL   (1U << 13)
#define TIMER1_T1CON_TWDIS  (1U << 12)
#define TIMER1_T1CON_TWIP   (1U << 11)
#define TIMER1_T1CON_PRWIP  (1U << 10)
#define TIMER1_T1CON_TECS(x)    (((x)&(0x3U))<<8)
#define TIMER1_T1CON_TGATE  (1U << 7)
#define TIMER1_T1CON_TCKPS(x)   (((x)&(0x3U))<<4)
#define TIMER1_T1CON_TSYNC  (1U << 2)
#define TIMER1_T1CON_TCS    (1U << 1)

/* Minimum cycles in the future to try to program. */
#define MIN_DELAY 1024
/* Timer1 timer has 16 bit, here use 15 bit to avoid the possible
 * overflow,e.g, 0xffff + any value will cause overflow
 */
#define COUNTER_MAX 0x7fff
#define TIMER1_STOPPED 0x0
#define CYC_PER_TICK (sys_clock_hw_cycles_per_sec()	\
		      / CONFIG_SYS_CLOCK_TICKS_PER_SEC)

#define MAX_TICKS ((COUNTER_MAX / CYC_PER_TICK) - 1)
#define MAX_CYCLES (MAX_TICKS * CYC_PER_TICK)

#define TICKLESS (IS_ENABLED(CONFIG_TICKLESS_KERNEL))

static struct k_spinlock lock;

static uint32_t last_load;

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


/*
 * This local variable holds the amount of elapsed HW cycles due to
 * timer wraps ('overflows') and is used in the calculation
 * in elapsed() function, as well as in the updates to cycle_count.
 *
 * Note:
 * Each time cycle_count is updated with the value from overflow_cycles,
 * the overflow_cycles must be reset to zero.
 */
static volatile uint32_t overflow_cycles;

/**
 *
 * @brief Get contents of Timer1 count register
 *
 * @return Current Timer1 count
 */
static ALWAYS_INLINE uint16_t timer1_count_register_get(void)
{
	return sys_read16(TIMER1_TMR1);
}

/**
 *
 * @brief Set Timer1 count register to the specified value
 *
 * @return N/A
 */
static ALWAYS_INLINE void timer1_count_register_set(uint16_t value)
{
    sys_write16(value, TIMER1_TMR1);
}

/**
 *
 * @brief Get contents of Timer1 control register
 *
 * @return N/A
 */
static ALWAYS_INLINE uint16_t timer1_control_register_get(void)
{
	return sys_read16(TIMER1_T1CON);
}

/**
 *
 * @brief Set Timer0 control register to the specified value
 *
 * @return N/A
 */
static ALWAYS_INLINE void timer1_control_register_set(uint16_t value)
{
    sys_write16(value, TIMER1_T1CON);
}

/**
 *
 * @brief Get contents of Timer1 limit register
 *
 * @return N/A
 */
static ALWAYS_INLINE uint16_t timer1_limit_register_get(void)
{
	return sys_read16(TIMER1_PR1);
}

/**
 *
 * @brief Set Timer1 limit register to the specified value
 *
 * @return N/A
 */
static ALWAYS_INLINE void timer1_limit_register_set(uint16_t count)
{
	sys_write16(count, TIMER1_PR1);
}

/**
 *
 * @brief Enable Timer1
 *
 * @return N/A
 */
static ALWAYS_INLINE void timer1_enable(void)
{
    uint16_t reg = sys_read16(TIMER1_T1CON);
	sys_write16(reg | TIMER1_T1CON_TON, TIMER1_T1CON);
}

/**
 *
 * @brief Disable Timer1
 *
 * @return N/A
 */
static ALWAYS_INLINE void timer1_disable(void)
{
    uint16_t reg = sys_read16(TIMER1_T1CON);
	sys_write16(reg & ~TIMER1_T1CON_TON, TIMER1_T1CON);
}

/* This internal function calculates the amount of HW cycles that have
 * elapsed since the last time the absolute HW cycles counter has been
 * updated. 'cycle_count' may be updated either by the ISR, or
 * in z_clock_set_timeout().
 *
 * Additionally, the function updates the 'overflow_cycles' counter, that
 * holds the amount of elapsed HW cycles due to (possibly) multiple
 * timer wraps (overflows).
 *
 * Prerequisites:
 * - reprogramming of LIMIT must be clearing the COUNT
 * - ISR must be clearing the 'overflow_cycles' counter.
 * - no more than one counter-wrap has occurred between
 *     - the timer reset or the last time the function was called
 *     - and until the current call of the function is completed.
 * - the function is invoked with interrupts disabled.
 */
static uint32_t elapsed(void)
{
	uint16_t val;
    uint16_t ctrl;

	do {
		val =  timer1_count_register_get();
		ctrl = timer1_control_register_get();
	} while (timer1_count_register_get() < val);
#if 0
	if (ctrl & _ARC_V2_TMR_CTRL_IP) {
		overflow_cycles += last_load;
		/* clear the IP bit of the control register */
		timer1_control_register_set(_ARC_V2_TMR_CTRL_NH |
					    _ARC_V2_TMR_CTRL_IE);
		/* use sw triggered irq to remember the timer irq request
		 * which may be cleared by the above operation. when elapsed ()
		 * is called in z_timer_int_handler, no need to do this.
		 */
        /* If not in ISR or not in Timer ISR */
		if (!z_arc_v2_irq_unit_is_in_isr() ||
		    z_arc_v2_aux_reg_read(_ARC_V2_ICAUSE) != IRQ_TIMER0) {
			z_arc_v2_aux_reg_write(_ARC_V2_AUX_IRQ_HINT,
					       IRQ_TIMER0);
		}
	}
#endif

	return val + overflow_cycles;
}

static void pic30_timer_irq_handler(const void *device)
{
	/* timer_int_handler may be triggered by timer irq or
	 * software helper irq
	 */

	ARG_UNUSED(device);
	uint32_t dticks;
	uint32_t key;

	/* irq with higher priority may call z_clock_set_timeout
	 * so need a lock here
	 */
	key = arch_irq_lock();

	elapsed();
	cycle_count += overflow_cycles;
	overflow_cycles = 0;

	arch_irq_unlock(key);

	dticks = (cycle_count - announced_cycles) / CYC_PER_TICK;
	announced_cycles += dticks * CYC_PER_TICK;
	z_clock_announce(TICKLESS ? dticks : 1);
}

void z_clock_set_timeout(int32_t ticks, bool idle)
{
	/* If the kernel allows us to miss tick announcements in idle,
	 * then shut off the counter. (Note: we can assume if idle==true
	 * that interrupts are already disabled)
	 */
	if (IS_ENABLED(CONFIG_TICKLESS_IDLE) && idle
	    && ticks == K_TICKS_FOREVER) {
        timer1_disable();
		last_load = TIMER1_STOPPED;
		return;
	}

#if defined(CONFIG_TICKLESS_KERNEL)
	uint32_t delay;
	uint32_t unannounced;

	ticks = MIN(MAX_TICKS, (uint32_t)(MAX((int32_t)(ticks - 1L), 0)));

	k_spinlock_key_t key = k_spin_lock(&lock);

	cycle_count += elapsed();
	/* clear counter early to avoid cycle loss as few as possible,
	 * between cycle_count and clearing 0, few cycles are possible
	 * to loss
	 */
    timer1_disable();
	timer1_count_register_set(0);
	overflow_cycles = 0U;

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

	timer1_limit_register_set(last_load - 1);
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

	cyc =  elapsed() + cycle_count - announced_cycles;

	k_spin_unlock(&lock, key);

	return cyc / CYC_PER_TICK;
}

uint32_t z_timer_cycle_get_32(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t ret = elapsed() + cycle_count;

	k_spin_unlock(&lock, key);
	return ret;
}

int z_clock_driver_init(const struct device *device)
{
	ARG_UNUSED(device);
    uint16_t ctrl;

	last_load = CYC_PER_TICK;
	overflow_cycles = 0;
	announced_cycles = 0;

	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority),
			pic30_timer_irq_handler, NULL, 0);

    /* Disable timer */
	timer1_control_register_set(0);
	timer1_limit_register_set(last_load - 1);
	timer1_count_register_set(0);

    ctrl = TIMER1_T1CON_SIDL | TIMER1_T1CON_TECS(1) |
        TIMER1_T1CON_TCKPS(0);
    timer1_control_register_set(ctrl);

	irq_enable(DT_INST_IRQN(0));

    /* Enable timer */
    timer1_enable();

	return 0;
}
