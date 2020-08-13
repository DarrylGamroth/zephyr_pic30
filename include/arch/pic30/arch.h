/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PIC30 specific kernel interface header
 * This header contains the PIC30 specific kernel interface.  It is
 * included by the generic kernel interface header (arch/cpu.h)
 */

#ifndef ZEPHYR_INCLUDE_ARCH_PIC30_ARCH_H_
#define ZEPHYR_INCLUDE_ARCH_PIC30_ARCH_H_

#include <arch/pic30/thread.h>
#include <arch/pic30/exp.h>
#include <arch/common/sys_io.h>
#include <arch/common/ffs.h>

#include <irq.h>
#include <sw_isr_table.h>
#include <soc.h>
#include <devicetree.h>

/* stacks, for PIC30 architecture stack should be 2byte-aligned */
#define ARCH_STACK_PTR_ALIGN  2

#ifndef _ASMLANGUAGE
#include <sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STACK_ROUND_UP(x) ROUND_UP(x, ARCH_STACK_PTR_ALIGN)

/* macros convert value of its argument to a string */
#define DO_TOSTR(s) #s
#define TOSTR(s) DO_TOSTR(s)

/* concatenate the values of the arguments into one */
#define DO_CONCAT(x, y) x ## y
#define CONCAT(x, y) DO_CONCAT(x, y)

void arch_irq_enable(unsigned int irq);
void arch_irq_disable(unsigned int irq);
int arch_irq_is_enabled(unsigned int irq);
void arch_irq_priority_set(unsigned int irq, unsigned int prio);
void z_irq_spurious(void *unused);

#define ARCH_IRQ_CONNECT(irq_p, priority_p, isr_p, isr_param_p, flags_p) \
{ \
	Z_ISR_DECLARE(irq_p, 0, isr_p, isr_param_p); \
}

static ALWAYS_INLINE unsigned int arch_irq_lock(void)
{
	unsigned int key;

    key = __builtin_get_isr_state();
    __builtin_disable_interrupts();

	return key;
}

static ALWAYS_INLINE void arch_irq_unlock(unsigned int key)
{
    __builtin_set_isr_state(key);
}

static ALWAYS_INLINE bool arch_irq_unlocked(unsigned int key)
{
	return __builtin_get_isr_state() == key;
}

static ALWAYS_INLINE void arch_nop(void)
{
    __builtin_nop();
}

extern uint32_t z_timer_cycle_get_32(void);

static inline uint32_t arch_k_cycle_get_32(void)
{
#if 0
	return z_timer_cycle_get_32();
#endif
    return 0UL;
}

#ifdef __cplusplus
}
#endif

#endif /*_ASMLANGUAGE */

#endif
