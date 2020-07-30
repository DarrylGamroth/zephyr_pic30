/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Either public functions or macros or invoked by public functions */

#ifndef ZEPHYR_INCLUDE_ARCH_PIC30_ASM_INLINE_GCC_H_
#define ZEPHYR_INCLUDE_ARCH_PIC30_ASM_INLINE_GCC_H_

/*
 * The file must not be included directly
 * Include arch/cpu.h instead
 */

#ifndef _ASMLANGUAGE

#include <zephyr/types.h>
#include <irq.h>

#ifdef __cplusplus
extern "C" {
#endif

/* On PIC30 CPUs, this function prevents regular
 * exceptions (i.e. with interrupt priority lower than or equal to
 * _EXC_IRQ_DEFAULT_PRIO) from interrupting the CPU. NMI, Faults,
 * and Zero Latency IRQs (if supported) may still interrupt the CPU.
 */

static ALWAYS_INLINE unsigned int arch_irq_lock(void)
{
	unsigned int key;

#if 0
	unsigned int tmp;

	__asm__ volatile(
		"mov.b SRL,%0;"
        "lsr.b %0, #5, %0;"

		"mrs %0, BASEPRI;"
		"msr BASEPRI, %1;"
		"isb;"
		: "=r"(key), "=r"(tmp)
		: "i"(_EXC_IRQ_DEFAULT_PRIO)
		: "memory");
#endif

	return key;
}


static ALWAYS_INLINE void arch_irq_unlock(unsigned int key)
{
#if 0
	__asm__ volatile(
		"msr BASEPRI, %0;"
		"isb;"
		:  : "r"(key) : "memory");
#endif
}

static ALWAYS_INLINE bool arch_irq_unlocked(unsigned int key)
{
	/* This convention works for both PRIMASK and BASEPRI */
	return key == 0;
}

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_PIC30_ASM_INLINE_GCC_H_ */
