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

#include <arch/pic30/cpu.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

static ALWAYS_INLINE unsigned int arch_irq_lock(void)
{
	unsigned int key;
#if 0
	__asm__ volatile("mrs %0, daif;"
			 "msr daifset, %1;"
			 "isb"
			 : "=r" (key)
			 : "i" (DAIFSET_IRQ)
			 : "memory", "cc");
#endif
    SET_AND_SAVE_CPU_IPL(key, 6);

	return key;
}

static ALWAYS_INLINE void arch_irq_unlock(unsigned int key)
{
#if 0
	__asm__ volatile("msr daif, %0;"
			 "isb"
			 :
			 : "r" (key)
			 : "memory", "cc");
#endif
    RESTORE_CPU_IPL(key);
}

static ALWAYS_INLINE bool arch_irq_unlocked(unsigned int key)
{
    return (key == 0);
}

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_PIC30_ASM_INLINE_GCC_H_ */
