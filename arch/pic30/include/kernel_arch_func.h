/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Private kernel definitions
 *
 * This file contains private kernel function/macro definitions and various
 * other definitions for the PIC30 processor architecture.
 */

#ifndef ZEPHYR_ARCH_PIC30_INCLUDE_KERNEL_ARCH_FUNC_H_
#define ZEPHYR_ARCH_PIC30_INCLUDE_KERNEL_ARCH_FUNC_H_

#include <kernel_arch_data.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _ASMLANGUAGE
static ALWAYS_INLINE void arch_kernel_init(void)
{
}

/* PIC30 currently supports USE_SWITCH and USE_SWITCH only */
void arch_switch(void *switch_to, void **switched_from);

static inline bool arch_is_in_isr(void)
{
	return _kernel.cpus[0].nested != 0U;
}

#ifdef CONFIG_IRQ_OFFLOAD
int z_irq_do_offload(void);
#endif

#endif /* _ASMLANGUAGE */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_ARCH_PIC30_INCLUDE_KERNEL_ARCH_FUNC_H_ */
