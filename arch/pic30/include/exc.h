/*
 * Copyright (c) 2019 Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Exception/interrupt context helpers for PIC30 CPUs
 *
 * Exception/interrupt context helpers.
 */

#ifndef ZEPHYR_ARCH_PIC30_INCLUDE_EXC_H_
#define ZEPHYR_ARCH_PIC30_INCLUDE_EXC_H_

#include <arch/cpu.h>

#ifdef _ASMLANGUAGE

/* nothing */

#else

#include <irq_offload.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_IRQ_OFFLOAD)
extern void z_pic30_offload(void);
#endif

static ALWAYS_INLINE bool arch_is_in_isr(void)
{
	return _kernel.cpus[0].nested != 0U;
}

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_ARCH_PIC30_INCLUDE_EXC_H_ */
