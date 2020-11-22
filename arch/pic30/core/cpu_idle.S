/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief CPU power management
 *
 * CPU power management routines.
 */

#include <toolchain.h>
#include <linker/sections.h>
#include <arch/cpu.h>

GTEXT(_arch_cpu_idle)
GTEXT(_arch_cpu_atomic_idle)

/*
 * @brief Put the CPU in low-power mode
 *
 * This function always exits with interrupts unlocked.
 *
 * void arch_cpu_idle(void)
 */
SECTION_FUNC(text, _arch_cpu_idle)
#ifdef CONFIG_TRACING
	call	_sys_trace_idle
#endif
	disi	#3
	mov	#0xff1f, w0
	and	SR
	pwrsav	#1
	return

/*
 * @brief Put the CPU in low-power mode, entered with IRQs locked
 *
 * This function exits with interrupts restored to <key>.
 *
 * void arch_cpu_atomic_idle(unsigned int key)
 */
SECTION_FUNC(text, _arch_cpu_atomic_idle)
#ifdef CONFIG_TRACING
	call	_sys_trace_idle
#endif
	push	w0
	disi	#3
	mov	#0xff1f, w0
	and	SR
	pwrsav	#1
	pop	SR
	return
