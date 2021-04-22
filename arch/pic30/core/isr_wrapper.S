/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * PIC30 ISRs wrapper
 */

#include <toolchain.h>
#include <linker/sections.h>
#include <offsets_short.h>
#include <arch/cpu.h>
#include <sw_isr_table.h>
#include "macro_priv.inc"

/*
 * Wrapper around ISRs when inserted in software ISR table
 *
 * When inserted in the vector table, _isr_wrapper() demuxes the ISR table
 * using the running interrupt number as the index, and invokes the registered
 * ISR with its corresponding argument. When returning from the ISR, it
 * determines if a context switch needs to happen.
 */

GTEXT(__isr_wrapper)
SECTION_FUNC(isr, __isr_wrapper)
	z_pic30_enter_exc

	/* Increment nesting counter and switch to interrupt stack */
	inc	__kernel+_kernel_offset_to_nested
	dec	__kernel+_kernel_offset_to_nested, wreg
	bra	nz, 1f

	/* Disable stack checking */
	/*
	 * Set the SPLIM register to the top of RAM so it won't trigger
	 * the Stack Error Trap.
	 */
	setm	SPLIM

	/*
	 * Save the current stack pointer on the IRQ stack then switch
	 * to the IRQ stack.
	 */
	mov	__kernel+_kernel_offset_to_irq_stack, w0
	mov	w15, [w0++]
	mov	w0, w15

1:
#ifdef CONFIG_TRACING
	call	_sys_trace_isr_enter
#endif

	call	_z_pic30_isr_demux

#ifdef CONFIG_TRACING
	call	_sys_trace_isr_exit
#endif

	/* --(_kernel->nested) */
	dec	__kernel+_kernel_offset_to_nested
	bra	nz, exit

	/*
	 * z_arch_get_next_switch_handle() is returning:
	 *
	 * - The next thread to schedule in w0
	 * - The current thread in w1. This value is returned using the
	 *   **old_thread parameter, so we need to make space on the stack for
	 *   that.
	 */
	mov	#__kernel+_kernel_offset_to_current, w0
	push	w0
	call	_z_get_next_switch_handle
	pop	w1

	/* Switch back to the task stack pointer */
	pop	w15

	/*
	 * Check if new thread is the same as the current thread,
	 * if so, exit.
	 * w0: 1st thread in the ready queue
	 * w1: _current thread
	 */
	cp	w0, w1
	bra	z, exit

	/* Switch thread */

	/* Save callee saved registers, including stack pointer */
	add	#_thread_offset_to_callee_saved, w1
	mov.d	w8, [w1++]
	mov.d	w10, [w1++]
	mov.d	w12, [w1++]
	mov.d	w14, [w1++]

	/* Load callee saved registers, including stack pointer */
	add	#_thread_offset_to_callee_saved, w0
	mov.d	[w0++], w8
	mov.d	[w0++], w10
	mov.d	[w0++], w12
	mov.d	[w0++], w14

#ifdef CONFIG_INSTRUMENT_THREAD_SWITCHING
	call	_z_thread_mark_switched_in
#endif

#ifdef CONFIG_STACK_SENTINEL
	call	_z_check_stack_sentinel
#endif

exit:
	z_pic30_exit_exc
	retfie

