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

GDATA(_sw_isr_table)

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

#ifdef CONFIG_TRACING
	call	_sys_trace_isr_enter
#endif

	/* Disable stack checking */
	/*
	 * Set the SPLIM register to the top of RAM so it won't trigger
	 * the Stack Error Trap.
	 * SPLIM<0> is always 0
	 */
	setm	SPLIM
	nop

	/* ++(_kernel->nested) to be checked by arch_is_in_isr() */
	inc	__kernel+_kernel_offset_to_nested

	/* Switch to the IRQ stack */
	mov	__kernel+_kernel_offset_to_irq_stack, w15

	/* Get active IRQ number from the interrupt controller */
	mov	INTTREG, w0
	and	#0xff, w0

	/* Calculate the IFSx array offset */
	lsr	w0, #4, w1

	/* Calculate the IFSx bit offset */
	and	w0, #0xf, w2

	/* Calculate bit mask */
	mov	#1, w3
	sl	w3, w2, w2
	com	w2, w2

	/* Get the IFSx base address and add the offset */
	mov	#IFS0, w3
	add	w3, w1, w3

	/* Apply the bitmask */
	and	w2, [w3], [w3]

	/* Retrieve the interrupt service routine */
	mov	__sw_isr_table, w1
	mov	[w0+w1], w2
	mov.d	[w2], w0	/* arg in w0, ISR in w2 */

	/* Acknowledge the interrupt */

	/* Call the ISR */
	call	w2

#ifdef CONFIG_TRACING
	call	_sys_trace_isr_exit
#endif

	/* --(_kernel->nested) */
	dec	__kernel+_kernel_offset_to_nested

	cp0	w1
	bra	nz, exit

	/*
	 * z_arch_get_next_switch_handle() is returning:
	 *
	 * - The next thread to schedule in w0
	 * - The current thread in w1.
	 * - This value is returned using w0
	 */
	push	w0
	call	_z_arch_get_next_switch_handle
	pop	w1

	/*
	 * w0: 1st thread in the ready queue
	 * w1: _current thread
	 */
	cp	w0, w1
	bra	z, exit

	/* Switch thread */
	/*call	_z_pic30_context_switch */

exit:
#ifdef CONFIG_STACK_SENTINEL
	call	_z_check_stack_sentinel
#endif
	z_pic30_exit_exc
	retfie

