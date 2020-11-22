/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Thread context switching for PIC30
 *
 * This module implements the routines necessary for thread context switching
 * on PIC30
 */

#include <toolchain.h>
#include <linker/sections.h>
#include <offsets_short.h>
#include <arch/cpu.h>
#include <syscall.h>
#include "macro_priv.inc"

/*
 * Routine to handle context switches
 *
 * This function is directly called either by _isr_wrapper() in case of
 * preemption, or z_pic30_sync_exc() in case of cooperative switching.
 */

GTEXT(_z_pic30_arch_switch)
SECTION_FUNC(text, _z_pic30_arch_switch)

	/*
	 *  - w0 = new_thread->switch_handle = switch_to thread
	 *  - w1 = &old_thread->switch_handle = current thread
	 */

	/*
	 * The PIC30 family does not support software traps so build an
	 * interrupt stack frame to emulate an trap.
	 */
	mov	SR, w2
	sl	w2, #8, w2
	btsc	CORCON, #IPL3
	bset	w2, #7
	ior	w2, [--w15], w2
	mov	w2, [w15++]
	
	z_pic30_enter_exc

	/*
	 * Set the SPLIM register to the top of RAM so it won't trigger
	 * the Stack Error Trap.
	 * SPLIM<0> is always 0
	 */
	setm	SPLIM

	/* Save callee saved registers, including stack pointer */
	add	#_thread_offset_to_callee_saved, w1
	mov.d	w8, [w1++]
	mov.d	w10, [w1++]
	mov.d	w12, [w1++]
	mov.d	w14, [w1++]

	/* Load callee saved registers, including stack pointer */
	add	#_thread_offset_to_callee_saved, w0
	mov.d	[--w0], w14
	mov.d	[--w0], w12
	mov.d	[--w0], w10
	mov.d	[--w0], w8

#ifdef CONFIG_INSTRUMENT_THREAD_SWITCHING
	call	_z_thread_mark_switched_in
#endif
	z_pic30_exit_exc

	retfie

/*
 * Synchronous exceptions handler
 *
 * The service call (SVC) is used in the following occasions:
 * - IRQ offloading
 */

GTEXT(z_pic30_sync_exc)
SECTION_FUNC(text, z_pic30_sync_exc)
	z_pic30_enter_exc

	/* Demux Hard Trap */
#ifdef CONFIG_IRQ_OFFLOAD
	cp	INTCON4, #SGHT
	bra	nz, offload
#endif
#ifdef CONIFG_ECCDBE
	cp	INTCON4, #ECCDBE
	bra	eccdbe
#endif
	bra	invalid

#ifdef CONFIG_IRQ_OFFLOAD
offload:
	/* ++(_kernel->nested) to be checked by arch_is_in_isr() */
	inc_nest_counter w0, w1

	/* Clear SWTRAP flag */
	bclr	INTCON2, #SWTRAP
	bclr	INTCON4, #SGHT

	call	_z_irq_do_offload

	/* --(_kernel->nested) */
	dec_nest_counter w0, w1
	bra	exit
#endif

#ifdef CONIFG_ECCDBE
eccdbe:
	bclr	INTCON4, #ECCDBE
	bra	eccdbe
#endif

invalid:
	mov	#0, w0	/* K_ERR_CPU_EXCEPTION */
	mov	w15, w1
	call	_z_pic30_fatal_error

exit:
	z_pic30_enter_exc
	retfie

#ifdef CONFIG_IRQ_OFFLOAD
GTEXT(z_pic30_offload)
SECTION_FUNC(text, z_pic30_offload)
	bset	INTCON2, #SWTRAP
	return
#endif

