/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief New thread creation for PIC30
 *
 * Core thread related primitives for the PIC30
 */

#include <kernel.h>
#include <ksched.h>
#include <wait_q.h>
#include <arch/cpu.h>

/*
 * An initial context, to be "restored" by z_pic30_context_switch(), is put at
 * the other end of the stack, and thus reusable by the stack when not needed
 * anymore.
 */
void arch_new_thread(struct k_thread *thread, k_thread_stack_t *stack,
		     char *stack_ptr, k_thread_entry_t entry,
		     void *p1, void *p2, void *p3)
{
	z_arch_esf_t *pInitCtx;

	pInitCtx = (z_arch_esf_t *)stack_ptr;

	/* Emulate the stack frame of an interrupt call */

	/* Set SR to enable all interrupts */
	pInitCtx->pcl_sfa = (uint16_t)z_thread_entry;
	pInitCtx->srl_ipl3_pch = 0;

	pInitCtx->w0 = (uint16_t)entry;
	pInitCtx->w1 = (uint16_t)p1;
	pInitCtx->w2 = (uint16_t)p2;
	pInitCtx->w3 = (uint16_t)p3;
	pInitCtx->w4 = 0x4444u;
	pInitCtx->w5 = 0x5555u;
	pInitCtx->w6 = 0x6666u;
	pInitCtx->w7 = 0x7777u;

#if defined(CONFIG_PIC30_DSP)
	pInitCtx->accal = 0x000a;
	pInitCtx->accah = 0x00aa;
	pInitCtx->accau = 0x0aaa;
	pInitCtx->accbl = 0x000b;
	pInitCtx->accbh = 0x00bb;
	pInitCtx->accbu = 0x0bbb;
#endif /* CONFIG_PIC30_DSP */

	pInitCtx->tblpag = TBLPAG;

#if defined(CONFIG_PIC30_EDS)
	pInitCtx->dsrpag = DSRPAG;
	pInitCtx->dswpag = DSWPAG;
#else
	pInitCtx->psvpag = PSVPAG;
#endif /* CONFIG_PIC30_EDS */

	pInitCtx->splim = thread->stack_info.start + thread->stack_info.size;

	/* Set SR to enable all interrupts */
	pInitCtx->sr = 0U;
	pInitCtx->corcon = CORCON;

	thread->callee_saved.w8 = 0x8888u;
	thread->callee_saved.w9 = 0x9999u;
	thread->callee_saved.w10 = 0xaaaau;
	thread->callee_saved.w11 = 0xbbbbu;
	thread->callee_saved.w12 = 0xccccu;
	thread->callee_saved.w13 = 0xddddu;
	thread->callee_saved.w14 = 0xeeeeu;
	thread->callee_saved.w15 = (uint16_t)(stack_ptr + sizeof(z_arch_esf_t));

	thread->switch_handle = thread;
}

void *z_arch_get_next_switch_handle(struct k_thread **old_thread)
{
	*old_thread = _current;

	return z_get_next_switch_handle(*old_thread);
}
