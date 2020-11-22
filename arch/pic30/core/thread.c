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

	//pInitCtx = Z_STACK_PTR_TO_FRAME(struct __esf, stack_ptr);
	pInitCtx = Z_STACK_PTR_TO_FRAME(z_arch_esf_t, stack_ptr);

    /* Emulate the stack frame of an interrupt call */

    /* Set SR to enable all interrupts */
    pInitCtx->srl_ipl3_pch = 0;
    pInitCtx->pcl_sfa = (uint16_t)entry;

	pInitCtx->w0 = (uint16_t)entry;
	pInitCtx->w1 = (uint16_t)p1;
	pInitCtx->w2 = (uint16_t)p2;
	pInitCtx->w3 = (uint16_t)p3;

    pInitCtx->tblpag = TBLPAG;
    pInitCtx->dsrpag = DSRPAG;
    pInitCtx->dswpag = DSWPAG;

    /* Set the SPLIM register to point to the end of the stack */
	pInitCtx->splim = 0U;

    /* Set SR to enable all interrupts */
	pInitCtx->sr = 0U;
	pInitCtx->corcon = CORCON;

	/*
	 * We are saving SP to pop out entry and parameters when going through
	 * z_pic30_exit_exc()
	 */
	thread->callee_saved.w15 = (uint16_t)pInitCtx;

	thread->switch_handle = thread;
}

void *z_arch_get_next_switch_handle(struct k_thread **old_thread)
{
	*old_thread =  _current;

	return z_get_next_switch_handle(*old_thread);
}
