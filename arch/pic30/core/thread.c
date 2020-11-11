/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <ksched.h>

void z_thread_entry_wrapper(k_thread_entry_t thread,
			    void *arg1,
			    void *arg2,
			    void *arg3);

struct init_stack_frame {
    uint16_t w0;
};

void arch_new_thread(struct k_thread *thread, k_thread_stack_t *stack,
		     char *stack_ptr, k_thread_entry_t entry,
		     void *arg1, void *arg2, void *arg3)
{
	struct init_stack_frame *iframe;

	struct __esf *stack_init;

	/* Initial stack frame data, stored at the base of the stack */
	iframe = Z_STACK_PTR_TO_FRAME(struct init_stack_frame, stack_ptr);

#ifdef CONFIG_PIC30_SOC_CONTEXT_SAVE
	const struct soc_esf soc_esf_init = {SOC_ESF_INIT};
#endif
#if 0
	/* Setup the initial stack frame */
	iframe->w0 = (uint16_t)arg1;
	iframe->w1 = (uint16_t)arg2;
	iframe->w2 = (uint16_t)arg3;
#endif

#if 0
#ifdef CONFIG_PIC30_SOC_CONTEXT_SAVE
	stack_init->soc_context = soc_esf_init;
#endif

	thread->callee_saved.sp = (uint16_t)stack_init;
#endif
    thread->switch_handle = thread;
}

