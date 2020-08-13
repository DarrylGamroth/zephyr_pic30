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

void arch_new_thread(struct k_thread *thread, k_thread_stack_t *stack,
		     size_t stack_size, k_thread_entry_t thread_func,
		     void *arg1, void *arg2, void *arg3,
		     int priority, unsigned int options)
{
	char *stack_memory = Z_THREAD_STACK_BUFFER(stack);

	struct __esf *stack_init;

#ifdef CONFIG_PIC30_SOC_CONTEXT_SAVE
	const struct soc_esf soc_esf_init = {SOC_ESF_INIT};
#endif

	z_new_thread_init(thread, stack_memory, stack_size);

	/* Initial stack frame for thread */
	stack_init = (struct __esf *)
		     Z_STACK_PTR_ALIGN(stack_memory +
				       stack_size - sizeof(struct __esf));

	/* Setup the initial stack frame */
	stack_init->w0 = (uint16_t)thread_func;
	stack_init->w1 = (uint16_t)arg1;
	stack_init->w2 = (uint16_t)arg2;
	stack_init->w3 = (uint16_t)arg3;
	/*
	 * Following the RISC-V architecture,
	 * the MSTATUS register (used to globally enable/disable interrupt),
	 * as well as the MEPC register (used to by the core to save the
	 * value of the program counter at which an interrupt/exception occcurs)
	 * need to be saved on the stack, upon an interrupt/exception
	 * and restored prior to returning from the interrupt/exception.
	 * This shall allow to handle nested interrupts.
	 *
	 * Given that context switching is performed via a system call exception
	 * within the PIC30 architecture implementation, initially set:
	 * 1) MSTATUS to MSTATUS_DEF_RESTORE in the thread stack to enable
	 *    interrupts when the newly created thread will be scheduled;
	 * 2) MEPC to the address of the z_thread_entry_wrapper in the thread
	 *    stack.
	 * Hence, when going out of an interrupt/exception/context-switch,
	 * after scheduling the newly created thread:
	 * 1) interrupts will be enabled, as the MSTATUS register will be
	 *    restored following the MSTATUS value set within the thread stack;
	 * 2) the core will jump to z_thread_entry_wrapper, as the program
	 *    counter will be restored following the MEPC value set within the
	 *    thread stack.
	 */
#if 0
	stack_init->mstatus = MSTATUS_DEF_RESTORE;
	stack_init->mepc = (uint16_t)z_thread_entry_wrapper;

#ifdef CONFIG_PIC30_SOC_CONTEXT_SAVE
	stack_init->soc_context = soc_esf_init;
#endif

	thread->callee_saved.sp = (uint16_t)stack_init;
#endif
}

