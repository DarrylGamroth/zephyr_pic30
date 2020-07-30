/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief New thread creation for Microchip PIC30
 *
 * Core thread related primitives for the Microchip PIC30
 * processor architecture.
 */

#include <kernel.h>
#include <ksched.h>
#include <wait_q.h>

/* An initial context, to be "restored" by z_arm_pendsv(), is put at the other
 * end of the stack, and thus reusable by the stack when not needed anymore.
 *
 * The initial context is an exception stack frame (ESF) since exiting the
 * PendSV exception will want to pop an ESF. Interestingly, even if the lsb of
 * an instruction address to jump to must always be set since the CPU always
 * runs in thumb mode, the ESF expects the real address of the instruction,
 * with the lsb *not* set (instructions are always aligned on 16 bit
 * halfwords).  Since the compiler automatically sets the lsb of function
 * addresses, we have to unset it manually before storing it in the 'pc' field
 * of the ESF.
 */
void arch_new_thread(struct k_thread *thread, k_thread_stack_t *stack,
		     size_t stackSize, k_thread_entry_t pEntry,
		     void *parameter1, void *parameter2, void *parameter3,
		     int priority, unsigned int options)
{
	char *pStackMem = Z_THREAD_STACK_BUFFER(stack);
	char *stackEnd;
	/* Offset between the top of stack and the high end of stack area. */
	uint32_t top_of_stack_offset = 0U;

	stackEnd = pStackMem + stackSize;

	struct __esf *pInitCtx;

	z_new_thread_init(thread, pStackMem, stackSize);

	/* Carve the thread entry struct from the "base" of the stack
	 *
	 * The initial carved stack frame only needs to contain the basic
	 * stack frame (state context).
	 */
	pInitCtx = (struct __esf *)(Z_STACK_PTR_ALIGN(stackEnd -
		(char *)top_of_stack_offset - sizeof(struct __basic_sf)));

	pInitCtx->basic.pc = (uint32_t)z_thread_entry;

	pInitCtx->basic.a1 = (uint32_t)pEntry;
	pInitCtx->basic.a2 = (uint32_t)parameter1;
	pInitCtx->basic.a3 = (uint32_t)parameter2;
	pInitCtx->basic.a4 = (uint32_t)parameter3;

	thread->callee_saved.psp = (uint32_t)pInitCtx;

	thread->arch.basepri = 0;

	/* swap_return_value can contain garbage */

	/*
	 * initial values in all other registers/thread entries are
	 * irrelevant.
	 */
}

#if defined(CONFIG_BUILTIN_STACK_GUARD)
/*
 * @brief Configure ARM built-in stack guard
 *
 * This function configures per thread stack guards by reprogramming
 * the built-in Process Stack Pointer Limit Register (PSPLIM).
 * The functionality is meant to be used during context switch.
 *
 * @param thread thread info data structure.
 */
void configure_builtin_stack_guard(struct k_thread *thread)
{
	uint32_t guard_start = thread->stack_info.start;
#if defined(CONFIG_CPU_CORTEX_M_HAS_SPLIM)
	__set_PSPLIM(guard_start);
#else
#error "Built-in PSP limit checks not supported by HW"
#endif
}
#endif /* CONFIG_BUILTIN_STACK_GUARD */

void arch_switch_to_main_thread(struct k_thread *main_thread,
				k_thread_stack_t *main_stack,
				size_t main_stack_size,
				k_thread_entry_t _main)
{

	/* get high address of the stack, i.e. its start (stack grows down) */
	char *start_of_main_stack;

	start_of_main_stack =
		Z_THREAD_STACK_BUFFER(main_stack) + main_stack_size;

	start_of_main_stack = (char *)Z_STACK_PTR_ALIGN(start_of_main_stack);

	_current = main_thread;
#ifdef CONFIG_TRACING
	sys_trace_thread_switched_in();
#endif

	/* the ready queue cache already contains the main thread */

#if defined(CONFIG_BUILTIN_STACK_GUARD)
	/* Set PSPLIM register for built-in stack guarding of main thread. */
#if defined(CONFIG_CPU_CORTEX_M_HAS_SPLIM)
	__set_PSPLIM((uint32_t)main_stack);
#else
#error "Built-in PSP limit checks not supported by HW"
#endif
#endif /* CONFIG_BUILTIN_STACK_GUARD */

	/*
	 * Set PSP to the highest address of the main stack
	 * before enabling interrupts and jumping to main.
	 */
	__asm__ volatile (
	"mov   r0,  %0\n\t"	/* Store _main in R0 */
#if defined(CONFIG_CPU_CORTEX_M)
	"msr   PSP, %1\n\t"	/* __set_PSP(start_of_main_stack) */
#endif

	"movs r1, #0\n\t"
#if defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE) \
			|| defined(CONFIG_ARMV7_R)
	"cpsie i\n\t"		/* __enable_irq() */
#elif defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
	"cpsie if\n\t"		/* __enable_irq(); __enable_fault_irq() */
	"msr   BASEPRI, r1\n\t"	/* __set_BASEPRI(0) */
#else
#error Unknown ARM architecture
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */
	"isb\n\t"
	"movs r2, #0\n\t"
	"movs r3, #0\n\t"
	"bl z_thread_entry\n\t"	/* z_thread_entry(_main, 0, 0, 0); */
	:
	: "r" (_main), "r" (start_of_main_stack)
	: "r0" /* not to be overwritten by msr PSP, %1 */
	);

	CODE_UNREACHABLE;
}
