/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PIC30 interrupt management
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <tracing/tracing.h>
#include <irq.h>
#include <toolchain.h>
#include <linker/sections.h>
#include <sw_isr_table.h>
#include <drivers/interrupt_controller/pic30-intc.h>

void z_pic30_fatal_error(unsigned int reason, const z_arch_esf_t *esf);

void arch_irq_enable(unsigned int irq)
{
	pic30_intc_irq_enable(irq);
}

void arch_irq_disable(unsigned int irq)
{
	pic30_intc_irq_disable(irq);
}

int arch_irq_is_enabled(unsigned int irq)
{
	return pic30_intc_irq_is_enabled(irq);
}

void z_pic30_isr_demux(void)
{
	struct _isr_table_entry *ite;
	unsigned int irq;

	/* Get the actual interrupt source from the interrupt controller */
	irq = pic30_intc_get_active();

	pic30_intc_irq_pending_clear(irq);

	ite = &_sw_isr_table[irq];
	ite->isr(ite->arg);
}

void z_pic30_irq_priority_set(unsigned int irq, unsigned int prio,
		uint32_t flags)
{
#if defined(CONFIG_ZERO_LATENCY_IRQS)
	/* If we have zero latency interrupts, those interrupts will
	 * run at a priority level which is not masked by irq_lock().
	 * Our policy is to express priority levels with special properties
	 * via flags
	 */
	if (flags & IRQ_ZERO_LATENCY) {
		prio = _EXC_ZERO_LATENCY_IRQS_PRIO;
    }
#else
	ARG_UNUSED(flags);
#endif

	__ASSERT(prio < CONFIG_NUM_IRQ_PRIO_LEVELS,
		 "invalid priority %d for irq %d", prio, irq);

	pic30_intc_irq_set_priority(irq, prio);
}

#ifdef CONFIG_DYNAMIC_INTERRUPTS
int arch_irq_connect_dynamic(unsigned int irq, unsigned int priority,
			     void (*routine)(const void *parameter),
			     const void *parameter, uint32_t flags)
{
	z_isr_install(irq, routine, parameter);
	z_pic30_irq_priority_set(irq, priority, flags);
	return irq;
}
#endif

void z_irq_spurious(const void *unused)
{
	ARG_UNUSED(unused);

	z_pic30_fatal_error(K_ERR_SPURIOUS_IRQ, NULL);
}
