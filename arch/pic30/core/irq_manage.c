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

void z_pic30_fatal_error(unsigned int reason, const z_arch_esf_t *esf);

void arch_irq_enable(unsigned int irq)
{
    /* FIXME IEC0 */
    uint16_t reg = (uint16_t)0x820U + irq / 16;
    uint16_t val = 1U << (irq % 16);


    sys_write16(sys_read16(reg) | val, reg);
}

void arch_irq_disable(unsigned int irq)
{
    /* FIXME IEC0 */
    uint16_t reg = (uint16_t)0x820U + irq / 16;
    uint16_t val = 1U << (irq % 16);


    sys_write16(sys_read16(reg) & ~val, reg);
}

int arch_irq_is_enabled(unsigned int irq)
{
    /* FIXME IEC0 */
    uint16_t reg = (uint16_t)0x820U + irq / 16;
    uint16_t val = 1U << (irq % 16);


    return !!(sys_read16(reg) & val);
}

void z_pic30_irq_priority_set(unsigned int irq, unsigned int prio, uint32_t flags)
{
    ARG_UNUSED(flags);

	__ASSERT(prio < CONFIG_NUM_IRQ_PRIO_LEVELS,
		 "invalid priority %d for irq %d", prio, irq);

    /* FIXME IPC0 */
    uint16_t reg = (uint16_t)0x840U + irq / 4;
    uint16_t offset = ((irq % 4) * 4);
    uint16_t mask = 0x7 << offset;
    uint16_t val = (prio & 0x7) << offset;

    sys_write16((sys_read16(reg) & ~mask) | val, reg);
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
