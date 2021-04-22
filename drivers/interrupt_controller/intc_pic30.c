/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <arch/cpu.h>
#include <drivers/interrupt_controller/pic30-intc.h>

void pic30_intc_irq_enable(unsigned int irq)
{
	uint16_t int_grp = irq / 16;
	uint16_t int_off = irq % 16;

	uint16_t reg = PIC30_IEC_BASE + (int_grp * 2);
	uint16_t val = 1U << int_off;

	sys_write16(sys_read16(reg) | val, reg);
}

void pic30_intc_irq_disable(unsigned int irq)
{
	uint16_t int_grp = irq / 16;
	uint16_t int_off = irq % 16;

	uint16_t reg = PIC30_IEC_BASE + (int_grp * 2);
	uint16_t val = 1U << int_off;

	sys_write16(sys_read16(reg) & ~val, reg);
}

int pic30_intc_irq_is_enabled(unsigned int irq)
{
	uint16_t int_grp = irq / 16;
	uint16_t int_off = irq % 16;

	uint16_t reg = PIC30_IEC_BASE + (int_grp * 2);
	uint16_t val = 1U << int_off;

	return !!(sys_read16(reg) & val);
}

void pic30_intc_irq_pending_set(unsigned int irq)
{
	uint16_t int_grp = irq / 16;
	uint16_t int_off = irq % 16;

	uint16_t reg = PIC30_IFS_BASE + (int_grp * 2);
	uint16_t val = 1U << int_off;

	sys_write16(sys_read16(reg) | val, reg);
}

int pic30_intc_irq_is_pending(unsigned int irq)
{
	uint16_t int_grp = irq / 16;
	uint16_t int_off = irq % 16;

	uint16_t reg = PIC30_IFS_BASE + (int_grp * 2);
	uint16_t val = 1U << int_off;

	return !!(sys_read16(reg) & val);
}

void pic30_intc_irq_set_priority(unsigned int irq, unsigned int prio)
{
	uint16_t int_grp = irq / 4;
	uint16_t int_off = irq % 4;

	uint16_t reg = PIC30_IPC_BASE + (int_grp * 2);
	uint16_t offset = int_off * 4;
	uint16_t mask = (CONFIG_NUM_IRQ_PRIO_LEVELS - 1) << offset;
	uint16_t val = (prio & (CONFIG_NUM_IRQ_PRIO_LEVELS - 1)) << offset;

	sys_write16((sys_read16(reg) & ~mask) | val, reg);
}

/**
 *
 * @brief Initialize the intc device driver
 *
 *
 * @return N/A
 */
int pic30_intc_init(const struct device *unused)
{
	ARG_UNUSED(unused);

	int irq = 0;

	for (; irq < CONFIG_NUM_IRQS; irq++) {
		pic30_intc_irq_set_priority(irq, _EXC_IRQ_DEFAULT_PRIO);
	}

	return 0;
}

SYS_INIT(pic30_intc_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
