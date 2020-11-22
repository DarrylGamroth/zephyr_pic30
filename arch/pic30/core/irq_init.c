/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PIC30 interrupt initialisation
 */

#include <arch/cpu.h>

/**
 *
 * @brief Initialize interrupts
 *
 * Ensures all interrupts have their priority set to _EXC_IRQ_DEFAULT_PRIO and
 * not 4, which they have it set to when coming out of reset. This ensures that
 * interrupt locking via SR IPL works as expected.
 *
 * @return N/A
 */

void z_pic30_interrupt_init(void)
{
	int irq = 0;

	for (; irq < CONFIG_NUM_IRQS; irq++) {
#ifdef FIXME
#warning "Initialize interrupt default priorities"
		NVIC_SetPriority((IRQn_Type)irq, _IRQ_PRIO_OFFSET);
#endif
	}
}
