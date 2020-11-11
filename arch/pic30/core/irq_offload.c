/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <irq.h>
#include <irq_offload.h>

static irq_offload_routine_t _offload_routine;
static const void *offload_param;

void z_irq_do_offload(void)
{
    offload_routine(offload_param);
}

void arch_irq_offload(irq_offload_routine_t routine, const void *parameter)
{
	unsigned int key;

	key = irq_lock();
	_offload_routine = routine;
	offload_param = parameter;

	__asm__ volatile ("bset INTCON2,#13");

	irq_unlock(key);
}
