/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Software interrupts utility code - PIC30 implementation
 */

#include <kernel.h>
#include <irq_offload.h>
#include <aarch64/exc.h>

volatile irq_offload_routine_t offload_routine;
static const void *offload_param;

void z_irq_do_offload(void)
{
	offload_routine(offload_param);
}

void arch_irq_offload(irq_offload_routine_t routine, const void *parameter)
{
	k_sched_lock();
	offload_routine = routine;
	offload_param = parameter;

	z_pic30_offload();

	offload_routine = NULL;
	k_sched_unlock();
}
