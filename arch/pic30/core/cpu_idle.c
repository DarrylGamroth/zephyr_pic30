/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <tracing/tracing.h>
#include <irq.h>

void __weak arch_cpu_idle(void)
{
    sys_trace_idle();
    __builtin_pwrsav(1);
}

void __weak arch_cpu_atomic_idle(unsigned int key)
{
	irq_unlock(key);
    sys_trace_idle();
    __builtin_pwrsav(1);
}
