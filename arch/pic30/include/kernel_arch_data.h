/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Private kernel definitions
 *
 * This file contains private kernel structures definitions and various
 * other definitions for the PIC30 processor architecture.
 */

#ifndef ZEPHYR_ARCH_PIC30_INCLUDE_KERNEL_ARCH_DATA_H_
#define ZEPHYR_ARCH_PIC30_INCLUDE_KERNEL_ARCH_DATA_H_

#include <toolchain.h>
#include <linker/sections.h>
#include <arch/cpu.h>

#ifndef _ASMLANGUAGE
#include <kernel.h>
#include <zephyr/types.h>
#include <sys/util.h>
#include <sys/dlist.h>

#ifdef __cplusplus
extern "C" {
#endif

extern K_THREAD_STACK_DEFINE(_interrupt_stack, CONFIG_ISR_STACK_SIZE);

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_ARCH_PIC30_INCLUDE_KERNEL_ARCH_DATA_H_ */
