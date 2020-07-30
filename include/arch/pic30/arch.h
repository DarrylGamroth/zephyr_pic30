/*
 * Copyright (c) 2013-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PIC30 specific kernel interface header
 *
 * This header contains the PIC30 specific kernel interface.  It is
 * included by the kernel interface architecture-abstraction header
 * (include/pic30/cpu.h)
 */

#ifndef ZEPHYR_INCLUDE_ARCH_PIC30_ARCH_H_
#define ZEPHYR_INCLUDE_ARCH_PIC30_ARCH_H_

/* Add include for DTS generated information */
#include <devicetree.h>

#include <arch/pic30/thread.h>
#include <arch/pic30/exc.h>
#include <arch/pic30/irq.h>
#include <arch/pic30/error.h>
#include <arch/pic30/misc.h>
#include <arch/common/addr_types.h>
#include <arch/common/ffs.h>
#include <arch/pic30/nmi.h>
#include <arch/pic30/asm_inline.h>

#include <arch/pic30/cortex_m/cpu.h>
#include <arch/pic30/cortex_m/memory_map.h>
#include <arch/common/sys_io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Declare the ARCH_STACK_PTR_ALIGN
 *
 * Denotes the required alignment of the stack pointer on public API
 * boundaries
 *
 */
#define ARCH_STACK_PTR_ALIGN 2

/**
 * @brief Declare the minimum alignment for a thread stack
 *
 * Denotes the minimum required alignment of a thread stack.
 *
 */
#define Z_THREAD_MIN_STACK_ALIGN ARCH_STACK_PTR_ALIGN


/**
 * @brief Define alignment of a stack buffer
 *
 * This is used for two different things:
 *
 * -# Used in checks for stack size to be a multiple of the stack buffer
 *    alignment
 * -# Used to determine the alignment of a stack buffer
 *
 */
#define STACK_ALIGN MAX(Z_THREAD_MIN_STACK_ALIGN, Z_MPU_GUARD_ALIGN)

/**
 * @brief Calculate power of two ceiling for a buffer size input
 *
 */
#define POW2_CEIL(x) (1U << (15U + __builtin_fbcl(x)))

#define ARCH_THREAD_STACK_DEFINE(sym, size) \
	struct z_thread_stack_element __noinit __aligned(STACK_ALIGN) \
		sym[size]

#define ARCH_THREAD_STACK_LEN(size) ((size))

#define ARCH_THREAD_STACK_ARRAY_DEFINE(sym, nmemb, size) \
	struct z_thread_stack_element __noinit \
		__aligned(STACK_ALIGN) \
		sym[nmemb][ARCH_THREAD_STACK_LEN(size)]

#define ARCH_THREAD_STACK_MEMBER(sym, size) \
	struct z_thread_stack_element __aligned(STACK_ALIGN) \
		sym[size]

#define ARCH_THREAD_STACK_SIZEOF(sym) (sizeof(sym) )

#define ARCH_THREAD_STACK_BUFFER(sym) \
		((char *)(sym) )

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ARCH_PIC30_ARCH_H_ */
