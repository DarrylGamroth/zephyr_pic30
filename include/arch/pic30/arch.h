/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PIC30 specific kernel interface header
 * This header contains the PIC30 specific kernel interface.  It is
 * included by the generic kernel interface header (arch/cpu.h)
 */

#ifndef ZEPHYR_INCLUDE_ARCH_PIC30_ARCH_H_
#define ZEPHYR_INCLUDE_ARCH_PIC30_ARCH_H_

/* Add include for DTS generated information */
#include <devicetree.h>

#include <arch/pic30/thread.h>
#include <arch/pic30/exc.h>
#include <arch/pic30/irq.h>
#include <arch/pic30/misc.h>
#include <arch/pic30/asm_inline.h>
#include <arch/pic30/cpu.h>
#include <arch/pic30/ffs.h>
#include <arch/common/sys_bitops.h>
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
#define ARCH_STACK_PTR_ALIGN  2

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ARCH_PIC30_ARCH_H_ */
