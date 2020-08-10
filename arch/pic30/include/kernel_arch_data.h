/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Private kernel definitions (PIC30)
 *
 * This file contains private kernel structures definitions and various
 * other definitions for the PIC30 architecture.
 *
 * This file is also included by assembly language files which must #define
 * _ASMLANGUAGE before including this header file.  Note that kernel
 * assembly source files obtains structure offset values via "absolute symbols"
 * in the offsets.o module.
 */

#ifndef ZEPHYR_ARCH_PIC30_INCLUDE_KERNEL_ARCH_DATA_H_
#define ZEPHYR_ARCH_PIC30_INCLUDE_KERNEL_ARCH_DATA_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _ASMLANGUAGE

/**
 * @brief Sparc Standard Stack Frame
 *
 * From The PIC30 Architecture Manual Version 8
 *
 * The following are always allocated at compile time in every
 * procedure’s stack frame:
 *
 * - 16 words, always starting at %sp, for saving the procedure’s in
 *   and local registers, should a register window overflow occur
 *
 * The following are allocated at compile time in the stack frames of
 * non-leaf procedures:
 *
 * - One word, for passing a “hidden” (implicit) parameter. This is
 *   used when the caller is expecting the callee to return a data
 *   aggregate by value; the hidden word contains the address of stack
 *   space allocated (if any) by the caller for that purpose See
 *   Section D.4.
 *
 * - Six words, into which the callee may store parameters that must
 *   be addressable
 *
 */
struct _standard_stack_frame {
	uint16_t w0;
	uint16_t w1;
	uint16_t w2;
	uint16_t w3;
	uint16_t w4;
	uint16_t w5;
	uint16_t w6;
	uint16_t w7;
};

typedef struct _standard_stack_frame _standard_stack_frame_t;

#endif /* _ASMLANGUAGE */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_ARCH_PIC30_INCLUDE_KERNEL_ARCH_DATA_H_ */
