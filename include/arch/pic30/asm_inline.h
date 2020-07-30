/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_PIC30_ASM_INLINE_H_
#define ZEPHYR_INCLUDE_ARCH_PIC30_ASM_INLINE_H_

/*
 * The file must not be included directly
 * Include kernel.h instead
 */

#if defined(__GNUC__)
#include <arch/pic30/asm_inline_gcc.h>
#else
#error "Supports only GNU C compiler"
#endif

#endif /* ZEPHYR_INCLUDE_ARCH_PIC30_ASM_INLINE_H_ */
