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
 * other definitions for the Microchip PIC30 processor architecture family.
 *
 * This file is also included by assembly language files which must #define
 * _ASMLANGUAGE before including this header file.  Note that kernel
 * assembly source files obtains structure offset values via "absolute symbols"
 * in the offsets.o module.
 */

#ifndef ZEPHYR_ARCH_PIC30_INCLUDE_KERNEL_ARCH_DATA_H_
#define ZEPHYR_ARCH_PIC30_INCLUDE_KERNEL_ARCH_DATA_H_

#include <toolchain.h>
#include <linker/sections.h>
#include <arch/cpu.h>
#include <exc.h>

#ifndef _ASMLANGUAGE
#include <kernel.h>
#include <zephyr/types.h>
#include <sys/dlist.h>
#include <sys/atomic.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct __esf _esf_t;
typedef struct __basic_sf _basic_sf_t;

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_ARCH_PIC30_INCLUDE_KERNEL_ARCH_DATA_H_ */
