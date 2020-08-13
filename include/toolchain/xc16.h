/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_TOOLCHAIN_XC16_H_
#define ZEPHYR_INCLUDE_TOOLCHAIN_XC16_H_

/* BYTE_ORDER IS LITTLE ENDIAN */
#define __BYTE_ORDER__ (2)

#include <toolchain/gcc.h>

#undef __in_section_unique
#define __in_section_unique(seg) ___in_section(seg, __LINE__, __COUNTER__)

#endif
