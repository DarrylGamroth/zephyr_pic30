/*
 * Copyright (c) 2019 BayLibre SAS
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Basic macro definitions that gcc and clang provide on their own
 * but that xc16 lacks. Only those that Zephyr requires are provided here.
 */

#ifndef ZEPHYR_INCLUDE_TOOLCHAIN_XC16_MISSING_DEFS_H_
#define ZEPHYR_INCLUDE_TOOLCHAIN_XC16_MISSING_DEFS_H_

/*
 * The following defines are inferred from the xc16 provided defines
 * already tested above.
 */

#define __INT8_MAX__ 0x7f
#define __INT8_TYPE__ signed char

#define __INT16_MAX__ 0x7fff
#define __INT16_TYPE__ short int

#define __INT32_MAX__ 0x7fffffff
#define __INT32_TYPE__ long

#define __INT64_MAX__ 0x7fffffffffffffffLL
#define __INT64_TYPE__ long long int

#define __INT_FAST8_MAX__ 0x7f
#define __INT_FAST8_TYPE__ signed char
#define __INT_FAST8_WIDTH__ 8

#define __INT_FAST16_MAX__ 0x7fffffff
#define __INT_FAST16_TYPE__ int
#define __INT_FAST16_WIDTH__ 32

#define __INT_FAST32_MAX__ 0x7fffffff
#define __INT_FAST32_TYPE__ long
#define __INT_FAST32_WIDTH__ 32

#define __INT_FAST64_MAX__ 0x7fffffffffffffffLL
#define __INT_FAST64_TYPE__ long long int
#define __INT_FAST64_WIDTH__ 64

#define __INT_LEAST8_MAX__ 0x7f
#define __INT_LEAST8_TYPE__ signed char
#define __INT_LEAST8_WIDTH__ 8

#define __INT_LEAST16_MAX__ 0x7fff
#define __INT_LEAST16_TYPE__ short int
#define __INT_LEAST16_WIDTH__ 16

#define __INT_LEAST32_MAX__ 0x7fffffff
#define __INT_LEAST32_TYPE__ long
#define __INT_LEAST32_WIDTH__ 32

#define __INT_LEAST64_MAX__ 0x7fffffffffffffffLL
#define __INT_LEAST64_TYPE__ long long int
#define __INT_LEAST64_WIDTH__ 64

#define __UINT8_MAX__ 0xff
#define __UINT8_TYPE__ unsigned char

#define __UINT16_MAX__ 0xffff
#define __UINT16_TYPE__ short unsigned int

#define __UINT32_MAX__ 0xffffffffU
#define __UINT32_TYPE__ unsigned long

#define __UINT64_MAX__ 0xffffffffffffffffULL
#define __UINT64_TYPE__ long long unsigned int

#define __UINT_FAST8_MAX__ 0xff
#define __UINT_FAST8_TYPE__ unsigned char

#define __UINT_FAST16_MAX__ 0xffffffffU
#define __UINT_FAST16_TYPE__ unsigned int

#define __UINT_FAST32_MAX__ 0xffffffffU
#define __UINT_FAST32_TYPE__ unsigned long

#define __UINT_FAST64_MAX__ 0xffffffffffffffffULL
#define __UINT_FAST64_TYPE__ long long unsigned int

#define __UINT_LEAST8_MAX__ 0xff
#define __UINT_LEAST8_TYPE__ unsigned char

#define __UINT_LEAST16_MAX__ 0xffff
#define __UINT_LEAST16_TYPE__ short unsigned int

#define __UINT_LEAST32_MAX__ 0xffffffffU
#define __UINT_LEAST32_TYPE__ unsigned long

#define __UINT_LEAST64_MAX__ 0xffffffffffffffffULL
#define __UINT_LEAST64_TYPE__ long long unsigned int

#endif
