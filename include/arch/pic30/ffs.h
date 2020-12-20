/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_PIC30_FFS_H_
#define ZEPHYR_INCLUDE_ARCH_PIC30_FFS_H_

#ifndef _ASMLANGUAGE

#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *
 * @brief find most significant bit set in a 32-bit word
 *
 * This routine finds the first bit set starting from the most significant bit
 * in the argument passed in and returns the index of that bit.  Bits are
 * numbered starting at 1 from the least significant bit.  A return value of
 * zero indicates that the value passed is zero.
 *
 * @return most significant bit set, 0 if @a op is 0
 */

static ALWAYS_INLINE unsigned int find_msb_set(uint32_t op)
{
	uint16_t op_lsw = (uint16_t)op;
	uint16_t op_msw = (uint16_t)(op >> 16UL);

	if (op == 0) {
		return 0;
	} else if (op_msw) {
		return 32U - (__builtin_ff1l(op_msw) - 1);
	} else {
		return 16U - (__builtin_ff1l(op_lsw) - 1);
	}
}


/**
 *
 * @brief find least significant bit set in a 32-bit word
 *
 * This routine finds the first bit set starting from the least significant bit
 * in the argument passed in and returns the index of that bit.  Bits are
 * numbered starting at 1 from the least significant bit.  A return value of
 * zero indicates that the value passed is zero.
 *
 * @return least significant bit set, 0 if @a op is 0
 */

static ALWAYS_INLINE unsigned int find_lsb_set(uint32_t op)
{
	uint16_t op_lsw = (uint16_t)op;
	uint16_t op_msw = (uint16_t)(op >> 16UL);

	if (op == 0) {
		return 0;
	} else if (op_lsw) {
		return __builtin_ff1r(op_lsw);
	} else {
		return 16U + __builtin_ff1r(op_msw);
	}
}

#ifdef __cplusplus
}
#endif

#endif  /* _ASMLANGUAGE */

#endif  /* ZEPHYR_INCLUDE_ARCH_PIC30_FFS_H_ */
