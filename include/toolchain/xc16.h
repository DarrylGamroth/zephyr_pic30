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

#undef SECTION_VAR
#undef SECTION_FUNC
#undef SECTION_SUBSEC_FUNC

#define SECTION_VAR(sect, sym)  .section .sect.##sym; sym :
#define SECTION_FUNC(sect, sym)						\
	.section .sect.sym,code;					\
				PERFOPT_ALIGN; sym :		\
							FUNC_INSTR(sym)
#define SECTION_SUBSEC_FUNC(sect, subsec, sym)				\
		.section .sect.subsec,code; PERFOPT_ALIGN; sym :

#undef __in_section_unique
#define __in_section_unique(seg) ___in_section(seg, __LINE__, __COUNTER__)

#endif
