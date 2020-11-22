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

#undef popcount

#if __GNUC__ >= 5
#undef HAS_BUILTIN___builtin_add_overflow
#undef HAS_BUILTIN___builtin_sub_overflow
#undef HAS_BUILTIN___builtin_mul_overflow
#undef HAS_BUILTIN___builtin_div_overflow
#endif
#if __GNUC__ >= 4
#undef HAS_BUILTIN___builtin_clz
#undef HAS_BUILTIN___builtin_clzl
#undef HAS_BUILTIN___builtin_clzll
#undef HAS_BUILTIN___builtin_ctz
#undef HAS_BUILTIN___builtin_ctzl
#undef HAS_BUILTIN___builtin_ctzll
#endif


#if defined(_ASMLANGUAGE)

#undef SECTION_VAR
#undef SECTION_FUNC
#undef SECTION_SUBSEC_FUNC

/*
 * Need to use assembly macros because ';' is interpreted as the start of
 * a single line comment in the ARC assembler.
 *
 * Also, '\()' is needed in the .section directive of these macros for
 * correct substitution of the 'section' variable.
 */

.macro section_var section, symbol
	.section .\section\().\symbol
	\symbol :
.endm

.macro section_func section, symbol
	.section .\section\().\symbol, code;
	FUNC_CODE()
	\symbol :
	FUNC_INSTR(\symbol)
.endm

.macro section_subsec_func section, subsection, symbol
	.section .\section\().\subsection, code
	PERFOPT_ALIGN
	\symbol :
.endm

#define SECTION_VAR(sect, sym) section_var sect, sym
#define SECTION_FUNC(sect, sym) section_func sect, sym
#define SECTION_SUBSEC_FUNC(sect, subsec, sym) \
	section_subsec_func sect, subsec, sym

#endif /* _ASMLANGUAGE */

#undef __in_section_unique
#define __in_section_unique(seg) ___in_section(seg, __LINE__, __COUNTER__)

#endif
