/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Populated vector table
 */

#include <toolchain.h>
#include <linker/sections.h>

/* imports */
GTEXT(__reset)
GTEXT(__isr_wrapper)
GTEXT(_z_pic30_oscillator_fail)
GTEXT(_z_pic30_address_error)
GTEXT(_z_pic30_hard_trap_error)
GTEXT(_z_pic30_stack_error)
GTEXT(_z_pic30_math_error)
GTEXT(_z_pic30_soft_trap_error)

SECTION_FUNC(exc_vector_table, __interrupt_vector_table)
	goto	__reset
	.pword paddr(_z_pic30_oscillator_fail)
	.pword paddr(_z_pic30_address_error)
	.pword paddr(_z_pic30_hard_trap_error)
	.pword paddr(_z_pic30_stack_error)
	.pword paddr(_z_pic30_math_error)
	.pword 0
	.pword paddr(_z_pic30_soft_trap_error)
	.pword 0
	/* End of system exception */

	.rept CONFIG_NUM_IRQS
	.pword paddr(__isr_wrapper)
	.endr

/* exports */
GDATA(__interrupt_vector_table)

SECTION_FUNC(TEXT, _z_pic30_oscillator_fail)
	bra .
SECTION_FUNC(TEXT, _z_pic30_address_error)
	bra .
SECTION_FUNC(TEXT, _z_pic30_hard_trap_error)
	bra .
SECTION_FUNC(TEXT, _z_pic30_stack_error)
	bra .
SECTION_FUNC(TEXT, _z_pic30_math_error)
	bra .
SECTION_FUNC(TEXT, _z_pic30_soft_trap_error)
	bra .
