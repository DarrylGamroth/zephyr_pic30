/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _MACRO_PRIV_INC_
#define _MACRO_PRIV_INC_

#include <toolchain.h>
#include <arch/cpu.h>

#ifdef _ASMLANGUAGE

.macro z_pic30_enter_exc
	push.d	w0
	push.d	w2
	push.d	w4
	push.d	w6
#if defined(CONFIG_PIC30_DSP)
	push	ACCAL
	push	ACCAH
	push	ACCAU
	push	ACCBL
	push	ACCBH
	push	ACCBU
#endif /* CONFIG_PIC30_DSP */
	push	TBLPAG
#if defined(CONFIG_PIC30_EDS)
	push	DSRPAG
	push	DSWPAG
#else
	push	PSVPAG
#endif /* CONFIG_PIC30_EDS */
	push	RCOUNT
	push	SPLIM
	push	SR
	push	CORCON
.endm

.macro z_pic30_exit_exc
	pop	CORCON
	pop	SR
	pop	SPLIM
	pop	RCOUNT
#if defined(CONFIG_PIC30_EDS)
	pop	DSWPAG
	pop	DSRPAG
#else
	pop	PSVPAG
#endif /* CONFIG_PIC30_EDS */
	pop	TBLPAG
#if defined(CONFIG_PIC30_DSP)
	pop	ACCBU
	pop	ACCBH
	pop	ACCBL
	pop	ACCAU
	pop	ACCAH
	pop	ACCAL
#endif /* CONFIG_PIC30_DSP */
	pop.d	w6
	pop.d	w4
	pop.d	w2
	pop.d	w0
.endm

#endif /* _ASMLANGUAGE */

#endif /* _MACRO_PRIV_INC_ */
