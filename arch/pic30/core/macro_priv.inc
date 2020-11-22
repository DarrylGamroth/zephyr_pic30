/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _MACRO_PRIV_INC_
#define _MACRO_PRIV_INC_

#ifdef _ASMLANGUAGE

.macro z_pic30_enter_exc
	push.d	w0
	push.d	w2
	push.d	w4
	push.d	w6
	push	ACCAL
	push	ACCAH
	push	ACCAU
	push	ACCBL
	push	ACCBH
	push	ACCBU
	push	TBLPAG
	push	DSRPAG
	push	DSWPAG
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
	pop	DSWPAG
	pop	DSRPAG
	pop	TBLPAG
	pop	ACCBU
	pop	ACCBH
	pop	ACCBL
	pop	ACCAU
	pop	ACCAH
	pop	ACCAL
	pop.d	w6
	pop.d	w4
	pop.d	w2
	pop.d	w0
.endm

#endif /* _ASMLANGUAGE */

#endif /* _MACRO_PRIV_INC_ */
