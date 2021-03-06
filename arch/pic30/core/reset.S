;;
;; C Run-time startup module for MPLAB XC16 C compiler.
;; (c) Copyright 2016 Microchip Technology, All rights reserved
;;
;; Primary version, with data initialization support.
;; The linker loads this version when the --data-init
;; option is selected.
;;
;; See file crt1.s for the alternate version without
;; data initialization support.
;;
;; Entry __reset takes control at device reset and
;; performs the following:
;;
;;  1. initialize stack and stack limit register
;;  2. initialize PSV window if __const_length > 0
;;  3. process the data initialization template
;;  4. call the .user_init section, if it exists
;;  5. call the user's _main entry point
;;
;; Assigned to section .init, which may be allocated
;; at a specific address in linker scripts.
;;
;; If a local copy of this file is customized, be sure
;; to choose a file name other than crt0.s or crt1.s.
;;

#include <toolchain.h>
#include <linker/sections.h>
#include <arch/cpu.h>
#include "vector_table.h"
#include "macro_priv.inc"

/* exports */
GTEXT(__reset)
GTEXT(__start)

/* imports */
GTEXT(_z_pic30_prep_c)
GTEXT(_z_interrupt_stacks)
GTEXT(__data_init)

;;
;; Initialize stack, PSV, and data
;;
;; registers used:  w0
;;
;; Inputs (defined by user or linker):
;;  __SP_init
;;  __SPLIM_init
;;
;; Outputs:
;;  (does not return - resets the processor)
;;
;; Calls:
;;  __psv_init
;;  __data_init
;;  _z_pic30_prep_c
;;

SECTION_SUBSEC_FUNC(reset,_reset_section,__reset)
SECTION_SUBSEC_FUNC(reset,_reset_section,__start)

	mov	#__SP_init, w15		; compiler initialized stack pointer

#if defined(CONFIG_PIC30_DSP)
	mov	#__enable_fixed, w0
	cp0	w0
	bra	z, CORCON_RESET
	mov	#0x0010,w0
	mov	w0,CORCON		; enable super saturation and clear IF

CORCON_RESET:
#endif /* CONFIG_PIC30_DSP */

	rcall	__psv_init		; initialize PSV

	mov	#__dinit_tbloffset, w0	; w0,w1 = template
	mov	#__dinit_tblpage, w1
	ior	w0,w1,[w15]
	bra	eq, 1f			; skip if no init table

	rcall	__data_init		; initialize data

1:
#ifdef CONFIG_INIT_STACKS
	/* Pre-populate all bytes in z_interrupt_stacks with 0xAA */
	mov	#_z_interrupt_stacks, w15
	mov	#0xaaaa, w14
	repeat	#CONFIG_ISR_STACK_SIZE/2
	mov	w14, [w15++]
#endif

	mov	#_z_interrupt_stacks, w15	; initialize stack pointer

	mov	#CONFIG_ISR_STACK_SIZE/2-16, w14	; leave some space for traps
	add	w14, w15, w14
	mov	w14, SPLIM		; initialize SPLIM
	nop				; wait 1 cycle

	/*
	 * Jump into C domain. _PrepC zeroes BSS
	 * and then enters kernel z_cstart
	 */
	goto	_z_pic30_prep_c

;;
;; Initialize PSV window if __const_length > 0
;;
;; Registers used:  w0
;;
;; Inputs (defined by linker):
;;  __const_length
;;  __const_psvpage
;;
;; Outputs:
;;  (none)
;;
SECTION_SUBSEC_FUNC(reset,_reset_section,__psv_init)
#ifdef CONFIG_PIC30_EDS
	mov	#__const_length, w0
	cp0	w0			; test length of constants
	bra	z, 1f			; br if zero

	mov	#__const_psvpage, w0
	mov	w0, DSRPAG		; DSRPAG = psvpage(constants)
#else
	bclr	CORCON, #PSV		; disable PSV (default)
	mov	#__const_length, w0
	cp0	w0			; test length of constants
	bra	z, 1f			; br if zero

	mov	#__const_psvpage, w0
	mov	w0, PSVPAG		; PSVPAG = psvpage(constants)
	bset	CORCON, #PSV		; enable PSV (default)
#endif

1:
	return				;  and exit

