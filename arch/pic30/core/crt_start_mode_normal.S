;; C Run-time startup module for MPLAB XC16 C compiler.
;; (c) Copyright 2016 Microchip Technology, All rights reserved

;
;   Default __crt_start_mode function
;
;	modify to your taste
;
;   This function returns 0 to indicate that a full startup is required
;

	.section .libpic30_crt_start_mode,code

	.weak __crt_start_mode_normal
	.weak __crt_start_mode

__crt_start_mode:
__crt_start_mode_normal:
        retlw #0,w0

