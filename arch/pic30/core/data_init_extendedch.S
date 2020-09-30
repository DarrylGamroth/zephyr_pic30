;;
;; C Run-time startup module for MPLAB XC16 C compiler.
;; (c) Copyright 2009 - 2016 Microchip Technology, All rights reserved
;;
;; Data initialization support.
;; The linker loads this version when the --data-init
;; option is selected.
;;
#include <toolchain.h>
#include <autoconf.h>
#include <arch/cpu.h>

/* exports */
GTEXT(__data_init_extended)

;; 
;; Process data init template
;;
;; The template is null-terminated, with records
;; in the following format:
;;
;; struct data_record {
;;  char *dst;        /* destination address  */
;;  int  len;         /* length in bytes      */
;;  int  format:7     /* format code          */
;;  int  page:9;      /* destination page     */
;;  char dat[0];      /* variable length data */
;; };
;; 
;; Registers used:  w0 w1 w2 w3 w4 w5
;; 
;; Inputs:
;;  w0 <- tbloffset of initialization template
;;  w1 <- tblpage of initialization template
;; 
;; Outputs:
;;  (none)
;;
;; Calls:
;;  __memcpypd3crt
;; 
;; for PIC24F devices with extended data memory
;;

        .equ __data_init, __data_init_extended
        .global __data_init

SECTION_FUNC(init, __data_init_extended)

        ; for all format codes, hi-bit set means there is a DST page at
        ; data[0]
        .equiv   FMT_CLEAR,0    ;  format codes
        .equiv   FMT_COPY2,1    ; 
        .equiv   FMT_COPY3,2    ; 
	.equiv   FMT_CALL, 3
        .equiv   FMT_DUO_COPY3,0x1F ;  copy len instructions to slave


#define ZERO      w0
#define TBLOFFSET w9
        ; DSTOFFSET, LEN must be paired for call.l
#define DSTOFFSET w10
#define LEN       w11
#define UPPERBYTE w12
#define FORMAT    w13
#define PAGE      w14

        mov       w1,_TBLPAG
	mov       w0,TBLOFFSET
        clr       ZERO
        bra      4f

1:      add      TBLOFFSET,#2,TBLOFFSET
        addc     _TBLPAG                         ; ZERO must be tied to 0

        tblrdl.w [TBLOFFSET],LEN
        add      TBLOFFSET,#2,TBLOFFSET
        addc     _TBLPAG                         ; ZERO must be tied to 0

        tblrdl.w [TBLOFFSET],FORMAT
        add      TBLOFFSET,#2,TBLOFFSET
        addc     _TBLPAG                         ; ZERO must be tied to 0

        clr      UPPERBYTE
        lsr      FORMAT,#7,PAGE                  ; PAGE <- stored DSWPAG
        and      #0x7F,FORMAT                    ; zero out upper bits
        mov      PAGE,_DSWPAG                    ; set DSWPAG

        cp.b     FORMAT,#FMT_CALL
        bra nz,  2f

        call.l   DSTOFFSET
        clr      ZERO                            ; calling a fn may clobber this
        bra      4f

2:      cp.b     FORMAT,#FMT_CLEAR
        bra nz,  2f

        ;; FMT_CLEAR - clear destination memory
9:      clr.b    [DSTOFFSET]
        inc      DSTOFFSET,DSTOFFSET
        bra nc,  8f
        inc      _DSWPAG
        bset     DSTOFFSET,#15
8:      dec      LEN,LEN
        bra gtu, 9b                              ; loop if not done 
        bra      4f

        ;; FMT_COPY2, FMT_COPY3
2:      cp       FORMAT,#FMT_COPY2
        bra z,   3f

        setm     UPPERBYTE

        ;; special memcpyd3
3:      rcall __memcpyd3extended

4:      tblrdh.b [TBLOFFSET],PAGE               ; check for link
        tblrdl.w [TBLOFFSET],DSTOFFSET
        cp0.b    PAGE
        bra nz,  1f
        cp0      DSTOFFSET
        bra nz,  1b
        return

1:      and      #0x7F,PAGE
        mov      PAGE, _TBLPAG                  ; link
        mov      DSTOFFSET, TBLOFFSET
        bra      4b

;;
;; Copy data from program memory to data memory using
;;  Inputs, ZERO and w5
;;
;; not user callable
;;
;; Inputs:
;;  TBLOFFSET:_TBLPAG source addres (24-bits)
;;  DSTOFFSET:_DSWPAG destination address (24-bits)
;;  LEN number of bytes (even or odd)
;;  UPPERBYTE upper byte flag   (0 = false)
;;
;; Outputs:
;;  TBLOFFSET:_TBLPAG  next source address (24 bits)
;;
;; This function does not follow the standard C calling convesions
;;

SECTION_FUNC(init, __memcpyd3extended)
1:      
        btst.c   UPPERBYTE, #13        ; set carry if UPPERBYTE is set      
        addc     DSTOFFSET, #2, [w15]  
        bra nc,  2f                    ; this will not cross

        ; the destination offset will cross a page, so check each time
        mov      TBLOFFSET,w5          ; w5 =   tbloffset(src)
        tblrdl.b [w5++],[DSTOFFSET]    ; dst = lo byte
        inc      DSTOFFSET,DSTOFFSET
        bra nc,  3f
	inc      _DSWPAG               ; increment write page
	bset     DSTOFFSET,#15         ; set bit 15
3:       
        dec      LEN,LEN               ; num--
        bra z,   4f                    ; br if done

        tblrdl.b [w5--],[DSTOFFSET]    ; dst++ = hi byte
        inc      DSTOFFSET,DSTOFFSET
        bra nc,  3f
        inc      _DSWPAG
        bset     DSTOFFSET,#15
3:
        dec      LEN,LEN               ; num--
        bra z,   4f                    ; br if done

        cp0      UPPERBYTE             ; test upper flag
        bra z,   4f                    ; br if false

        tblrdh.b [w5],[DSTOFFSET]      ; dst++ = upper byte
        inc      DSTOFFSET,DSTOFFSET
        bra nc,  3f
        inc      _DSWPAG
        bset     DSTOFFSET,#15
3:       
        dec      LEN,LEN               ; num--
4:      
        inc2     TBLOFFSET,TBLOFFSET
        addc     _TBLPAG               ; ZERO must be tied to 0
        cp0      LEN
        bra nz,   1b
        return                         ; exit

2:      ; the destination offset will not cross this time, be quicker
        ;   still need to write byte at a time in case there is a funky offset
        tblrdl  [TBLOFFSET],w5
        mov.b   w5,[DSTOFFSET++]
        dec     LEN,LEN
        bra z,  4b

        lsr     w5,#8,w5
        mov.b   w5,[DSTOFFSET++]
        dec     LEN,LEN
        bra z,  4b

        cp0     UPPERBYTE
        bra z,  4b

        tblrdh.b [TBLOFFSET],[DSTOFFSET++]
        bra     3b


