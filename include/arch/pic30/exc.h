/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PIC30 public exception handling
 *
 * PIC30 kernel exception handling interface. Included by
 * pic30/arch.h.
 */

#ifndef ZEPHYR_INCLUDE_ARCH_PIC30_EXC_H_
#define ZEPHYR_INCLUDE_ARCH_PIC30_EXC_H_

#include <devicetree.h>

//#include <arch/pic30/aarch32/cortex_m/nvic.h>

/* for assembler, only works with constants */
#define Z_EXC_PRIO(pri) (((pri) << (8 - NUM_IRQ_PRIO_BITS)) & 0xff)

/*
 * In architecture variants with non-programmable fault exceptions
 * (e.g. Cortex-M Baseline variants), hardware ensures processor faults
 * are given the highest interrupt priority level. SVCalls are assigned
 * the highest configurable priority level (level 0); note, however, that
 * this interrupt level may be shared with HW interrupts.
 *
 * In Cortex variants with programmable fault exception priorities we
 * assign the highest interrupt priority level (level 0) to processor faults
 * with configurable priority.
 * The highest priority level may be shared with either Zero-Latency IRQs (if
 * support for the feature is enabled) or with SVCall priority level.
 * Regular HW IRQs are always assigned priority levels lower than the priority
 * levels for SVCalls, Zero-Latency IRQs and processor faults.
 *
 * PendSV IRQ (which is used in Cortex-M variants to implement thread
 * context-switching) is assigned the lowest IRQ priority level.
 */
#if defined(CONFIG_CPU_CORTEX_M_HAS_PROGRAMMABLE_FAULT_PRIOS)
#define _EXCEPTION_RESERVED_PRIO 1
#else
#define _EXCEPTION_RESERVED_PRIO 0
#endif

#define _EXC_FAULT_PRIO 0
#ifdef CONFIG_ZERO_LATENCY_IRQS
#define _EXC_ZERO_LATENCY_IRQS_PRIO 0
#define _EXC_SVC_PRIO 1
#define _IRQ_PRIO_OFFSET (_EXCEPTION_RESERVED_PRIO + 1)
#else
#define _EXC_SVC_PRIO 0
#define _IRQ_PRIO_OFFSET (_EXCEPTION_RESERVED_PRIO)
#endif

#define _EXC_IRQ_DEFAULT_PRIO Z_EXC_PRIO(_IRQ_PRIO_OFFSET)

/* Use lowest possible priority level for PendSV */
#define _EXC_PENDSV_PRIO 0xff
#define _EXC_PENDSV_PRIO_MASK Z_EXC_PRIO(_EXC_PENDSV_PRIO)

#ifdef _ASMLANGUAGE
GTEXT(z_pic30_exc_exit);
#else
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct __esf {
    uint16_t w0;
    uint16_t w1;
    uint16_t w2;
    uint16_t w3;
    uint16_t w4;
    uint16_t w5;
    uint16_t w6;
    uint16_t w7;
    uint16_t tblpag;
    uint16_t accal;
    uint16_t accah;
    uint16_t accau;
    uint16_t accbl;
    uint16_t accbh;
    uint16_t accbu;
    uint16_t dcount;
    uint16_t dostartl;
    uint16_t dostarth;
    uint16_t doendl;
    uint16_t doendh;
    uint16_t corcon;
#if defined( __dsPIC30F__ ) || defined( __dsPIC33F__ )
    uint16_t psvpag;
#elif defined ( __dsPIC33E__ ) || defined ( __dsPIC33C__ )
    uint16_t dsrpag;
    uint16_t dswpag;
#endif
};

typedef struct __esf z_arch_esf_t;

extern void z_pic30_exc_exit(void);

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_PIC30_EXC_H_ */
