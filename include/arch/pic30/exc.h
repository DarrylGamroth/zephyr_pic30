/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PIC30 public exception handling
 *
 * PIC30-specific kernel exception handling interface.
 */

#ifndef ZEPHYR_INCLUDE_ARCH_PIC30_EXC_H_
#define ZEPHYR_INCLUDE_ARCH_PIC30_EXC_H_

#ifdef CONFIG_ZERO_LATENCY_IRQS
#define _EXC_ZERO_LATENCY_IRQS_PRIO	7U
#endif

/* Deafult priority disables interrupts */
#define _EXC_IRQ_DEFAULT_PRIO		0U

#ifndef _ASMLANGUAGE
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct __esf {
	uint16_t pcl_sfa;
	uint16_t srl_ipl3_pch;
	uint16_t w0;
	uint16_t w1;
	uint16_t w2;
	uint16_t w3;
	uint16_t w4;
	uint16_t w5;
	uint16_t w6;
	uint16_t w7;
#if defined(CONFIG_PIC30_DSP)
	uint16_t accal;
	uint16_t accah;
	uint16_t accau;
	uint16_t accbl;
	uint16_t accbh;
	uint16_t accbu;
#endif /* CONFIG_PIC30_DSP */
	uint16_t tblpag;
#if defined(CONFIG_PIC30_EDS)
	uint16_t dsrpag;
	uint16_t dswpag;
#else
	uint16_t psvpag;
#endif /* CONFIG_PIC30_EDS */
	uint16_t rcount;
	uint16_t splim;
	uint16_t sr;
	uint16_t corcon;
};

typedef struct __esf z_arch_esf_t;

#ifdef __cplusplus
}
#endif

#endif  /* _ASMLANGUAGE */

#endif  /* ZEPHYR_INCLUDE_ARCH_PIC30_EXC_H_ */
