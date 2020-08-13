/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Extra definitions required for CONFIG_PIC30_SOC_CONTEXT_SAVE.
 */

#ifndef SOC_PIC30_DSPIC33C_SOC_CONTEXT_H_
#define SOC_PIC30_DSPIC33C_SOC_CONTEXT_H_

#ifdef CONFIG_SOC_PIC30_DSPIC33C

/* Extra state for dsPIC33C registers. */
#define SOC_ESF_MEMBERS					\
    uint16_t tblpag;    \
    uint16_t accal;     \
    uint16_t accah;     \
    uint16_t accau;     \
    uint16_t accbl;     \
    uint16_t accbh;     \
    uint16_t accbu;     \
    uint16_t dcount;    \
    uint16_t dostartl;  \
    uint16_t dostarth;  \
    uint16_t doendl;    \
    uint16_t doendh;    \
    uint16_t dsrpag;    \
    uint16_t dswpag;

/* Initial saved state. */
#define SOC_ESF_INIT						\
	0,						\
	0,						\
	0,						\
	0,						\
	0,						\
	0,						\
	0,						\
	0,						\
	0,						\
	0,						\
	0,						\
	0,						\
	0,						\
	0

#endif /* CONFIG_SOC_PIC30_DSPIC33C */

#endif /* SOC_PIC30_DSPIC33C_SOC_CONTEXT_H_ */
