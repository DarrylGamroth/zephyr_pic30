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
	uint16_t lpstart0;					\
	uint16_t lpend0;						\
	uint16_t lpcount0;					\
	uint16_t lpstart1;					\
	uint16_t lpend1;						\
	uint16_t lpcount1

/* Initial saved state. */
#define SOC_ESF_INIT						\
	0,						\
	0,						\
	0,						\
	0,						\
	0,						\
	0

#endif /* CONFIG_SOC_PIC30_DSPIC33C */

#endif /* SOC_PIC30_DSPIC33C_SOC_CONTEXT_H_ */
