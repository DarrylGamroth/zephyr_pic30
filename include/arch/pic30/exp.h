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

#ifndef ZEPHYR_INCLUDE_ARCH_PIC30_EXP_H_
#define ZEPHYR_INCLUDE_ARCH_PIC30_EXP_H_

#ifndef _ASMLANGUAGE
#include <zephyr/types.h>
#include <toolchain.h>

#ifdef CONFIG_PIC30_SOC_CONTEXT_SAVE
#include <soc_context.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The name of the structure which contains soc-specific state, if
 * any, as well as the soc_esf_t typedef below, are part of the PIC30
 * arch API.
 *
 * The contents of the struct are provided by a SOC-specific
 * definition in soc_context.h.
 */
#ifdef CONFIG_PIC30_SOC_CONTEXT_SAVE
struct soc_esf {
	SOC_ESF_MEMBERS;
};
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

#ifdef CONFIG_PIC30_SOC_CONTEXT_SAVE
	struct soc_esf soc_context;
#endif
};

typedef struct __esf z_arch_esf_t;
#ifdef CONFIG_PIC30_SOC_CONTEXT_SAVE
typedef struct soc_esf soc_esf_t;
#endif

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_PIC30_EXP_H_ */
