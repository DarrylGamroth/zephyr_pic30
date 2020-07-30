/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _MICROCHIP_DSPIC33F_SOC_H_
#define _MICROCHIP_DSPIC33F_SOC_H_

#ifndef _ASMLANGUAGE

#include <zephyr/types.h>

/* Add include for DTS generated information */
#include <devicetree.h>

#if 0
#if defined(CONFIG_SOC_PART_NUMBER_DSPIC33FJ256GP710A)
#include <p33FJ256GP710A.h>
#elif defined(CONFIG_SOC_PART_NUMBER_DSPIC33FJ128MC804)
#include <p33FJ128MC804.h>
#else
#error Library does not support the specified device.
#endif
#else
#include <xc.h>
#endif

#endif /* _ASMLANGUAGE */

#endif /* _MICROCHIP_DSPIC33F_SOC_H_ */
