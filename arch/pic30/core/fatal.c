/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Kernel fatal error handler for PIC30
 *
 * This module provides the z_pic30_fatal_error() routine for PIC30
 * CPUs
 */

#include <kernel.h>
#include <logging/log.h>

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

FUNC_NORETURN void z_pic30_fatal_error(unsigned int reason,
		const z_arch_esf_t *esf)
{
	if (esf != NULL) {
#if 0
		esf_dump(esf);
#endif
	}

	z_fatal_error(reason, esf);
	CODE_UNREACHABLE;
}
