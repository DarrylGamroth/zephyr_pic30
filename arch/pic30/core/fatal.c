/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <kernel_structs.h>
#include <inttypes.h>
#include <logging/log.h>
LOG_MODULE_DECLARE(os);

FUNC_NORETURN void z_pic30_fatal_error(unsigned int reason,
				       const z_arch_esf_t *esf)
{
	if (esf != NULL) {
	}

	z_fatal_error(reason, esf);
	CODE_UNREACHABLE;
}

FUNC_NORETURN void _Fault(const z_arch_esf_t *esf)
{
	z_pic30_fatal_error(K_ERR_CPU_EXCEPTION, esf);
}
