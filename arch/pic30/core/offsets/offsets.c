/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PIC30 kernel structure member offset definition file
 *
 * This module is responsible for the generation of the absolute symbols whose
 * value represents the member offsets for various PIC30 kernel structures.
 *
 * All of the absolute symbols defined by this module will be present in the
 * final kernel ELF image (due to the linker's reference to the _OffsetAbsSyms
 * symbol).
 *
 * INTERNAL
 * It is NOT necessary to define the offset for every member of a structure.
 * Typically, only those members that are accessed by assembly language routines
 * are defined; however, it doesn't hurt to define all fields for the sake of
 * completeness.
 */

#include <kernel.h>
#include <kernel_arch_data.h>
#include <gen_offset.h>
#include <kernel_offsets.h>

#ifdef CONFIG_PIC30_SOC_CONTEXT_SAVE
#include <soc_context.h>
#endif
#ifdef CONFIG_PIC30_SOC_OFFSETS
#include <soc_offsets.h>
#endif

/* thread_arch_t member offsets */
GEN_OFFSET_SYM(_thread_t, switch_handle);

/* struct coop member offsets */
GEN_OFFSET_SYM(_callee_saved_t, w8);
GEN_OFFSET_SYM(_callee_saved_t, w9);
GEN_OFFSET_SYM(_callee_saved_t, w10);
GEN_OFFSET_SYM(_callee_saved_t, w11);
GEN_OFFSET_SYM(_callee_saved_t, w12);
GEN_OFFSET_SYM(_callee_saved_t, w13);
GEN_OFFSET_SYM(_callee_saved_t, w14);

/* esf member offsets */
GEN_OFFSET_SYM(z_arch_esf_t, w0);
GEN_OFFSET_SYM(z_arch_esf_t, w1);
GEN_OFFSET_SYM(z_arch_esf_t, w2);
GEN_OFFSET_SYM(z_arch_esf_t, w3);
GEN_OFFSET_SYM(z_arch_esf_t, w4);
GEN_OFFSET_SYM(z_arch_esf_t, w5);
GEN_OFFSET_SYM(z_arch_esf_t, w6);
GEN_OFFSET_SYM(z_arch_esf_t, w7);

#if defined(CONFIG_PIC30_SOC_CONTEXT_SAVE)
GEN_OFFSET_SYM(z_arch_esf_t, soc_context);
#endif
#if defined(CONFIG_PIC30_SOC_OFFSETS)
GEN_SOC_OFFSET_SYMS();
#endif

GEN_ABSOLUTE_SYM(__z_arch_esf_t_SIZEOF, STACK_ROUND_UP(sizeof(z_arch_esf_t)));

/*
 * size of the struct k_thread structure sans save area for floating
 * point registers.
 */
GEN_ABSOLUTE_SYM(_K_THREAD_NO_FLOAT_SIZEOF, sizeof(struct k_thread));

GEN_ABS_SYM_END
