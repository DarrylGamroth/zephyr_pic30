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

GEN_ABSOLUTE_SYM(___esf_t_SIZEOF, sizeof(_esf_t));

GEN_OFFSET_SYM(_thread_t, switch_handle);

GEN_OFFSET_SYM(_standard_stack_frame_t, w0);
GEN_OFFSET_SYM(_standard_stack_frame_t, w1);
GEN_OFFSET_SYM(_standard_stack_frame_t, w2);
GEN_OFFSET_SYM(_standard_stack_frame_t, w3);
GEN_OFFSET_SYM(_standard_stack_frame_t, w4);
GEN_OFFSET_SYM(_standard_stack_frame_t, w5);
GEN_OFFSET_SYM(_standard_stack_frame_t, w6);
GEN_OFFSET_SYM(_callee_saved_t, w8);
GEN_OFFSET_SYM(_callee_saved_t, w9);
GEN_OFFSET_SYM(_callee_saved_t, w10);
GEN_OFFSET_SYM(_callee_saved_t, w11);
GEN_OFFSET_SYM(_callee_saved_t, w12);
GEN_OFFSET_SYM(_callee_saved_t, w13);
GEN_OFFSET_SYM(_callee_saved_t, w14);

/* size of the entire preempt registers structure */

GEN_ABSOLUTE_SYM(___callee_saved_t_SIZEOF, sizeof(struct _callee_saved));

GEN_ABSOLUTE_SYM(__STD_STACK_FRAME_SIZEOF,
		 STACK_ROUND_UP(sizeof(_standard_stack_frame_t)));

#if defined(CONFIG_THREAD_STACK_INFO)
GEN_OFFSET_SYM(_thread_stack_info_t, start);

GEN_ABSOLUTE_SYM(___thread_stack_info_t_SIZEOF,
	 sizeof(struct _thread_stack_info));
#endif

/*
 * size of the struct k_thread structure sans save area for floating
 * point registers.
 */

GEN_ABSOLUTE_SYM(_K_THREAD_NO_FLOAT_SIZEOF, sizeof(struct k_thread));

GEN_ABS_SYM_END
