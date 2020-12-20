/*
 * Copyright (c) Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

SYMBOL(_vector_start) = .;
KEEP(*(.exc_vector_table))
KEEP(*(".exc_vector_table.*"))
KEEP(*(_IRQ_VECTOR_TABLE_SECTION_NAME))
SYMBOL(_vector_end) = .;
