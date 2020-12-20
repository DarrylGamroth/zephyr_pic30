/*
 * Copyright (c) Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * ICD Debug Exec
 *
 * This section provides optional storage for
 * the ICD2 debugger. Define a global symbol
 * named __ICD2RAM to enable ICD2. This section
 * must be loaded at the start of RAM.
 */
SECTION_PROLOGUE(.icd, (NOLOAD),)
{
    . += (DEFINED (__ICD2RAM) ? 0x50 : 0 );
} GROUP_LINK_IN(RAMABLE_REGION)
