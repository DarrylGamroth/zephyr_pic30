/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Linker command/script file
 *
 * Linker script for the Microchip PIC30 platform.
 */

#include <autoconf.h>
#include <linker/sections.h>
#include <devicetree.h>

#include <linker/linker-defs.h>
#include <linker/linker-tool.h>

OUTPUT_ARCH("33CK256MP508")
CRT0_STARTUP(reset.o)
CRT1_STARTUP(reset.o)
CRT_STARTMODE(crt_start_mode_normal)

/* physical address of RAM */
#define ROMABLE_REGION program
#define RAMABLE_REGION data

#define _DATA_IN_ROM __data_rom_start

#define ROM_SIZE (CONFIG_FLASH_SIZE * 1K)
#define ROM_ADDR (CONFIG_FLASH_BASE_ADDRESS)

#define RAM_SIZE (CONFIG_SRAM_SIZE * 1K)
#define RAM_ADDR CONFIG_SRAM_BASE_ADDRESS

#if defined(CONFIG_CUSTOM_SECTION_ALIGN)
_region_min_align = CONFIG_CUSTOM_SECTION_MIN_ALIGN_SIZE;
#else
/* Use default 2-byte alignment. */
_region_min_align = 2;
#endif

#define MPU_ALIGN(region_size) \
    . = ALIGN(_region_min_align)

__CODE_BASE = 0x200;

MEMORY
    {
    ROMABLE_REGION	(rx)   : ORIGIN = ROM_ADDR, LENGTH = ROM_SIZE
    RAMABLE_REGION	(a!xr) : ORIGIN = RAM_ADDR, LENGTH = RAM_SIZE
    /* Used by and documented in include/linker/intlist.ld */
    IDT_LIST  (wx)      : ORIGIN = 0xfff7ff, LENGTH = 2K
    }

ENTRY(CONFIG_KERNEL_ENTRY)

SECTIONS
    {

    GROUP_START(ROMABLE_REGION)

    SYMBOL(_image_rom_start) = ROM_ADDR;

    SECTION_PROLOGUE(_TEXT_SECTION_NAME,,)
    {
        SYMBOL(_image_text_start) = .;

        #include <snippets-rom-start.ld>

	. = __CODE_BASE;

	/* Don't specify text sections to allow the best-fit allocator
	 * flow around PSV sections.
	 */
        *(.reset)
        *(".reset.*")
        KEEP(*(.handle))
        *(.gnu.linkonce.t.*)
	/* keep together in this order */
        *(.libm)
        *(".libm.*")
	*(.libdsp)
	*(".libdsp.*")
	*(.libpic30);
	*(".libpic30.*");

    } GROUP_LINK_IN(ROMABLE_REGION)

    SYMBOL(_image_text_end) = .;
    SYMBOL(_image_text_size) = SYMBOL(_image_text_end) -
        SYMBOL(_image_text_start);


    SYMBOL(_image_rodata_start) = .;

    /* Replace common-rom.ld with snippet */
    #include <linker/common-rom.ld>

    SECTION_PROLOGUE(.const,,)
    {
        *(.const);
        *(".const.*");
    } GROUP_LINK_IN(ROMABLE_REGION)


    SECTION_RODATA_PROLOGUE(_RODATA_SECTION_NAME,,)
    {
	*(.rodata)
	*(".rodata.*")
	*(.gnu.linkonce.r.*)
/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
        #include <snippets-rodata.ld>

    } GROUP_LINK_IN(ROMABLE_REGION)

    SYMBOL(_image_rodata_end) = .;
    SYMBOL(_image_rodata_size) = SYMBOL(_image_rodata_end) -
        SYMBOL(_image_rodata_start);
    SYMBOL(_image_rom_end) = .;

    GROUP_END(ROMABLE_REGION)

    GROUP_START(RAMABLE_REGION)

    SYMBOL(_image_ram_start) = .;

/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
    #include <snippets-ram-sections.ld>

    SYMBOL(__kernel_ram_start) = .;

    SECTION_DATA_PROLOGUE(nbss,(NOLOAD),)
    {
        . = ALIGN(2);
	*(.nbss)
	*(".nbss.*")
	*(.bss._kernel)
    } GROUP_LINK_IN(RAMABLE_REGION)

    SECTION_DATA_PROLOGUE(ndata,,)
    {
        . = ALIGN(2);
        *(.ndata)
        *(".ndata.*")
    } GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

#if 0
    SECTION_DATA_PROLOGUE(_BSS_SECTION_NAME,(NOLOAD),)
    {
        . = ALIGN(2);
	SYMBOL(__bss_start) = .;
	*(.bss)
	*(".bss.*")
	COMMON_SYMBOLS
	*(".kernel_bss.*")

        /*
         * As memory is cleared in words only, it is simpler to ensure the BSS
         * section ends on a 2 byte boundary. This wastes a maximum of 1 bytes.
         */
	SYMBOL(__bss_end) = ALIGN(2);
    } GROUP_LINK_IN(RAMABLE_REGION)
#endif

    #include <linker/common-noinit.ld>

    SECTION_DATA_PROLOGUE(_DATA_SECTION_NAME,,)
    {
        SYMBOL(__data_ram_start) = .;
        *(.data)
        *(".data.*")
        *(".kernel.*")

/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
        #include <snippets-rwdata.ld>

    } GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

    SYMBOL(__data_rom_start) = LOADADDR(_DATA_SECTION_NAME);

    #include <linker/common-ram.ld>

    SYMBOL(__data_ram_end) = .;

    /* Define linker symbols */
    SYMBOL(_image_ram_end) = .;
    SYMBOL(_end) = .; /* end of image */

    SYMBOL(__kernel_ram_end) = RAM_ADDR + RAM_SIZE;
    SYMBOL(__kernel_ram_size) = SYMBOL(__kernel_ram_end) - SYMBOL(__kernel_ram_start);

    GROUP_END(RAMABLE_REGION)

/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
#include <snippets-sections.ld>

#include <linker/debug-sections.ld>

/DISCARD/ : {
	*(.note.GNU-stack)
	*(.got.plt)
	*(.igot.plt)
	*(.got)
	*(.igot)
}

    /* Must be last in romable region */
    SECTION_PROLOGUE(.last_section,(NOLOAD),)
    {
    } GROUP_LINK_IN(ROMABLE_REGION)

    /* To provide the image size as a const expression,
     * calculate this value here. */
    SYMBOL(_flash_used) = LOADADDR(.last_section) - SYMBOL(_image_rom_start);

    }

 SPLIM        = 0x20;
_SPLIM        = 0x20;
 ACCAL        = 0x22;
_ACCAL        = 0x22;
 ACCAH        = 0x24;
_ACCAH        = 0x24;
 ACCAU        = 0x26;
_ACCAU        = 0x26;
 ACCBL        = 0x28;
_ACCBL        = 0x28;
 ACCBH        = 0x2A;
_ACCBH        = 0x2A;
 ACCBU        = 0x2C;
_ACCBU        = 0x2C;
 PCL          = 0x2E;
_PCL          = 0x2E;
 PCH          = 0x30;
_PCH          = 0x30;
 DSRPAG       = 0x32;
_DSRPAG       = 0x32;
_DSRPAGbits   = 0x32;
 DSWPAG       = 0x34;
_DSWPAG       = 0x34;
_DSWPAGbits   = 0x34;
 RCOUNT       = 0x36;
_RCOUNT       = 0x36;
 DCOUNT       = 0x38;
_DCOUNT       = 0x38;
 DOSTARTL     = 0x3A;
_DOSTARTL     = 0x3A;
 DOSTARTH     = 0x3C;
_DOSTARTH     = 0x3C;
 DOENDL       = 0x3E;
_DOENDL       = 0x3E;
 DOENDH       = 0x40;
_DOENDH       = 0x40;
 SR           = 0x42;
_SR           = 0x42;
_SRbits       = 0x42;
 CORCON       = 0x44;
_CORCON       = 0x44;
_CORCONbits   = 0x44;
 MODCON       = 0x46;
_MODCON       = 0x46;
_MODCONbits   = 0x46;
 XMODSRT      = 0x48;
_XMODSRT      = 0x48;
 XMODEND      = 0x4A;
_XMODEND      = 0x4A;
 YMODSRT      = 0x4C;
_YMODSRT      = 0x4C;
 YMODEND      = 0x4E;
_YMODEND      = 0x4E;
 XBREV        = 0x50;
_XBREV        = 0x50;
_XBREVbits    = 0x50;
 DISICNT      = 0x52;
_DISICNT      = 0x52;
 TBLPAG       = 0x54;
_TBLPAG       = 0x54;
_TBLPAGbits   = 0x54;
 YPAG         = 0x56;
_YPAG         = 0x56;
_YPAGbits     = 0x56;
 MSTRPR       = 0x58;
_MSTRPR       = 0x58;
_MSTRPRbits   = 0x58;
 CTXTSTAT     = 0x5A;
_CTXTSTAT     = 0x5A;
_CTXTSTATbits = 0x5A;
 DMTCON       = 0x5C;
_DMTCON       = 0x5C;
_DMTCONbits   = 0x5C;
 DMTPRECLR    = 0x60;
_DMTPRECLR    = 0x60;
_DMTPRECLRbits = 0x60;
 DMTCLR       = 0x64;
_DMTCLR       = 0x64;
_DMTCLRbits   = 0x64;
 DMTSTAT      = 0x68;
_DMTSTAT      = 0x68;
_DMTSTATbits  = 0x68;
 DMTCNTL      = 0x6C;
_DMTCNTL      = 0x6C;
 DMTCNTH      = 0x6E;
_DMTCNTH      = 0x6E;
 DMTHOLDREG   = 0x70;
_DMTHOLDREG   = 0x70;
 DMTPSCNTL    = 0x74;
_DMTPSCNTL    = 0x74;
 DMTPSCNTH    = 0x76;
_DMTPSCNTH    = 0x76;
 DMTPSINTVL   = 0x78;
_DMTPSINTVL   = 0x78;
 DMTPSINTVH   = 0x7A;
_DMTPSINTVH   = 0x7A;

 IFS0         = 0x800;
_IFS0         = 0x800;
_IFS0bits     = 0x800;
 IEC0         = 0x820;
_IEC0         = 0x820;
_IEC0bits     = 0x820;
 IPC0         = 0x840;
_IPC0         = 0x840;
_IPC0bits     = 0x840;
 INTCON1      = 0x8C0;
_INTCON1      = 0x8C0;
_INTCON1bits  = 0x8C0;
 INTCON2      = 0x8C2;
_INTCON2      = 0x8C2;
_INTCON2bits  = 0x8C2;
 INTCON3      = 0x8C4;
_INTCON3      = 0x8C4;
_INTCON3bits  = 0x8C4;
 INTCON4      = 0x8C6;
_INTCON4      = 0x8C6;
_INTCON4bits  = 0x8C6;
 INTTREG      = 0x8C8;
_INTTREG      = 0x8C8;
_INTTREGbits  = 0x8C8;
