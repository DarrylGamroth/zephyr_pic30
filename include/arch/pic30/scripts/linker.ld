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

MEMORY
    {
    ROMABLE_REGION	(rx)   : ORIGIN = ROM_ADDR, LENGTH = ROM_SIZE
    RAMABLE_REGION	(a!xr) : ORIGIN = RAM_ADDR, LENGTH = RAM_SIZE
    /* Used by and documented in include/linker/intlist.ld */
    IDT_LIST  (wx)      : ORIGIN = (RAM_ADDR + RAM_SIZE), LENGTH = 2K
    }

ENTRY(CONFIG_KERNEL_ENTRY)

SECTIONS
    {

    GROUP_START(ROMABLE_REGION)

	_image_rom_start = ROM_ADDR;

    SECTION_PROLOGUE(rom_start,,)
	{

/* Located in generated directory. This file is populated by calling
 * zephyr_linker_sources(ROM_START ...). This typically contains the vector
 * table and debug information.
 */
#include <snippets-rom-start.ld>

	} GROUP_LINK_IN(ROMABLE_REGION)

    SECTION_PROLOGUE(_TEXT_SECTION_NAME,,)
	{
	_image_text_start = .;

#include <linker/kobject-text.ld>
  /*
  ** Note that input sections *(.text) are not mapped here.
  ** The best-fit allocator locates them, so that .text
  ** may flow around PSV sections as needed.
  */
/*
	*(.text)
	*(".text.*")
*/
	*(.handle);
	*(.gnu.linkonce.t.*)

	} GROUP_LINK_IN(ROMABLE_REGION)

	_image_text_end = .;

	_image_rodata_start = .;

#include <linker/common-rom.ld>

    SECTION_PROLOGUE(_RODATA_SECTION_NAME,,)
	{
	*(.rodata)
	*(".rodata.*")
	*(.gnu.linkonce.r.*)

/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
#include <snippets-rodata.ld>

#include <linker/kobject-rom.ld>

	/*
	 * For XIP images, in order to avoid the situation when __data_rom_start
	 * is 16-bit aligned, but the actual data is placed right after rodata
	 * section, which may not end exactly at 16-bit border, pad rodata
	 * section, so __data_rom_start points at data and it is 16-bit aligned.
	 *
	 * On non-XIP images this may enlarge image size up to 1 bytes. This
	 * generally is not an issue, since modern ROM and FLASH memory is
	 * usually 4k aligned.
	 */
	. = ALIGN(2);
	} GROUP_LINK_IN(ROMABLE_REGION)

#include <linker/cplusplus-rom.ld>

	_image_rodata_end = .;
	MPU_ALIGN(_image_rodata_end -_image_rom_start);
	_image_rom_end = .;

    GROUP_END(ROMABLE_REGION)

    GROUP_START(RAMABLE_REGION)

	. = RAM_ADDR;
	/* Align the start of image SRAM with the
	 * minimum granularity required by MPU.
	 */
	. = ALIGN(_region_min_align);
	_image_ram_start = .;

/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
#include <snippets-ram-sections.ld>

    SECTION_DATA_PROLOGUE(_DATA_SECTION_NAME,,)
	{
	__data_ram_start = .;
	*(.data)
	*(".data.*")
	*(".kernel.*")

/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
#include <snippets-rwdata.ld>

	} GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

    __data_rom_start = LOADADDR(_DATA_SECTION_NAME);

#include <linker/common-ram.ld>
#include <linker/kobject.ld>

#include <linker/cplusplus-ram.ld>

    __data_ram_end = .;

   SECTION_DATA_PROLOGUE(_BSS_SECTION_NAME,(NOLOAD),)
	{
        . = ALIGN(2);
	__bss_start = .;
	___bss_start = .;
	__kernel_ram_start = .;

	*(.bss)
	*(".bss.*")
	*(COMMON)
	*(".kernel_bss.*")

	__bss_end = ALIGN(2);
	} GROUP_DATA_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)

    SECTION_PROLOGUE(_NOINIT_SECTION_NAME,(NOLOAD),)
        {
        /*
         * This section is used for non-initialized objects that
         * will not be cleared during the boot process.
         */
        *(.noinit)
        *(".noinit.*")
	*(".kernel_noinit.*")

/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
#include <snippets-noinit.ld>

        } GROUP_LINK_IN(RAMABLE_REGION)

    /* Define linker symbols */

    _image_ram_end = .;
    _end = .; /* end of image */

    __kernel_ram_end = RAM_ADDR + RAM_SIZE;
    __kernel_ram_size = __kernel_ram_end - __kernel_ram_start;

    GROUP_END(RAMABLE_REGION)

/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
#include <snippets-sections.ld>

#include <linker/debug-sections.ld>

    /DISCARD/ : { *(.note.GNU-stack) }


/* Must be last in romable region */
SECTION_PROLOGUE(.last_section,(NOLOAD),)
{
} GROUP_LINK_IN(ROMABLE_REGION)

/* To provide the image size as a const expression,
 * calculate this value here. */
_flash_used = LOADADDR(.last_section) - _image_rom_start;

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

 INTCON2      = 0x8C2;
_INTCON2      = 0x8C2;
