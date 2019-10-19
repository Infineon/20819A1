/* Linker script for ARM Simulator
 *
 * Version: Sourcery G++ Lite 2011.03-42
 * Support: https://support.codesourcery.com/GNUToolchain/
 *
 * Copyright (c) 2007, 2008, 2009, 2010 CodeSourcery, Inc.
 *
 * The authors hereby grant permission to use, copy, modify, distribute,
 * and license this software and its documentation for any purpose, provided
 * that existing copyright notices are retained in all copies and that this
 * notice is included verbatim in any distributions.  No written agreement,
 * license, or royalty fee is required for any of the authorized uses.
 * Modifications to this software may be copyrighted by their authors
 * and need not follow the licensing terms described here, provided that
 * the new terms are clearly indicated on the first page of each file where
 * they apply.
 */
/*
 * Linker script for SPAR in RAM application. Modified from generic.ld
 *   2011, 2012, 2013 Changes COPYRIGHT CYPRESS SEMICONDUCTOR.
 */

OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
SEARCH_DIR(.)
/* GROUP(-lgcc -lc) */

MEMORY
{
  ram (rwx) : ORIGIN = IRAM_BEGIN, LENGTH = IRAM_LEN
  aon (rwx) : ORIGIN = IAON_BEGIN, LENGTH = IAON_LEN
  static_section (r) : ORIGIN = ISTATIC_BEGIN, LENGTH = ISTATIC_LEN
  xip_section (r) : ORIGIN = IXIP_BEGIN, LENGTH = IXIP_LEN

#if ( IAPP_XIP_PI == 1 )
   // Stores the relocation information.  None of this goes into the flash.
   xip_relo (rwx)  : ORIGIN = 0xFF000000, LENGTH = 0x100000
#endif
}

/*
 * Extern the linker generated symbols
 */
EXTERN(spar_irom_begin spar_irom_end spar_irom_length);
EXTERN(spar_iram_begin spar_iram_end spar_iram_length);
EXTERN(spar_iram_data_begin spar_iram_data_end spar_iram_data_length);
EXTERN(spar_iram_bss_begin spar_iram_bss_end spar_iram_bss_length);
EXTERN(spar_irom_data_begin);
EXTERN(aon_iram_end);
EXTERN(xip_area_begin);
EXTERN(xip_area_end);

/*
 * Provide some symbols for spar init code to use
 */
PROVIDE(spar_irom_length = spar_irom_end - spar_irom_begin);
PROVIDE(spar_iram_length = spar_iram_end - spar_iram_begin);
PROVIDE(spar_iram_data_length = spar_iram_data_end - spar_iram_data_begin);
PROVIDE(spar_iram_bss_length = spar_iram_bss_end - spar_iram_bss_begin);

SECTIONS
{
  xip_area_begin = IXIP_BEGIN;

  #include "spar_ram_overlays.ld"

  /* Leaving this section as an example of how to pull out specific files into a section */
  .app_xip_area : ALIGN (4)
  {
    CREATE_OBJECT_SYMBOLS
    /* Using perl script XIP_OBJS symbol will be replaced by all app/profile objects list */
    XIP_OBJS(.text.* .gnu.linkonce.t.* .rodata .constdata* .rodata.* .gnu.linkonce.r.* )

    /* special section for Position Independent trampoline functions. */
    *(.pisec_*)

    KEEP(*(.data_in_xip_area))
    KEEP(*(.dsp_pram_section))

    // .text
    . = ALIGN(4);
    *(.app_init_code)
    *(.emb_text)
    *(.text)
    *(EmHeAacDec)
    *(.text.* .gnu.linkonce.t.*)
    *(i.*)
    *(.glue_7t) *(.glue_7) *(.vfp11_veneer) *(.v4_bx)

    // .setup
    . = ALIGN(4);
    *(.init_code)
    *(.init_text)
    *(.spar_setup)

    //  .ordered
    . = ALIGN(4);
    *(.ARM.extab*)
    *(.gnu.linkonce.armextab.*)
    *(.ARM.exidx*)
    *(.gnu.linkonce.armexidx.*)

    // .rodata
    . = ALIGN(4);
    *(const_drom_var)
    *(.rodata)
    *(.constdata*)
    *(.rodata.* .gnu.linkonce.r.*)
    *(.data.rom)

    //  .data_rel_ro
    . = ALIGN(4);

    // This handles the constants addresses used by other constant vars that the linker thinks might be moved...
    //   These are post processed to get ready for dynamic linking upon boot.
    *(.data.rel.ro)
    *(.data.rel.ro.local)

    xip_area_end = .;
  } >xip_section =0

#if ( IAPP_XIP_PI == 1 )

  // Got table and constant variables using addresses of constant data.
  .datagot : ALIGN (4)
  {
     xip_got_begin = .;
    *(.got)
    *(.got.plt)
    *(.plt)
     xip_got_end = .;
  } >xip_section =0


  // Store the relocation information here so we dont wast space in OCF
  // None of this info is needed in the device.
  .dyn_info : ALIGN (4)
  {
     *(.interp)
     *(.dynsym)
     *(.dynstr)
     *(.hash)
     *(.dynamic)

     // These are here for completeness.  We should never get this!
     //    They are for OS specific stuff.
     *(.iplt)
     *(.igot.plt)

     // These are relocations.  By putting them in this section, it hides them
     // from the "readelf" command, so they dont get printed. We dont want them.
     // They are relocations for the entire got table, that are not needed since
     // we put the got table into a section above and then stitch it in FW.
      *(.rel.got)
      *(.rel.plt)
  } >xip_relo

  // This must be a seperate section, otherwise linker puts it in the first section!!!
  // We dont need these in the device, they are pre-processed by a script and we
  // build a cgs record out of them.
  .rel.dyn : ALIGN (4)
  {
     *(.rel.dyn)
     *(.rel.init_patch_table)
  } >xip_relo
#endif

  spar_irom_begin = IRAM_BEGIN;
  spar_irom_end = .;

  .data : ALIGN (4)
  {
    CREATE_OBJECT_SYMBOLS
    spar_iram_begin = .;
    spar_iram_data_begin = .;
    *(.data)
    *(.data.* .gnu.linkonce.d.*)
    *(.init_patch_table)
    spar_iram_data_end = .;
  } >ram

  /* Nothing to load because it is loaded from EEPROM/SF/OCF at boot. */
  spar_irom_data_begin = spar_iram_data_begin;

  .bss : ALIGN (4)
  {
    CREATE_OBJECT_SYMBOLS
    spar_iram_bss_begin = .;
    *(.bss)
    *(.bss.*  .gnu.linkonce.b.*)
    *(COMMON)
    spar_iram_bss_end = .;
    spar_iram_end = .;
  } >ram


  .aon : ALIGN (4)
  {
    CREATE_OBJECT_SYMBOLS
    aon_iram_begin = .;
    *(always_on_ram_var)
    *(.data_in_retention_ram)
    *(.code_in_retention_ram)
    aon_iram_end = .;
  } >aon

  .static_area : ALIGN (4)
  {
    CREATE_OBJECT_SYMBOLS
    static_area_begin = .;
    KEEP(*(.data_in_static_area))
    static_area_end = .;
  } >static_section
}
/*  */
