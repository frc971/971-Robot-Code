/* This file originally started as, and has now been extensively modified from:
 * Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
[Optimizing Performance on Kinetis K-series MCUs](https://www.nxp.com/docs/en/application-note/AN4745.pdf)
is helpful for deciding what to do here. Basically,
code or data accesses to SRAM_L (almost) always take a single cycle, but SRAM_U
instruction accesses take two cycles. SRAM_U data accesses are still a single
cycle. Also, a non-core master can simultaneously access either one while the
core is accessing the other one.

However, https://community.nxp.com/thread/329666#comment-431005 reports some
conflicting results from actual experimentation.

Also, at least some of SRAM_U is in one of the bit-banding regions.

In the end, putting the stack in SRAM_L so we get faults instead of corruption
for overflowing the end of it makes the most sense.
*/

ENTRY(_VectorsFlash);

SECTIONS
{
  .text : {
    . = 0;
    KEEP(*(.vectors))
    *(.startup*)
    . = 0x400;
    KEEP(*(.flashconfig*))
    *(SORT_BY_ALIGNMENT(.text*))
    *(SORT_BY_ALIGNMENT(.rodata*))
    . = ALIGN(4);
    __preinit_array_start = .;
    KEEP (*(.preinit_array))
    __preinit_array_end = .;
    __init_array_start = .;
    KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*)))
    KEEP (*(.init_array))
    __init_array_end = .;
    KEEP (*(.init))
    KEEP (*(.fini))
  } > FLASH = 0xFF

  .ARM.exidx : {
    __exidx_start = .;
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    __exidx_end = .;
  } > FLASH

  /* Currently, we hard-code the sizes of everything that goes in .sram_l here.
   * The alternative is linking twice (first time to determine the size, second
   * to actually do it). */
  . = ORIGIN(SRAM_L) + LENGTH(SRAM_L) - 0x8008 /* The tables in //motors:math */;
  __stack_end__ = .;

  .sram_l_data ALIGN(4) : ALIGN(4) {
    ASSERT(. == ALIGN(4), "Alignment failed");
    __data_sram_l_start__ = .;
    *(SORT_BY_ALIGNMENT(.sram_l))
    /* We want to grab smaller versions of the int tables here. When the int
     * tables get larger, they won't fit in SRAM_L, but we only use the larger
     * versions on parts with more SRAM_U. */
    *(SORT_BY_ALIGNMENT(*512EE20static_sin_int_tableE))
    *(SORT_BY_ALIGNMENT(*512EE20static_cos_int_tableE))
    *(SORT_BY_ALIGNMENT(*1024EE20static_sin_int_tableE))
    *(SORT_BY_ALIGNMENT(*1024EE20static_cos_int_tableE))
    *(SORT_BY_ALIGNMENT(*2048EE20static_sin_int_tableE))
    *(SORT_BY_ALIGNMENT(*2048EE20static_cos_int_tableE))
    . = ALIGN(4);
    __data_sram_l_end__ = .;
  } > SRAM_L AT>FLASH

  __sram_l_flash_start__ = LOADADDR(.data);
  . = ALIGN(8);

  /* Deliberately putting .bss before .data because we end up with some things
   * in here that have large alignments, so this minimizes padding. */
  .bss : {
    . = ALIGN(4);
    __bss_ram_start__ = .;
    *(SORT_BY_ALIGNMENT(.bss*))
    /* Stupid precompiled libc has common symbols, so stick them in. */
    */libc_nano.a:*(COMMON)
    . = ALIGN(4);
    __bss_ram_end__ = .;
  } > SRAM_U

  .data ALIGN(4) : ALIGN(4) {
    ASSERT(. == ALIGN(4), "Alignment failed");
    __data_ram_start__ = .;
    *(SORT_BY_ALIGNMENT(.data*))
    . = ALIGN(4);
    __data_ram_end__ = .;
  } > SRAM_U AT>FLASH
  __data_flash_start__ = LOADADDR(.data);

  . = ALIGN(8);
  __heap_start__ = .;
  __heap_end__ = ORIGIN(SRAM_U) + LENGTH(SRAM_U);

  .dummy_asserts : {
    /* Annoyingly, these have to be in some section, so stick them here. */
    ASSERT(__data_flash_start__ == ALIGN(__data_flash_start__, 4), "Flash data must be aligned");
    ASSERT(__stack_end__ == ALIGN(__stack_end__, 8), "Stack must be aligned");
    ASSERT(__sram_l_flash_start__ == ALIGN(__sram_l_flash_start__, 4), "Flash data must be aligned");
  }


  /* We don't want any common symbols, so just don't include them which results
   * in linker errors if any do show up somehow.
   * Contrary to sanity, even with --orphan-handling=error the linker will still
   * do normal orphan section handling with COMMON... */
  /DISCARD/ : {
    *(COMMON);
  }
}
