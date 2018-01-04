/* Teensyduino Core Library
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

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 512K
  RAM  (rwx) : ORIGIN = 0x1FFF0000, LENGTH = 192K
}

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
    KEEP(*(.init))
    . = ALIGN(4);
    __preinit_array_start = .;
    KEEP (*(.preinit_array))
    __preinit_array_end = .;
    __init_array_start = .;
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array))
    __init_array_end = .;
  } > FLASH = 0xFF

  .ARM.exidx : {
    __exidx_start = .;
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    __exidx_end = .;
  } > FLASH

  /* Deliberately putting .bss before .data because we end up with some things
   * in here that have large alignments, so this minimizes padding.
   */
  .bss : {
    . = ALIGN(4);
    __bss_ram_start__ = .;
    *(SORT_BY_ALIGNMENT(.bss*))
    /* Stupid precompiled libc has common symbols, so stick them in. */
    */libc_nano.a:*(COMMON)
    . = ALIGN(4);
    __bss_ram_end__ = .;
  } > RAM

  .data : {
    . = ALIGN(4);
    __data_ram_start__ = .;
    *(SORT_BY_ALIGNMENT(.data*))
    . = ALIGN(4);
    __data_ram_end__ = .;
  } > RAM AT>FLASH

  __data_flash_start__ = LOADADDR(.data);

  . = ALIGN(8);
  __heap_start__ = .;

  /* We don't want any common symbols, so just don't include them which results
   * in linker errors if any do show up somehow.
   * Contrary to sanity, even with --orphan-handling=error the linker will still
   * do normal orphan section handling with COMMON... */
  /DISCARD/ : {
    *(COMMON);
  }

  __stack_end__ = ORIGIN(RAM) + LENGTH(RAM);
}
