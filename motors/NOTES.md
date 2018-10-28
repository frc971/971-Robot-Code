* Teensy 3.5 has a MK64FX512VMD12 (Freescale Kinetis K64 sub-family)
    * It's a Cortex-M4F, which means it has an FPU and DSP instructions.
    * 512 KB of flash
    * 256 KB SRAM (64 KB SRAM\_L, 192 KB SRAM\_U)
    * Up to 120 MHz
    * [datasheet](http://cache.freescale.com/files/microcontrollers/doc/data_sheet/K64P144M120SF5.pdf)
    * [errata](https://www.nxp.com/docs/en/errata/Kinetis_K_1N83J.pdf)
        * TODO(Brian): Are all of our parts this revision?
    * [K64 reference manual](http://cache.nxp.com/assets/documents/data/en/reference-manuals/K64P144M120SF5RM.pdf)
    * [schematic](https://www.pjrc.com/teensy/schematic.html).
* [actual docs on the bit-banding](http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0439b/Behcjiic.html)
* [ARM Cortex-M Programming Guide to Memory Barrier Instructions](https://static.docs.arm.com/dai0321/a/DAI0321A_programming_guide_memory_barriers_for_m_profile.pdf)
* [Cortex-M4 instruction timings](http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0439b/CHDDIGAC.html)
* [Optimizing Performance on Kinetis K-series MCUs](https://www.nxp.com/docs/en/application-note/AN4745.pdf)
* [motor](https://hobbyking.com/en_us/turnigy-aquastar-t20-3t-730kv-1280kv-water-cooled-brushless-motor.html)
    * 2 pole pairs per revolution
* [RM08](https://www.rls.si/eng/rm08-super-small-non-contact-rotary-encoder), 5v, 2048 edges per revolution, single ended incremental, with index pulse
* fet12v2 has a MK22FN1M0AVLK12
    * It's a Cortex-M4F, which means it has an FPU and DSP instructions.
    * 1 MB of flash
    * 128 KB SRAM (64 KB SRAM\_L, 64 KB SRAM\_U)
    * Alternative part is MK22FX512AVLK12 with 512 KB of flash
    * [datasheet](https://www.nxp.com/docs/en/data-sheet/K22P80M120SF5V2.pdf)
    * [reference manual](https://www.nxp.com/docs/en/reference-manual/K22P80M120SF5V2RM.pdf)
* fet12v2accessory has a MK22FN1M0AVLH12
    * It's a Cortex-M4F, which means it has an FPU and DSP instructions.
    * 1 MB of flash
    * 128 KB SRAM (64 KB SRAM\_L, 64 KB SRAM\_U)
    * Alternative part is MK22FX512AVLH12 with 512 KB of flash
    * [datasheet](https://www.nxp.com/docs/en/data-sheet/K22P64M120SF5V2.pdf)
    * [reference manual](https://www.nxp.com/docs/en/reference-manual/K22P64M120SF5V2RM.pdf)
* [ARMv7-M Architecture Reference Manual](https://static.docs.arm.com/ddi0403/eb/DDI0403E_B_armv7m_arm.pdf)
* TODO(Brian): Turn the cache on. Writethrough caching for the flash would be useful.
  https://www.nxp.com/docs/en/application-note/AN4745.pdf "Cache initialization"
  has steps.

### Clocking
* Running the core clock at its maximum of 120 MHz
* Bus clocks (both of them) are their maximum of 60 MHz

### Timing (what triggers what)
Coordinating the timing of everything is pretty important. The general idea is
to keep everything in sync based on the FTM module(s) which drive the motor
outputs. They trigger the ADCs too.

FTM0 and FTM3 are synced using the global time base when driving two motors
for the pistol grip controller.

The timing is divided into "cycles". Each cycle is a fixed length of time.
The start of the cycle is when the FTM module(s) have a count of 0.

See `//motors/peripheral:adc_dma.cc` for details on how hardware-triggered ADC
sampling works.
