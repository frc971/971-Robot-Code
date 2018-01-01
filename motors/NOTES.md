* Teensy 3.5 has a MK64FX512VMD12 (Freescale Kinetis K64 sub-family)
    * It's a Cortex-M4F, which means it has an FPU and DSP instructions.
    * 512 KB of flash
    * 192 kB SRAM (64 kB SRAM\_L, 128 kB SRAM\_U)
    * Up to 120 MHz
* [datasheet](http://cache.freescale.com/files/microcontrollers/doc/data_sheet/K64P144M120SF5.pdf)
* [reference manual](http://cache.nxp.com/assets/documents/data/en/reference-manuals/K64P144M120SF5RM.pdf)
* [schematic](https://www.pjrc.com/teensy/schematic.html).
* [actual docs on the bit banding](http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0439b/Behcjiic.html)
* [ARM Cortex-M Programming Guide to Memory Barrier Instructions](https://static.docs.arm.com/dai0321/a/DAI0321A_programming_guide_memory_barriers_for_m_profile.pdf)
* [Cortex-M4 instruction timings](http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0439b/CHDDIGAC.html)
* RM08, 5v, 2048 edges per revolution, single ended incremental, with index pulse
* [motor](https://hobbyking.com/en_us/turnigy-aquastar-t20-3t-730kv-1280kv-water-cooled-brushless-motor.html)
    * 2 pole pairs per revolution

* Change MOSFET gate resistors to 1 ohm (or something else?)
* Reshuffle input capacitor connections
    * Thermals?
    * Farther apart
    * Other side?
* More capacitors on battery power
    * Like across individual half bridges
* No paste on wire pads

### Clocking
* Running the core clock at its maximum of 120 MHz
* Bus clocks (both of them) are their maximum of 60 MHz

### Timing (what triggers what)
Coordinating the timing of everything is pretty important. The general idea is
to keep everything in sync based on the FTM module(s) which drive the motor
outputs. They trigger the ADCs too.

FTM0 and FTM3 are synced using the global time base.

The timing is divided into "cycles". Each cycle is a fixed length of time.
The start of the cycle is when the FTM module(s) have a count of 0.

TODO(Brian): Update the rest of this.

Both PDBs are used to trigger ADC samples, in conjunction with DMA to
automatically setup the correct sequence and record the results. This gives
tight control over the timing (DMA can take a bit to get there, and interrupts
would be worse) and doesn't require CPU time besides reading the result at the
end.
The PDB is triggered by the initialization external trigger of an FTM.
From there, DMA triggered by the ADC conversion complete event finishes and
prepares for the next round.
PDB triggers of ADCs alternate between channels so DMA can read+update the other
one in the mean time.
One DMA channel copies the result out, and then links to another channel which
reconfigures that ADC conversion setup register to prepare for the next time.
This doesn't allow skipping ADC conversions, but conversion results can always
be ignored later.
This also doesn't allow scheduling ADC conversions at any points besides at
fixed multiples from thes start of the cycle, but that should be fine.
The ADC setup should stop converting by the end of the cycle so the new PDB
trigger will restart everything, and both DMA channels should reset themselves
after two cycles (so that the code can read ADC results in one area while
they're being written to the other).

Both PDBs are used to trigger ADC samples, in conjunction with DMA to
automatically setup the correct sequence and record the results. This gives
tight control over the timing (DMA can take a bit to get there, and interrupts
would be worse) and doesn't require CPU time besides reading the result at the
end.
The PDB is triggered by the initialization external trigger of an FTM.
From there, DMA triggered by two unused channels of the FTMs alternates between
copying results and reconfiguring each of the sets of ADC registers.
The DMA uses the SMOD functionality to get the corresponding registers of both
ADCs in one go, which means using a total of 4 DMA channels (1 to configure the
A registers, 1 to configure the B registers, 1 to copy the A results, and 1 to
copy the B results).

Currently, ADC sampling all happens at the beginning of a cycle, then the
computation of new output values happens, then these values need to be
written to the hardware before the next cycle starts.
ADC sampling may move to DMA-based at some point to increase flexibility and
reduce CPU time spent on it.
