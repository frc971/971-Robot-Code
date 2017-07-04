#ifndef MOTORS_PERIPHERAL_CONFIGURATION_H_
#define MOTORS_PERIPHERAL_CONFIGURATION_H_

// We're just going to leave the default DMA priorities which correspond to the
// channel numbers and fixed priority mode, so channel 0 is the lowest priority
// and 15 is the highest.
// We're also going to leave DMA_CR alone except for setting EMLM.

// The frequency of the peripheral bus(es) in hz.
#define BUS_CLOCK_FREQUENCY (F_CPU / 2)

// The frequency we switch the motor FETs at in hz.
#define SWITCHING_FREQUENCY 20000

#if 0
// Which PDB the ADC triggering uses.
#define ADC_TRIGGER_PDB 0
// The DMA channel which copies ADC results.
#define ADC_RESULT_DMA_CHANNEL 7
// The DMA channel which reconfigures the ADCs to take the next samples.
#define ADC_RECONFIGURE_DMA_CHANNEL 8
#endif

#endif  // MOTORS_PERIPHERAL_CONFIGURATION_H_
