#include "analog.h"

#include "LPC17xx.h"
#include "FreeRTOS.h"

static int discarded_samples[4];

static uint16_t raw_analog(int channel) {
  uint32_t value;
  switch (channel) {
    case 0:
      value = ADC->ADDR0;
      break;
    case 1:
      value = ADC->ADDR1;
      break;
    case 2:
      value = ADC->ADDR2;
      break;
    case 3:
      value = ADC->ADDR3;
      break;
  }

  return (value >> 4) & 0xFFF;
}

void TIMER1_IRQHandler(void) {
  TIM1->IR = 1 << 0;  // clear channel 0 match

  static const int kBadSampleThreshold = 175;
  static const int kMaxBadSamples = 4;

  static const uint32_t kBitShift = 16;
  static const uint32_t kA =
      (1.0 - 0.8408964152537146 /*0.5^0.25*/) * (1 << kBitShift) + 0.5;
  for (int i = 0; i < 4; ++i) {
    uint16_t current = raw_analog(i);
    uint16_t average = averaged_values[i];
    if ((current - average) < -kBadSampleThreshold ||
        (current - average) > kBadSampleThreshold) {
      ++discarded_samples[i];
      if (discarded_samples[i] >= kMaxBadSamples) {
        discarded_samples[i] = 0;
        averaged_values[i] = current;
      }
    } else {
      discarded_samples[i] = 0;
      averaged_values[i] =
          ((uint32_t)current * kA +
           (uint32_t)average * ((1 << kBitShift) - kA)) >> kBitShift;
    }
  }
}

void analog_init(void) {
  SC->PCONP |= PCONP_PCAD;

  // Enable AD0.0, AD0.1, AD0.2, and AD0.3 (0.23 - 0.26).
  PINCON->PINSEL1 &= ~(3 << 14 | 3 << 16 | 3 << 18 | 3 << 20);
  PINCON->PINSEL1 |= 1 << 14 | 1 << 16 | 1 << 18 | 1 << 20;
  PINCON->PINMODE1 &= ~(3 << 14 | 3 << 16 | 3 << 18 | 3 << 20);
  PINCON->PINMODE1 |= 2 << 14 | 2 << 16 | 2 << 18 | 2 << 20;

  ADC->ADCR = (1 << 0 | 1 << 1 | 1 << 2 | 1 << 3) /* enable all 4 */ |
      79 << 8 /* takes 208us to scan through all 4 */ |
      1 << 16 /* enable burst mode */ |
      1 << 21 /* turn on ADC */;

  // Set up the timer for the low-pass filter.
  SC->PCONP |= 1 << 2;
  TIM1->PR = (configCPU_CLOCK_HZ / 2000) - 1;
  TIM1->TC = 0;  // don't match the first time around
  TIM1->MR0 = 1;  // match every time it wraps
  TIM1->MCR = 1 << 0 | 1 << 1;  // interrupt and reset on match channel 0
  // Priority 4 is higher than any FreeRTOS-managed stuff (ie USB), but lower
  // than encoders etc.
  NVIC_SetPriority(TIMER1_IRQn, 4);
  NVIC_EnableIRQ(TIMER1_IRQn);
  TIM1->TCR = 1;  // enable it
}
