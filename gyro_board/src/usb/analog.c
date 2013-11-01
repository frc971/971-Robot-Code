#include "analog.h"

#include "LPC17xx.h"

#define USE_BURST 0

void analog_init(void) {
  SC->PCONP |= PCONP_PCAD;

  // Enable AD0.0, AD0.1, AD0.2, and AD0.3.
  PINCON->PINSEL1 &= ~(3 << 14 | 3 << 16 | 3 << 18 | 3 << 20);
  PINCON->PINSEL1 |= 1 << 14 | 1 << 16 | 1 << 18 | 1 << 20;

#if USE_BURST
  ADC->ADCR = (1 << 0 | 1 << 1 | 1 << 2 | 1 << 3) /* enable all 4 */ |
      7 << 8 /* 100MHz / 8 = 12.5MHz */ |
      1 << 16 /* enable burst mode */ |
      1 << 21 /* turn on ADC */;
#else
  ADC->ADCR = 7 << 8 /* 100MHz / 8 = 12.5MHz */ |
      1 << 21 /* turn on ADC */;
#endif
}

uint16_t analog(int channel) {
#if !USE_BURST
  // Set the channel number to the one we want.
  ADC->ADCR = (ADC->ADCR & ~0xFF) | (1 << channel);
  ADC->ADCR |= 1 << 24;  // start conversion
#endif
  uint32_t value;
  do {
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
      default:
        return 0xFFFF;
    }
  } while (!(value & 1 << 31));

  return (value >> 4) & 0x3FF;
}
