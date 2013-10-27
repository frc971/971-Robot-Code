#include "analog.h"

#include "LPC17xx.h"

void analog_init(void) {
  SC->PCONP |= PCONP_PCAD;

  // Enable AD0.0, AD0.1, AD0.2, AD0.3
  PINCON->PINSEL1 &= ~(2 << 14 | 2 << 16 | 2 << 18 | 2 << 20);
  PINCON->PINSEL1 |= 1 << 14 | 1 << 16 | 1 << 18 | 1 << 20;
  ADC->ADCR = 0x00200500;
  ADC->ADCR = (1 << 0 | 1 << 1 | 1 << 2 | 1 << 3) /* enable all 4 */ |
      7 << 8 /* 100MHz / 8 = 12.5MHz */ |
      1 << 16 /* enable burst mode */ |
      1 << 21 /* turn on ADC */;
}

uint16_t analog(int channel) {
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

  return ((value & 0xFFF0) >> 4);
}
