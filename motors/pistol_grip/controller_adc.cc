#include "motors/pistol_grip/controller_adc.h"

#include "motors/peripheral/adc.h"

namespace frc971 {
namespace motors {

void AdcInitSmall() {
  AdcInitCommon();

  // M0_CH0F ADC1_SE17
  PORTA_PCR17 = PORT_PCR_MUX(0);

  // M0_CH1F ADC1_SE14
  PORTB_PCR10 = PORT_PCR_MUX(0);

  // M0_CH2F ADC1_SE15
  PORTB_PCR11 = PORT_PCR_MUX(0);

  // M0_ABS ADC0_SE5b
  PORTD_PCR1 = PORT_PCR_MUX(0);

  // M1_CH0F ADC0_SE13
  PORTB_PCR3 = PORT_PCR_MUX(0);

  // M1_CH1F ADC0_SE12
  PORTB_PCR2 = PORT_PCR_MUX(0);

  // M1_CH2F ADC0_SE14
  PORTC_PCR0 = PORT_PCR_MUX(0);

  // M1_ABS ADC0_SE17
  PORTE_PCR24 = PORT_PCR_MUX(0);

  // WHEEL_ABS ADC0_SE18
  PORTE_PCR25 = PORT_PCR_MUX(0);

  // VIN ADC1_SE5B
  PORTC_PCR9 = PORT_PCR_MUX(0);
}

SmallAdcReadings AdcReadSmall0(const DisableInterrupts &) {
  SmallAdcReadings r;

  ADC1_SC1A = 17;
  while (!(ADC1_SC1A & ADC_SC1_COCO)) {
  }
  ADC1_SC1A = 14;
  r.currents[0] = ADC1_RA;
  while (!(ADC1_SC1A & ADC_SC1_COCO)) {
  }
  ADC1_SC1A = 15;
  r.currents[1] = ADC1_RA;
  while (!(ADC1_SC1A & ADC_SC1_COCO)) {
  }
  r.currents[2] = ADC1_RA;

  return r;
}

SmallAdcReadings AdcReadSmall1(const DisableInterrupts &) {
  SmallAdcReadings r;

  ADC0_SC1A = 13;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  ADC0_SC1A = 12;
  r.currents[0] = ADC0_RA;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  ADC0_SC1A = 14;
  r.currents[1] = ADC0_RA;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  r.currents[2] = ADC0_RA;

  return r;
}

SmallInitReadings AdcReadSmallInit(const DisableInterrupts &) {
  SmallInitReadings r;

  ADC0_SC1A = 5;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  ADC0_SC1A = 17;
  r.motor0_abs = ADC0_RA;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  ADC0_SC1A = 18;
  r.motor1_abs = ADC0_RA;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  r.wheel_abs = ADC0_RA;

  return r;
}

}  // namespace motors
}  // namespace frc971
