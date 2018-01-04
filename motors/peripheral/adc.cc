#include "motors/peripheral/adc.h"

#include "motors/core/kinetis.h"

namespace frc971 {
namespace salsa {
namespace {

#define ADC_SC2_BASE (ADC_SC2_REFSEL(0) /* Use the external reference pins. */)

#define ADC_FINISH_CALIBRATION(n, PM) \
  do {                                \
    uint16_t variable = 0;            \
    variable += ADC##n##_CL##PM##0;   \
    variable += ADC##n##_CL##PM##1;   \
    variable += ADC##n##_CL##PM##2;   \
    variable += ADC##n##_CL##PM##3;   \
    variable += ADC##n##_CL##PM##4;   \
    variable += ADC##n##_CL##PM##S;   \
    variable /= 2;                    \
    variable |= 0x8000;               \
    ADC##n##_##PM##G = variable;      \
  } while (0);

#define ADC_INIT_SINGLE(n)                                                   \
  do {                                                                       \
    ADC##n##_CFG1 = ADC_CFG1_ADIV(2) /* Divide clock by 4 to get 15MHz. */ | \
                    ADC_CFG1_MODE(1) /* 12 bit mode. */ |                    \
                    ADC_CFG1_ADICLK(0) /* Use the bus clock (60MHz). */;     \
    ADC##n##_CFG2 = ADC_CFG2_MUXSEL /* Use the b channels. */ |              \
                    ADC_CFG2_ADHSC /* Support higher ADC clock speeds. */;   \
    ADC##n##_SC1A = 0; /* Clear SC1A's COCO flag. */                         \
    ADC##n##_SC2 = ADC_SC2_BASE;                                             \
    do {                                                                     \
      ADC##n##_SC3 = ADC_SC3_CAL | ADC_SC3_AVGE |                            \
                     ADC_SC3_AVGS(3) /* Average 32 samples (max). */;        \
      /* Wait for calibration to complete. */                                \
      while (!(ADC##n##_SC1A & ADC_SC1_COCO)) {                              \
      }                                                                      \
    } while (ADC##n##_SC3 & ADC_SC3_CALF);                                   \
    ADC_FINISH_CALIBRATION(n, P);                                            \
    ADC_FINISH_CALIBRATION(n, M);                                            \
                                                                             \
    ADC##n##_SC3 = 0 /* Disable hardware averaging. */;                      \
  } while (0)

void AdcInitCommon() {
  SIM_SCGC3 |= SIM_SCGC3_ADC1;
  SIM_SCGC6 |= SIM_SCGC6_ADC0;
  // TODO(Brian): Mess with SIM_SOPT7 to reconfigure ADC trigger input source?
  ADC_INIT_SINGLE(0);
  ADC_INIT_SINGLE(1);
}

}  // namespace

void AdcInitMedium() {
  AdcInitCommon();

  // M_CH2V ADC0_SE14
  PORTC_PCR0 = PORT_PCR_MUX(0);

  // M_CH0V ADC0_SE13
  PORTB_PCR3 = PORT_PCR_MUX(0);

  // M_CH1V ADC0_SE12
  PORTB_PCR2 = PORT_PCR_MUX(0);

  // M_CH0F ADC1_SE14
  PORTB_PCR10 = PORT_PCR_MUX(0);

  // M_CH1F ADC1_SE15
  PORTB_PCR11 = PORT_PCR_MUX(0);

  // M_VREF ADC0_SE18
  PORTE_PCR25 = PORT_PCR_MUX(0);

  // VIN ADC1_SE5B
  PORTC_PCR9 = PORT_PCR_MUX(0);

  // M_CH2F ADC1_SE17
  PORTA_PCR17 = PORT_PCR_MUX(0);
}

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

MediumAdcReadings AdcReadMedium(const DisableInterrupts &) {
  MediumAdcReadings r;

  ADC1_SC1A = 14;
  while (!(ADC1_SC1A & ADC_SC1_COCO)) {
  }
  ADC1_SC1A = 15;
  r.motor_currents[0][0] = ADC1_RA;
  while (!(ADC1_SC1A & ADC_SC1_COCO)) {
  }
  ADC1_SC1A = 17;
  ADC0_SC1A = 18;
  r.motor_currents[1][0] = ADC1_RA;
  while (!(ADC1_SC1A & ADC_SC1_COCO)) {
  }
  ADC1_SC1A = 5;
  r.motor_currents[2][0] = ADC1_RA;
  while (!(ADC0_SC1A & ADC_SC1_COCO)) {
  }
  r.motor_current_ref = ADC0_RA;
  while (!(ADC1_SC1A & ADC_SC1_COCO)) {
  }
  ADC1_SC1A = 14;
  r.input_voltage = ADC1_RA;
  while (!(ADC1_SC1A & ADC_SC1_COCO)) {
  }
  ADC1_SC1A = 15;
  r.motor_currents[0][1] = ADC1_RA;
  while (!(ADC1_SC1A & ADC_SC1_COCO)) {
  }
  ADC1_SC1A = 17;
  r.motor_currents[1][1] = ADC1_RA;
  while (!(ADC1_SC1A & ADC_SC1_COCO)) {
  }
  r.motor_currents[2][1] = ADC1_RA;

  return r;
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

}  // namespace salsa
}  // namespace frc971
