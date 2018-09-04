#include "motors/peripheral/adc.h"

#include "motors/core/kinetis.h"

namespace frc971 {
namespace motors {
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

#define ADC_INIT_SINGLE(n, maybe_muxsel)                                     \
  do {                                                                       \
    ADC##n##_CFG1 =                                                          \
        ADC_CFG1_ADIV(2) /* Divide clock by 4 to get 15MHz. */ |             \
        0 /* !ADLSMP to sample faster */ |                                   \
        ADC_CFG1_MODE(1) /* 12 bit single-ended or 13-bit differential. */ | \
        ADC_CFG1_ADICLK(0) /* Use the bus clock (60MHz). */;                 \
    ADC##n##_CFG2 = (maybe_muxsel) |                                         \
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

}  // namespace

void AdcInitCommon(AdcChannels adc0_channels, AdcChannels adc1_channels) {
  SIM_SCGC3 |= SIM_SCGC3_ADC1;
  SIM_SCGC6 |= SIM_SCGC6_ADC0;
  // TODO(Brian): Mess with SIM_SOPT7 to reconfigure ADC trigger input source?
  ADC_INIT_SINGLE(0, (adc0_channels == AdcChannels::kB) ? ADC_CFG2_MUXSEL : 0);
  ADC_INIT_SINGLE(1, (adc1_channels == AdcChannels::kB) ? ADC_CFG2_MUXSEL : 0);
}

}  // namespace motors
}  // namespace frc971
