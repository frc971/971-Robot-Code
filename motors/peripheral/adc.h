#ifndef MOTORS_PERIPHERAL_ADC_H_
#define MOTORS_PERIPHERAL_ADC_H_

#include <stdint.h>

#include "motors/util.h"

namespace frc971::motors {

enum class AdcChannels {
  kA,
  kB,
};
void AdcInitCommon(AdcChannels adc0_channels = AdcChannels::kB,
                   AdcChannels adc1_channels = AdcChannels::kB);

}  // namespace frc971::motors

#endif  // MOTORS_PERIPHERAL_ADC_H_
