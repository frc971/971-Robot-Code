#ifndef MOTORS_PERIPHERAL_ADC_H_
#define MOTORS_PERIPHERAL_ADC_H_

#include <stdint.h>

namespace frc971 {
namespace salsa {

struct MediumAdcReadings {
  uint16_t motor_currents[3][2];
  uint16_t motor_current_ref;
  uint16_t input_voltage;
};

void AdcInit();

MediumAdcReadings AdcReadMedium();

}  // namespace salsa
}  // namespace frc971

#endif  // MOTORS_PERIPHERAL_ADC_H_
