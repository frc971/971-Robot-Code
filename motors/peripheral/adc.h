#ifndef MOTORS_PERIPHERAL_ADC_H_
#define MOTORS_PERIPHERAL_ADC_H_

#include <stdint.h>

#include "motors/util.h"

namespace frc971 {
namespace salsa {

struct MediumAdcReadings {
  uint16_t motor_currents[3][2];
  uint16_t motor_current_ref;
  uint16_t input_voltage;
};

struct SmallAdcReadings {
  uint16_t currents[3];
};

struct SmallInitReadings {
  uint16_t motor0_abs;
  uint16_t motor1_abs;
  uint16_t wheel_abs;
};

void AdcInitMedium();
void AdcInitSmall();

MediumAdcReadings AdcReadMedium(const DisableInterrupts &);
SmallAdcReadings AdcReadSmall0(const DisableInterrupts &);
SmallAdcReadings AdcReadSmall1(const DisableInterrupts &);
SmallInitReadings AdcReadSmallInit(const DisableInterrupts &);

}  // namespace salsa
}  // namespace frc971

#endif  // MOTORS_PERIPHERAL_ADC_H_
