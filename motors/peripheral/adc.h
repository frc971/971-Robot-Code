#ifndef MOTORS_PERIPHERAL_ADC_H_
#define MOTORS_PERIPHERAL_ADC_H_

#include <stdint.h>

#include "motors/util.h"

// TODO(Brian): Avoid cramming all the code for each specific application into a
// single file like this.

namespace frc971 {
namespace motors {

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

struct JoystickAdcReadings {
  uint16_t analog0, analog1, analog2, analog3;
};

struct SimpleAdcReadings {
  uint16_t sin, cos;
};

void AdcInitMedium();
void AdcInitSmall();
void AdcInitJoystick();
void AdcInitSimple();

MediumAdcReadings AdcReadMedium(const DisableInterrupts &);
SmallAdcReadings AdcReadSmall0(const DisableInterrupts &);
SmallAdcReadings AdcReadSmall1(const DisableInterrupts &);
SmallInitReadings AdcReadSmallInit(const DisableInterrupts &);
JoystickAdcReadings AdcReadJoystick(const DisableInterrupts &);
SimpleAdcReadings AdcReadSimple(const DisableInterrupts &);

}  // namespace motors
}  // namespace frc971

#endif  // MOTORS_PERIPHERAL_ADC_H_
