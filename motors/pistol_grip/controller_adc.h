#ifndef MOTORS_PISTOL_GRIP_CONTROLLER_ADC_H_
#define MOTORS_PISTOL_GRIP_CONTROLLER_ADC_H_

#include "motors/util.h"

namespace frc971 {
namespace motors {

struct SmallAdcReadings {
  uint16_t currents[3];
};

struct SmallInitReadings {
  uint16_t motor0_abs;
  uint16_t motor1_abs;
  uint16_t wheel_abs;
};

// Initializes the ADC.
void AdcInitSmall();

// Reads motor 0.
SmallAdcReadings AdcReadSmall0(const DisableInterrupts &);

// Reads motor 1.
SmallAdcReadings AdcReadSmall1(const DisableInterrupts &);

// Reads the absolute encoder values for initialization.
SmallInitReadings AdcReadSmallInit(const DisableInterrupts &);

}  // namespace motors
}  // namespace frc971

#endif  // MOTORS_PISTOL_GRIP_CONTROLLER_ADC_H_
