#ifndef FRC971_WPILIB_WPILIB_UTILS_H_
#define FRC971_WPILIB_WPILIB_UTILS_H_

#include <functional>

#include "frc971/constants.h"

namespace frc971 {
namespace wpilib {

// Convert min and max angle positions from range to voltage and compare to
// min and max potentiometer voltage to check if in range.

// subsystem_range is a ::frc971::constants::Range that defines the pot limits
// potentiometer_offset is a constant that is the initial offset of the pot
// pot_translate_inverse is a function that translates an angle to voltage
// reverse is a boolean that sets the pot voltage range as -5 to 0 when true
// limit_buffer is a constant that is the buffer for the maximum voltage values

bool SafePotVoltageRange(::frc971::constants::Range subsystem_range,
                         double potentiometer_offset,
                         ::std::function<double(double)> pot_translate_inverse,
                         bool reverse, double limit_buffer = 0.05);

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_WPILIB_UTILS_H_