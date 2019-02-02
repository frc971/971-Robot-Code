#ifndef FRC971_WPILIB_WPILIB_INTERFACE_H_
#define FRC971_WPILIB_WPILIB_INTERFACE_H_

#include <stdint.h>

#include "aos/robot_state/robot_state.q.h"

namespace frc971 {
namespace wpilib {

// Sends out a message on ::aos::robot_state.
void PopulateRobotState(::aos::RobotState *robot_state, int32_t my_pid);

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_WPILIB_INTERFACE_H_
