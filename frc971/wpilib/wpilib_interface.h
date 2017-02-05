#ifndef FRC971_WPILIB_WPILIB_INTERFACE_H_
#define FRC971_WPILIB_WPILIB_INTERFACE_H_

#include <stdint.h>

#ifdef WPILIB2017
namespace frc {
class DriverStation;
}  // namespace frc
#else
class DriverStation;
namespace frc {
using ::DriverStation;
}  // namespace frc
#endif

namespace frc971 {
namespace wpilib {

// Sends out a message on ::aos::robot_state.
void SendRobotState(int32_t my_pid, ::frc::DriverStation *ds);

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_WPILIB_INTERFACE_H_
