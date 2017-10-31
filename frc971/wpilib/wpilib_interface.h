#ifndef FRC971_WPILIB_WPILIB_INTERFACE_H_
#define FRC971_WPILIB_WPILIB_INTERFACE_H_

#include <stdint.h>

namespace frc971 {
namespace wpilib {

// Sends out a message on ::aos::robot_state.
void SendRobotState(int32_t my_pid);

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_WPILIB_INTERFACE_H_
