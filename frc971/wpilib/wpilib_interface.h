#ifndef FRC971_WPILIB_WPILIB_INTERFACE_H_
#define FRC971_WPILIB_WPILIB_INTERFACE_H_

#include <stdint.h>

#include "aos/events/event_loop.h"
#include "aos/robot_state/robot_state_generated.h"

namespace frc971 {
namespace wpilib {

// Sends out a message on ::aos::robot_state.
flatbuffers::Offset<aos::RobotState> PopulateRobotState(
    aos::Sender<::aos::RobotState>::Builder *builder, int32_t my_pid);

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_WPILIB_INTERFACE_H_
