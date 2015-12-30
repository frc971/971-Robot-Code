#ifndef FRC971_WPILIB_WPILIB_INTERFACE_H_
#define FRC971_WPILIB_WPILIB_INTERFACE_H_

#include <stdint.h>

class DriverStation;

namespace frc971 {
namespace wpilib {

class PDPFetcher;

// Sends out a message on ::aos::robot_state.
void SendRobotState(int32_t my_pid, DriverStation *ds,
                    PDPFetcher *pdp_fetcher);

}  // namespace wpilib
}  // namespace frc971

#endif  // FRC971_WPILIB_WPILIB_INTERFACE_H_
