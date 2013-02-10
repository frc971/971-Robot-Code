#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "aos/aos_core.h"
#include "aos/common/network/SendSocket.h"
#include "aos/common/control_loop/Timing.h"
#include "aos/common/messages/RobotState.q.h"
#include "aos/atom_code/output/MotorOutput.h"

#include "frc971/queues/Piston.q.h"
#include "frc971/control_loops/DriveTrain.q.h"
#include "frc971/constants.h"

using ::frc971::control_loops::drivetrain;
using ::frc971::control_loops::shifters;

namespace frc971 {
namespace output {

class MotorWriter : public aos::MotorOutput {
  void RunIteration() {
    if (drivetrain.output.FetchLatest()) {
      AddMotor(TALON, 7, -drivetrain.output->left_voltage / 12.0);
      AddMotor(TALON, 4, drivetrain.output->right_voltage / 12.0);
    } else {
      AddMotor(TALON, 7, 0.0f);
      AddMotor(TALON, 4, 0.0f);
    }

    if (shifters.FetchLatest()) {
      AddSolenoid(1, shifters->set);
    }
  }
};

}  // namespace output
}  // namespace frc971

AOS_RUN(frc971::output::MotorWriter)
