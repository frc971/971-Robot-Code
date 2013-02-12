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
  // Maximum age of an output packet before the motors get zeroed instead.
  static const int kOutputMaxAgeMS = 20;

  void RunIteration() {
    drivetrain.output.FetchLatest();
    if (drivetrain.output.IsNewerThanMS(kOutputMaxAgeMS)) {
      AddMotor(TALON, 2, drivetrain.output->right_voltage / 12.0);
      AddMotor(TALON, 3, drivetrain.output->right_voltage / 12.0);
      AddMotor(TALON, 5, -drivetrain.output->left_voltage / 12.0);
      AddMotor(TALON, 6, -drivetrain.output->left_voltage / 12.0);
    } else {
      AddMotor(TALON, 2, 0.0f);
      AddMotor(TALON, 3, 0.0f);
      AddMotor(TALON, 5, 0.0f);
      AddMotor(TALON, 6, 0.0f);
      LOG(WARNING, "zeroing drivetrain\n");
    }

    if (shifters.FetchLatest()) {
      AddSolenoid(1, shifters->set);
      AddSolenoid(2, !shifters->set);
    }
  }
};

}  // namespace output
}  // namespace frc971

AOS_RUN(frc971::output::MotorWriter)
