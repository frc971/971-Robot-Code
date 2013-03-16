#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "aos/aos_core.h"
#include "aos/common/control_loop/Timing.h"
#include "aos/common/messages/RobotState.q.h"
#include "aos/atom_code/output/MotorOutput.h"

#include "frc971/queues/Piston.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/constants.h"
#include "frc971/control_loops/wrist/wrist_motor.q.h"
#include "frc971/control_loops/shooter/shooter_motor.q.h"
#include "frc971/control_loops/index/index_motor.q.h"
#include "frc971/control_loops/angle_adjust/angle_adjust_motor.q.h"

using ::frc971::control_loops::drivetrain;
using ::frc971::control_loops::shifters;
using ::frc971::control_loops::wrist;
using ::frc971::control_loops::shooter;
using ::frc971::control_loops::index_loop;
using ::frc971::control_loops::angle_adjust;

namespace frc971 {
namespace output {

class MotorWriter : public aos::MotorOutput {
  // Maximum age of an output packet before the motors get zeroed instead.
  static const int kOutputMaxAgeMS = 20;

  void RunIteration() {
    drivetrain.output.FetchLatest();
    if (drivetrain.output.IsNewerThanMS(kOutputMaxAgeMS) && false) {
      AddMotor(TALON, 2, drivetrain.output->right_voltage / 12.0);
      AddMotor(TALON, 3, drivetrain.output->right_voltage / 12.0);
      AddMotor(TALON, 5, -drivetrain.output->left_voltage / 12.0);
      AddMotor(TALON, 6, -drivetrain.output->left_voltage / 12.0);
    } else {
      AddMotor(TALON, 2, 0);
      AddMotor(TALON, 3, 0);
      AddMotor(TALON, 5, 0);
      AddMotor(TALON, 6, 0);
      //LOG(WARNING, "drivetrain not new enough\n");
    }
    shifters.FetchLatest();
    if (shifters.IsNewerThanMS(kOutputMaxAgeMS)) {
      AddSolenoid(1, shifters->set);
      AddSolenoid(2, !shifters->set);
    }

    wrist.output.FetchLatest();
    if (wrist.output.IsNewerThanMS(kOutputMaxAgeMS)) {
      AddMotor(TALON, 10, wrist.output->voltage / 12.0);
    } else {
      AddMotor(TALON, 10, 0);
      LOG(WARNING, "wrist not new enough\n");
    }

    shooter.output.FetchLatest();
    if (shooter.output.IsNewerThanMS(kOutputMaxAgeMS)) {
      AddMotor(TALON, 4, shooter.output->voltage / 12.0);
    } else {
      AddMotor(TALON, 4, 0);
      LOG(WARNING, "shooter not new enough\n");
    }

    angle_adjust.output.FetchLatest();
    if (angle_adjust.output.IsNewerThanMS(kOutputMaxAgeMS)) {
      AddMotor(TALON, 1, -angle_adjust.output->voltage / 12.0);
    } else {
      AddMotor(TALON, 1, 0);
      LOG(WARNING, "angle adjust is not new enough\n");
    }

    index_loop.output.FetchLatest();
    if (index_loop.output.IsNewerThanMS(kOutputMaxAgeMS)) {
      AddMotor(TALON, 8, index_loop.output->intake_voltage / 12.0);
      AddMotor(TALON, 9, index_loop.output->transfer_voltage / 12.0);
      AddMotor(TALON, 7, -index_loop.output->index_voltage / 12.0);
      AddSolenoid(7, index_loop.output->loader_up);
      AddSolenoid(8, !index_loop.output->loader_up);
      AddSolenoid(6, index_loop.output->disc_clamped);
      AddSolenoid(3, index_loop.output->disc_ejected);
    } else {
      AddMotor(TALON, 8, 0);
      AddMotor(TALON, 9, 0);
      AddMotor(TALON, 7, 0);
      LOG(WARNING, "index not new enough\n");
    }
  }
};

}  // namespace output
}  // namespace frc971

AOS_RUN(frc971::output::MotorWriter)
