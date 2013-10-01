#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "aos/atom_code/output/motor_output.h"
#include "aos/common/logging/logging.h"
#include "aos/atom_code/init.h"

#include "frc971/queues/Piston.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/shooter/shooter_motor.q.h"

using ::frc971::control_loops::drivetrain;
using ::frc971::control_loops::shifters;
using ::frc971::control_loops::shooter;
using ::frc971::control_loops::hangers;

namespace bot3 {
namespace output {

class MotorWriter : public ::aos::MotorOutput {
  // Maximum age of an output packet before the motors get zeroed instead.
  static const int kOutputMaxAgeMS = 20;
  static const int kEnableDrivetrain = true;

  virtual void RunIteration() {
    values_.digital_module = 0;
    values_.pressure_switch_channel = 14;
    values_.compressor_channel = 1;
    values_.solenoid_module = 0;

    drivetrain.output.FetchLatest();
    if (drivetrain.output.IsNewerThanMS(kOutputMaxAgeMS) && kEnableDrivetrain) {
      SetPWMOutput(2, drivetrain.output->right_voltage / 12.0, kTalonBounds);
      SetPWMOutput(3, drivetrain.output->right_voltage / 12.0, kTalonBounds);
      SetPWMOutput(5, -drivetrain.output->left_voltage / 12.0, kTalonBounds);
      SetPWMOutput(6, -drivetrain.output->left_voltage / 12.0, kTalonBounds);
    } else {
      DisablePWMOutput(2);
      DisablePWMOutput(3);
      DisablePWMOutput(5);
      DisablePWMOutput(6);
      if (kEnableDrivetrain) {
        LOG(WARNING, "drivetrain not new enough\n");
      }
    }
    shifters.FetchLatest();
    if (shifters.get()) {
      SetSolenoid(1, shifters->set);
      SetSolenoid(2, !shifters->set);
    }

    shooter.output.FetchLatest();
    if (shooter.output.IsNewerThanMS(kOutputMaxAgeMS)) {
      SetPWMOutput(4, shooter.output->voltage / 12.0, kVictorBounds);
    } else {
      DisablePWMOutput(4);
      LOG(WARNING, "shooter not new enough\n");
    }
    // TODO(danielp): Add stuff for intake.

    hangers.FetchLatest();
    if (hangers.get()) {
      SetSolenoid(4, hangers->set);
      SetSolenoid(5, hangers->set);
    }
  }
};

}  // namespace output
}  // namespace bot3

int main() {
  ::aos::Init();
  ::bot3::output::MotorWriter writer;
  writer.Run();
  ::aos::Cleanup();
}
