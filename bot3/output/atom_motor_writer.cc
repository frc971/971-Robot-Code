#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "aos/atom_code/output/motor_output.h"
#include "aos/common/logging/logging.h"
#include "aos/atom_code/init.h"

#include "frc971/queues/Piston.q.h"
#include "bot3/control_loops/drivetrain/drivetrain.q.h"
#include "bot3/control_loops/shooter/shooter_motor.q.h"

using ::bot3::control_loops::drivetrain;
using ::bot3::control_loops::shooter;
using ::frc971::control_loops::shifters;

namespace bot3 {
namespace output {

class MotorWriter : public ::aos::MotorOutput {
  // Maximum age of an output packet before the motors get zeroed instead.
  static const int kOutputMaxAgeMS = 20;
  static const int kEnableDrivetrain = true;

  virtual void RunIteration() {
    values_.digital_module = 0;
    values_.pressure_switch_channel = 1;
    values_.compressor_channel = 1; //spike
    values_.solenoid_module = 0;

    drivetrain.output.FetchLatest();
    if (drivetrain.output.IsNewerThanMS(kOutputMaxAgeMS) && kEnableDrivetrain) {
      SetPWMOutput(4, drivetrain.output->right_voltage / 12.0, kTalonBounds);
      SetPWMOutput(3, -drivetrain.output->left_voltage / 12.0, kTalonBounds);
    } else {
      DisablePWMOutput(3);
      DisablePWMOutput(4);
      if (kEnableDrivetrain) {
        LOG(WARNING, "drivetrain not new enough\n");
      }
    }
    shifters.FetchLatest();
    if (shifters.get()) {
      SetSolenoid(7, shifters->set);
      SetSolenoid(8, !shifters->set);
    }

    shooter.output.FetchLatest();
    if (shooter.output.IsNewerThanMS(kOutputMaxAgeMS)) {
      SetPWMOutput(2, shooter.output->voltage / 12.0, kVictorBounds);
    } else {
      DisablePWMOutput(2);
      LOG(WARNING, "shooter not new enough\n");
    }
    // TODO(danielp): Add stuff for intake and shooter.
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
