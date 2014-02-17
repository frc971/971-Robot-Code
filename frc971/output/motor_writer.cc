#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "aos/prime/output/motor_output.h"
#include "aos/common/logging/logging.h"
#include "aos/linux_code/init.h"
#include "aos/common/util/log_interval.h"
#include "aos/common/time.h"
#include "aos/common/logging/queue_logging.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"

using ::aos::util::SimpleLogInterval;

using ::frc971::control_loops::drivetrain;

namespace frc971 {
namespace output {

class MotorWriter : public ::aos::MotorOutput {
  // Maximum age of an output packet before the motors get zeroed instead.
  static const int kOutputMaxAgeMS = 20;
  static constexpr ::aos::time::Time kOldLogInterval =
      ::aos::time::Time::InSeconds(0.5);

  virtual void RunIteration() {
    values_.digital_module = 0;
    values_.pressure_switch_channel = 14;
    values_.compressor_channel = 1;
    values_.solenoid_module = 0;

    if (true) {
      drivetrain.output.FetchLatest();
      if (drivetrain.output.get()) {
        LOG_STRUCT(DEBUG, "will output", *drivetrain.output.get());
      }
      if (drivetrain.output.IsNewerThanMS(kOutputMaxAgeMS)) {
        SetPWMOutput(2, drivetrain.output->right_voltage / 12.0, kTalonBounds);
        SetPWMOutput(3, drivetrain.output->right_voltage / 12.0, kTalonBounds);
        SetPWMOutput(5, -drivetrain.output->left_voltage / 12.0, kTalonBounds);
        SetPWMOutput(6, -drivetrain.output->left_voltage / 12.0, kTalonBounds);
        SetSolenoid(1, drivetrain.output->left_high);
        SetSolenoid(2, drivetrain.output->right_high);
      } else {
        DisablePWMOutput(2);
        DisablePWMOutput(3);
        DisablePWMOutput(5);
        DisablePWMOutput(6);
        LOG_INTERVAL(drivetrain_old_);
      }
    }
  }

  SimpleLogInterval drivetrain_old_ =
      SimpleLogInterval(kOldLogInterval, WARNING, "drivetrain too old");
};

constexpr ::aos::time::Time MotorWriter::kOldLogInterval;

}  // namespace output
}  // namespace frc971

int main() {
  ::aos::Init();
  ::frc971::output::MotorWriter writer;
  writer.Run();
  ::aos::Cleanup();
}
