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
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/shooter/shooter.q.h"

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
    values_.pressure_switch_channel = 1;
    values_.compressor_channel = 1;
    values_.solenoid_module = 0;

    if (true) {
      drivetrain.output.FetchLatest();
      if (drivetrain.output.IsNewerThanMS(kOutputMaxAgeMS)) {
        LOG_STRUCT(DEBUG, "will output", *drivetrain.output.get());
        SetPWMOutput(3, drivetrain.output->right_voltage / 12.0, kTalonBounds);
        SetPWMOutput(8, -drivetrain.output->left_voltage / 12.0, kTalonBounds);
        SetSolenoid(7, drivetrain.output->left_high);
        SetSolenoid(8, drivetrain.output->right_high);
      } else {
        DisablePWMOutput(3);
        DisablePWMOutput(8);
        drivetrain_old_.WantToLog();
      }
      LOG_INTERVAL(drivetrain_old_);
    }

    {
      static auto &shooter =
          ::frc971::control_loops::shooter_queue_group.output;
      shooter.FetchLatest();
      if (shooter.IsNewerThanMS(kOutputMaxAgeMS)) {
        LOG_STRUCT(DEBUG, "will output", *shooter.get());
        SetPWMOutput(9, shooter->voltage / 12.0, kTalonBounds);
        SetSolenoid(6, !shooter->latch_piston);
        SetSolenoid(5, !shooter->brake_piston);
      } else {
        DisablePWMOutput(9);
        SetSolenoid(5, false);  // engage the brake
        shooter_old_.WantToLog();
      }
      LOG_INTERVAL(shooter_old_);
    }

    {
      static auto &claw = ::frc971::control_loops::claw_queue_group.output;
      claw.FetchLatest();
      if (claw.IsNewerThanMS(kOutputMaxAgeMS)) {
        LOG_STRUCT(DEBUG, "will output", *claw.get());
        SetPWMOutput(6, claw->intake_voltage / 12.0, kTalonBounds);
        SetPWMOutput(7, claw->intake_voltage / 12.0, kTalonBounds);
        SetPWMOutput(1, claw->bottom_claw_voltage / 12.0, kTalonBounds);
        SetPWMOutput(2, claw->top_claw_voltage / 12.0, kTalonBounds);
        SetPWMOutput(5, claw->tusk_voltage / 12.0, kTalonBounds);  // left
        SetPWMOutput(4, claw->tusk_voltage / 12.0, kTalonBounds);  // right
      } else {
        DisablePWMOutput(6);
        DisablePWMOutput(7);
        DisablePWMOutput(1);
        DisablePWMOutput(2);
        DisablePWMOutput(4);
        DisablePWMOutput(5);
        claw_old_.WantToLog();
      }
      LOG_INTERVAL(claw_old_);
    }
  }

  SimpleLogInterval drivetrain_old_ =
      SimpleLogInterval(kOldLogInterval, WARNING, "drivetrain too old");
  SimpleLogInterval shooter_old_ =
      SimpleLogInterval(kOldLogInterval, WARNING, "shooter too old");
  SimpleLogInterval claw_old_ =
      SimpleLogInterval(kOldLogInterval, WARNING, "claw too old");
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
