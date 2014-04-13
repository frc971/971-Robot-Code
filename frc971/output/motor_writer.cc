#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "aos/prime/output/motor_output.h"
#include "aos/common/logging/logging.h"
#include "aos/linux_code/init.h"
#include "aos/common/util/log_interval.h"
#include "aos/common/time.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/controls/output_check.q.h"

#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/shooter/shooter.q.h"

using ::aos::util::SimpleLogInterval;

namespace frc971 {
namespace output {

class MotorWriter : public ::aos::MotorOutput {
  // Maximum age of an output packet before the motors get zeroed instead.
  static const int kOutputMaxAgeMS = 20;
  static constexpr ::aos::time::Time kOldLogInterval =
      ::aos::time::Time::InSeconds(0.5);

  double Cap(double value, double max) {
    if (value > max) return max;
    if (value < -max) return -max;
    return value;
  }

  virtual void RunIteration() {
    values_.digital_module = 0;
    values_.pressure_switch_channel = 1;
    values_.compressor_channel = 1;
    values_.solenoid_module = 0;

    if (true) {
      static auto &drivetrain = ::frc971::control_loops::drivetrain.output;
      drivetrain.FetchLatest();
      if (drivetrain.IsNewerThanMS(kOutputMaxAgeMS)) {
        LOG_STRUCT(DEBUG, "will output", *drivetrain);
        SetPWMOutput(3, drivetrain->right_voltage / 12.0, kTalonBounds);
        SetPWMOutput(6, -drivetrain->left_voltage / 12.0, kTalonBounds);
        SetSolenoid(7, drivetrain->left_high);
        SetSolenoid(8, drivetrain->right_high);
      } else {
        DisablePWMOutput(3);
        DisablePWMOutput(8);
        LOG_INTERVAL(drivetrain_old_);
      }
      drivetrain_old_.Print();
    }

    {
      static auto &shooter =
          ::frc971::control_loops::shooter_queue_group.output;
      shooter.FetchLatest();
      if (shooter.IsNewerThanMS(kOutputMaxAgeMS)) {
        LOG_STRUCT(DEBUG, "will output", *shooter);
        SetPWMOutput(7, shooter->voltage / 12.0, kTalonBounds);
        SetSolenoid(6, !shooter->latch_piston);
        SetSolenoid(5, !shooter->brake_piston);
      } else {
        DisablePWMOutput(9);
        SetSolenoid(5, false);  // engage the brake
        LOG_INTERVAL(shooter_old_);
      }
      shooter_old_.Print();
    }

    {
      static auto &claw = ::frc971::control_loops::claw_queue_group.output;
      claw.FetchLatest();
      if (claw.IsNewerThanMS(kOutputMaxAgeMS)) {
        LOG_STRUCT(DEBUG, "will output", *claw);
        SetPWMOutput(9, claw->intake_voltage / 12.0, kTalonBounds);
        SetPWMOutput(8, claw->intake_voltage / 12.0, kTalonBounds);
        SetPWMOutput(1, -claw->bottom_claw_voltage / 12.0, kTalonBounds);
        SetPWMOutput(2, claw->top_claw_voltage / 12.0, kTalonBounds);
        SetPWMOutput(5, claw->tusk_voltage / 12.0, kTalonBounds);  // left
        SetPWMOutput(4, -claw->tusk_voltage / 12.0, kTalonBounds);  // right
      } else {
        DisablePWMOutput(6);
        DisablePWMOutput(7);
        DisablePWMOutput(1);
        DisablePWMOutput(2);
        DisablePWMOutput(4);
        DisablePWMOutput(5);
        LOG_INTERVAL(claw_old_);
      }
      claw_old_.Print();
    }

    {
      auto message = ::aos::controls::output_check_sent.MakeMessage();
      ++output_check_;
      if (output_check_ == 0) output_check_ = 1;
      SetRawPWMOutput(10, output_check_);
      message->pwm_value = output_check_;
      message->pulse_length =
          static_cast<double>(message->pwm_value) / 255.0 * 2.0 + 0.5;
      LOG_STRUCT(DEBUG, "sending", *message);
      message.Send();
    }
  }

  SimpleLogInterval drivetrain_old_ =
      SimpleLogInterval(kOldLogInterval, WARNING, "drivetrain too old");
  SimpleLogInterval shooter_old_ =
      SimpleLogInterval(kOldLogInterval, WARNING, "shooter too old");
  SimpleLogInterval claw_old_ =
      SimpleLogInterval(kOldLogInterval, WARNING, "claw too old");

  uint8_t output_check_ = 0;
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
