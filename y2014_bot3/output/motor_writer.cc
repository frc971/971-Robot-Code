#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "aos/output/motor_output.h"
#include "aos/common/logging/logging.h"
#include "aos/linux_code/init.h"
#include "aos/common/util/log_interval.h"
#include "aos/common/time.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/controls/output_check.q.h"

#include "bot3/control_loops/drivetrain/drivetrain.q.h"
#include "bot3/control_loops/rollers/rollers.q.h"

using ::aos::util::SimpleLogInterval;

namespace bot3 {
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
      static auto &drivetrain = ::bot3::control_loops::drivetrain.output;
      drivetrain.FetchLatest();
      if (drivetrain.IsNewerThanMS(kOutputMaxAgeMS)) {
        LOG_STRUCT(DEBUG, "will output", *drivetrain);
        SetPWMOutput(5, drivetrain->right_voltage / 12.0, kTalonBounds);
        SetPWMOutput(2, -drivetrain->left_voltage / 12.0, kTalonBounds);
        SetSolenoid(1, drivetrain->left_high);
        SetSolenoid(8, drivetrain->right_high);
      } else {
        DisablePWMOutput(2);
        DisablePWMOutput(5);
        LOG_INTERVAL(drivetrain_old_);
      }
      drivetrain_old_.Print();
    }

    {
      static auto &rollers = ::bot3::control_loops::rollers.output;
      rollers.FetchLatest();
      if (rollers.IsNewerThanMS(kOutputMaxAgeMS)) {
        LOG_STRUCT(DEBUG, "will output", *rollers);
        // There are two motors for each of these.
        SetPWMOutput(3, rollers->front_intake_voltage / 12.0, kTalonBounds);
        SetPWMOutput(7, -rollers->front_intake_voltage / 12.0, kTalonBounds);
        SetPWMOutput(1, rollers->back_intake_voltage / 12.0, kTalonBounds);
        SetPWMOutput(6, -rollers->back_intake_voltage / 12.0, kTalonBounds);
        SetPWMOutput(4, rollers->low_goal_voltage / 12.0, kTalonBounds);

        SetSolenoid(2, rollers->front_extended);
        SetSolenoid(5, !rollers->front_extended);
        SetSolenoid(3, rollers->back_extended);
        SetSolenoid(4, !rollers->back_extended);
      } else {
        DisablePWMOutput(3);
        DisablePWMOutput(7);
        DisablePWMOutput(1);
        DisablePWMOutput(6);
        DisablePWMOutput(4);

        // Retract intakes.
        SetSolenoid(2, false);
        SetSolenoid(3, false);
        SetSolenoid(5, true);
        SetSolenoid(4, true);

        LOG_INTERVAL(rollers_old_);
      }
      rollers_old_.Print();
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
  SimpleLogInterval rollers_old_ =
      SimpleLogInterval(kOldLogInterval, WARNING, "rollers too old");

  uint8_t output_check_ = 0;
};

constexpr ::aos::time::Time MotorWriter::kOldLogInterval;

}  // namespace output
}  // namespace bot3

int main() {
  ::aos::Init();
  ::bot3::output::MotorWriter writer;
  writer.Run();
  ::aos::Cleanup();
}
