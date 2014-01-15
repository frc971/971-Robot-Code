#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "aos/prime/output/motor_output.h"
#include "aos/common/logging/logging.h"
#include "aos/linux_code/init.h"

#include "frc971/queues/Piston.q.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
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
using ::frc971::control_loops::hangers;

namespace frc971 {
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

    wrist.output.FetchLatest();
    if (wrist.output.IsNewerThanMS(kOutputMaxAgeMS)) {
      SetPWMOutput(10, wrist.output->voltage / 12.0, kTalonBounds);
    } else {
      DisablePWMOutput(10);
      LOG(WARNING, "wrist not new enough\n");
    }

    shooter.output.FetchLatest();
    if (shooter.output.IsNewerThanMS(kOutputMaxAgeMS)) {
      SetPWMOutput(4, shooter.output->voltage / 12.0, kTalonBounds);
    } else {
      DisablePWMOutput(4);
      LOG(WARNING, "shooter not new enough\n");
    }

    angle_adjust.output.FetchLatest();
    if (angle_adjust.output.IsNewerThanMS(kOutputMaxAgeMS)) {
      SetPWMOutput(1, -angle_adjust.output->voltage / 12.0, kTalonBounds);
    } else {
      DisablePWMOutput(1);
      LOG(WARNING, "angle adjust is not new enough\n");
    }

    index_loop.output.FetchLatest();
    if (index_loop.output.get()) {
      SetSolenoid(7, index_loop.output->loader_up);
      SetSolenoid(8, !index_loop.output->loader_up);
      SetSolenoid(6, index_loop.output->disc_clamped);
      SetSolenoid(3, index_loop.output->disc_ejected);
    }
    if (index_loop.output.IsNewerThanMS(kOutputMaxAgeMS)) {
      SetPWMOutput(8, index_loop.output->intake_voltage / 12.0, kTalonBounds);
      SetPWMOutput(9, index_loop.output->transfer_voltage / 12.0, kTalonBounds);
      SetPWMOutput(7, -index_loop.output->index_voltage / 12.0, kTalonBounds);
    } else {
      DisablePWMOutput(8);
      DisablePWMOutput(9);
      DisablePWMOutput(7);
      LOG(WARNING, "index not new enough\n");
    }

    hangers.FetchLatest();
    if (hangers.get()) {
      SetSolenoid(4, hangers->set);
      SetSolenoid(5, hangers->set);
    }
  }
};

}  // namespace output
}  // namespace frc971

int main() {
  ::aos::Init();
  ::frc971::output::MotorWriter writer;
  writer.Run();
  ::aos::Cleanup();
}
