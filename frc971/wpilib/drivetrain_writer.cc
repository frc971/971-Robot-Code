#include "frc971/wpilib/drivetrain_writer.h"

#include "aos/commonmath.h"
#include "aos/logging/logging.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/wpilib/ahal/PWM.h"
#include "frc971/wpilib/loop_output_handler.h"

namespace frc971 {
namespace wpilib {

void DrivetrainWriter::Write(
    const ::frc971::control_loops::drivetrain::Output &output) {
  left_controller0_->SetSpeed(
      SafeSpeed(reversed_left0_, output.left_voltage()));
  right_controller0_->SetSpeed(
      SafeSpeed(reversed_right0_, output.right_voltage()));

  if (left_controller1_) {
    left_controller1_->SetSpeed(
        SafeSpeed(reversed_left1_, output.left_voltage()));
  }
  if (right_controller1_) {
    right_controller1_->SetSpeed(
        SafeSpeed(reversed_right1_, output.right_voltage()));
  }
}

void DrivetrainWriter::Stop() {
  AOS_LOG(WARNING, "drivetrain output too old\n");
  left_controller0_->SetDisabled();
  right_controller0_->SetDisabled();

  if (left_controller1_) {
    left_controller1_->SetDisabled();
  }
  if (right_controller1_) {
    right_controller1_->SetDisabled();
  }
}

}  // namespace wpilib
}  // namespace frc971
