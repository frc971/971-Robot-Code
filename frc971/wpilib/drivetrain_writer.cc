#include "frc971/wpilib/drivetrain_writer.h"

#include "aos/commonmath.h"
#include "aos/logging/logging.h"
#include "aos/logging/queue_logging.h"
#include "frc971/control_loops/drivetrain/drivetrain.q.h"
#include "frc971/wpilib/ahal/PWM.h"
#include "frc971/wpilib/loop_output_handler.h"

namespace frc971 {
namespace wpilib {

void DrivetrainWriter::Write() {
  auto &queue = ::frc971::control_loops::drivetrain_queue.output;
  LOG_STRUCT(DEBUG, "will output", *queue);
  left_controller0_->SetSpeed(SafeSpeed(reversed_left0_, queue->left_voltage));
  right_controller0_->SetSpeed(
      SafeSpeed(reversed_right0_, queue->right_voltage));

  if (left_controller1_) {
    left_controller1_->SetSpeed(
        SafeSpeed(reversed_left1_, queue->left_voltage));
  }
  if (right_controller1_) {
    right_controller1_->SetSpeed(
        SafeSpeed(reversed_right1_, queue->right_voltage));
  }
}

void DrivetrainWriter::Read() {
  ::frc971::control_loops::drivetrain_queue.output.FetchAnother();
}

void DrivetrainWriter::Stop() {
  LOG(WARNING, "drivetrain output too old\n");
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
