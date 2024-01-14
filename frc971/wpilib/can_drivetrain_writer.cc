#include "frc971/wpilib/can_drivetrain_writer.h"

using frc971::wpilib::CANDrivetrainWriter;

CANDrivetrainWriter::CANDrivetrainWriter(::aos::EventLoop *event_loop)
    : ::frc971::wpilib::LoopOutputHandler<
          ::frc971::control_loops::drivetrain::Output>(event_loop,
                                                       "/drivetrain") {
  event_loop->SetRuntimeRealtimePriority(kDrivetrainWriterPriority);

  event_loop->OnRun([this]() { WriteConfigs(); });
}

void CANDrivetrainWriter::set_talonfxs(
    std::vector<std::shared_ptr<TalonFX>> right_talonfxs,
    std::vector<std::shared_ptr<TalonFX>> left_talonfxs) {
  right_talonfxs_ = std::move(right_talonfxs);
  left_talonfxs_ = std::move(left_talonfxs);
}

void CANDrivetrainWriter::HandleCANConfiguration(
    const CANConfiguration &configuration) {
  for (auto talonfx : right_talonfxs_) {
    talonfx->PrintConfigs();
  }

  for (auto talonfx : left_talonfxs_) {
    talonfx->PrintConfigs();
  }

  if (configuration.reapply()) {
    WriteConfigs();
  }
}

void CANDrivetrainWriter::WriteConfigs() {
  for (auto talonfx : right_talonfxs_) {
    talonfx->WriteConfigs();
  }

  for (auto talonfx : left_talonfxs_) {
    talonfx->WriteConfigs();
  }
}

void CANDrivetrainWriter::Write(
    const ::frc971::control_loops::drivetrain::Output &output) {
  for (auto talonfx : right_talonfxs_) {
    talonfx->WriteVoltage(output.right_voltage());
  }

  for (auto talonfx : left_talonfxs_) {
    talonfx->WriteVoltage(output.left_voltage());
  }
}

void CANDrivetrainWriter::Stop() {
  AOS_LOG(WARNING, "Drivetrain CAN output too old.\n");
  for (auto talonfx : right_talonfxs_) {
    talonfx->WriteVoltage(0);
  }

  for (auto talonfx : left_talonfxs_) {
    talonfx->WriteVoltage(0);
  }
}
