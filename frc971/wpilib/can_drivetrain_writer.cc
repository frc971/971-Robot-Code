#include "frc971/wpilib/can_drivetrain_writer.h"

using frc971::wpilib::CANDrivetrainWriter;

CANDrivetrainWriter::CANDrivetrainWriter(::aos::EventLoop *event_loop)
    : ::frc971::wpilib::LoopOutputHandler<
          ::frc971::control_loops::drivetrain::Output>(event_loop,
                                                       "/drivetrain") {
  event_loop->SetRuntimeRealtimePriority(kDrivetrainWriterPriority);

  event_loop->OnRun([this]() { WriteConfigs(); });
}

void CANDrivetrainWriter::set_falcons(
    std::vector<std::shared_ptr<Falcon>> right_falcons,
    std::vector<std::shared_ptr<Falcon>> left_falcons) {
  right_falcons_ = std::move(right_falcons);
  left_falcons_ = std::move(left_falcons);
}

void CANDrivetrainWriter::HandleCANConfiguration(
    const CANConfiguration &configuration) {
  for (auto falcon : right_falcons_) {
    falcon->PrintConfigs();
  }

  for (auto falcon : left_falcons_) {
    falcon->PrintConfigs();
  }

  if (configuration.reapply()) {
    WriteConfigs();
  }
}

void CANDrivetrainWriter::WriteConfigs() {
  for (auto falcon : right_falcons_) {
    falcon->WriteConfigs();
  }

  for (auto falcon : left_falcons_) {
    falcon->WriteConfigs();
  }
}

void CANDrivetrainWriter::Write(
    const ::frc971::control_loops::drivetrain::Output &output) {
  for (auto falcon : right_falcons_) {
    falcon->WriteVoltage(output.right_voltage());
  }

  for (auto falcon : left_falcons_) {
    falcon->WriteVoltage(output.left_voltage());
  }
}

void CANDrivetrainWriter::Stop() {
  AOS_LOG(WARNING, "Drivetrain CAN output too old.\n");
  for (auto falcon : right_falcons_) {
    falcon->WriteVoltage(0);
  }

  for (auto falcon : left_falcons_) {
    falcon->WriteVoltage(0);
  }
}
