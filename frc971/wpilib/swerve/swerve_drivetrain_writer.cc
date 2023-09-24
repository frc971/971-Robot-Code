#include "frc971/wpilib/swerve/swerve_drivetrain_writer.h"

using frc971::wpilib::swerve::DrivetrainWriter;

DrivetrainWriter::DrivetrainWriter(::aos::EventLoop *event_loop,
                                   int drivetrain_writer_priority,
                                   double max_voltage)
    : ::frc971::wpilib::LoopOutputHandler<
          ::frc971::control_loops::drivetrain::swerve::Output>(event_loop,
                                                               "/drivetrain"),
      max_voltage_(max_voltage) {
  event_loop->SetRuntimeRealtimePriority(drivetrain_writer_priority);

  event_loop->OnRun([this]() { WriteConfigs(); });
}

void DrivetrainWriter::set_falcons(std::shared_ptr<SwerveModule> front_left,
                                   std::shared_ptr<SwerveModule> front_right,
                                   std::shared_ptr<SwerveModule> back_left,
                                   std::shared_ptr<SwerveModule> back_right) {
  front_left_ = std::move(front_left);
  front_right_ = std::move(front_right);
  back_left_ = std::move(back_left);
  back_right_ = std::move(back_right);
}

void DrivetrainWriter::HandleCANConfiguration(
    const CANConfiguration &configuration) {
  for (auto module : {front_left_, front_right_, back_left_, back_right_}) {
    module->rotation->PrintConfigs();
    module->translation->PrintConfigs();
  }
  if (configuration.reapply()) {
    WriteConfigs();
  }
}

void DrivetrainWriter::WriteConfigs() {
  for (auto module : {front_left_, front_right_, back_left_, back_right_}) {
    module->rotation->WriteConfigs();
    module->translation->WriteConfigs();
  }
}

void DrivetrainWriter::Write(
    const ::frc971::control_loops::drivetrain::swerve::Output &output) {
  front_left_->WriteModule(output.front_left_output(), max_voltage_);
  front_right_->WriteModule(output.front_right_output(), max_voltage_);
  back_left_->WriteModule(output.back_left_output(), max_voltage_);
  back_right_->WriteModule(output.back_right_output(), max_voltage_);
}

void DrivetrainWriter::Stop() {
  AOS_LOG(WARNING, "drivetrain output too old\n");

  for (auto module : {front_left_, front_right_, back_left_, back_right_}) {
    module->rotation->WriteCurrent(0, 0);
    module->translation->WriteCurrent(0, 0);
  }
}
