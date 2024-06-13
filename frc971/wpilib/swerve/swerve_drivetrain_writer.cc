#include "frc971/wpilib/swerve/swerve_drivetrain_writer.h"

using frc971::wpilib::swerve::DrivetrainWriter;

DrivetrainWriter::DrivetrainWriter(::aos::EventLoop *event_loop,
                                   int drivetrain_writer_priority,
                                   double max_voltage)
    : ::frc971::wpilib::LoopOutputHandler<
          ::frc971::control_loops::swerve::Output>(event_loop, "/drivetrain"),
      max_voltage_(max_voltage) {
  event_loop->SetRuntimeRealtimePriority(drivetrain_writer_priority);

  event_loop->OnRun([this]() { WriteConfigs(); });
}

void DrivetrainWriter::set_talonfxs(SwerveModules modules) {
  modules_ = std::move(modules);
}

void DrivetrainWriter::HandleCANConfiguration(
    const CANConfiguration &configuration) {
  for (auto module : {modules_.front_left, modules_.front_right,
                      modules_.back_left, modules_.back_right}) {
    module->rotation->PrintConfigs();
    module->translation->PrintConfigs();
  }
  if (configuration.reapply()) {
    WriteConfigs();
  }
}

void DrivetrainWriter::WriteConfigs() {
  for (auto module : {modules_.front_left, modules_.front_right,
                      modules_.back_left, modules_.back_right}) {
    module->rotation->WriteConfigs();
    module->translation->WriteConfigs();
  }
}

void DrivetrainWriter::Write(
    const ::frc971::control_loops::swerve::Output &output) {
  modules_.front_left->WriteModule(output.front_left_output(), max_voltage_);
  modules_.front_right->WriteModule(output.front_right_output(), max_voltage_);
  modules_.back_left->WriteModule(output.back_left_output(), max_voltage_);
  modules_.back_right->WriteModule(output.back_right_output(), max_voltage_);
}

void DrivetrainWriter::Stop() {
  AOS_LOG(WARNING, "drivetrain output too old\n");

  for (auto module : {modules_.front_left, modules_.front_right,
                      modules_.back_left, modules_.back_right}) {
    module->rotation->WriteCurrent(0, 0);
    module->translation->WriteCurrent(0, 0);
  }
}
