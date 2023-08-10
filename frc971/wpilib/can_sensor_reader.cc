#include "frc971/wpilib/can_sensor_reader.h"

using frc971::wpilib::CANSensorReader;
using frc971::wpilib::kCANUpdateFreqHz;

CANSensorReader::CANSensorReader(
    aos::EventLoop *event_loop,
    std::vector<ctre::phoenix6::BaseStatusSignal *> signals_registry,
    std::vector<std::shared_ptr<Falcon>> falcons)
    : event_loop_(event_loop),
      signals_(signals_registry.begin(), signals_registry.end()),
      can_position_sender_(
          event_loop->MakeSender<control_loops::drivetrain::CANPosition>(
              "/drivetrain")),
      falcons_(falcons) {
  event_loop->SetRuntimeRealtimePriority(40);

  // TODO(max): Decide if we want to keep this on this core.
  event_loop->SetRuntimeAffinity(aos::MakeCpusetFromCpus({1}));
  timer_handler_ = event_loop->AddTimer([this]() { Loop(); });
  timer_handler_->set_name("CANSensorReader Loop");

  event_loop->OnRun([this]() {
    timer_handler_->Schedule(event_loop_->monotonic_now(),
                             1 / kCANUpdateFreqHz);
  });
}

void CANSensorReader::Loop() {
  ctre::phoenix::StatusCode status =
      ctre::phoenix6::BaseStatusSignal::WaitForAll(20_ms, signals_);

  if (!status.IsOK()) {
    AOS_LOG(ERROR, "Failed to read signals from falcons: %s: %s",
            status.GetName(), status.GetDescription());
  }

  auto builder = can_position_sender_.MakeBuilder();

  for (auto falcon : falcons_) {
    falcon->RefreshNontimesyncedSignals();
    falcon->SerializePosition(builder.fbb());
  }

  auto falcon_offsets =
      builder.fbb()
          ->CreateVector<flatbuffers::Offset<control_loops::CANFalcon>>(
              falcons_.size(), [this](size_t index) {
                auto offset = falcons_.at(index)->TakeOffset();
                CHECK(offset.has_value());
                return offset.value();
              });

  control_loops::drivetrain::CANPosition::Builder can_position_builder =
      builder.MakeBuilder<control_loops::drivetrain::CANPosition>();

  can_position_builder.add_falcons(falcon_offsets);
  if (!falcons_.empty()) {
    can_position_builder.add_timestamp(falcons_.at(0)->GetTimestamp());
  }
  can_position_builder.add_status(static_cast<int>(status));

  builder.CheckOk(builder.Send(can_position_builder.Finish()));
}
