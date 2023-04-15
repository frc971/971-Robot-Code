#include "y2023/vision/camera_monitor_lib.h"
namespace y2023::vision {
namespace {
// This needs to include the amount of time that it will take for the
// camera_reader to start.
constexpr std::chrono::seconds kImageTimeout{5};
}  // namespace
CameraMonitor::CameraMonitor(aos::EventLoop *event_loop)
    : event_loop_(event_loop), starter_(event_loop_) {
  event_loop_->MakeNoArgWatcher<frc971::vision::CameraImage>(
      "/camera", [this]() { SetImageTimeout(); });
  starter_.SetTimeoutHandler([this]() {
    LOG(WARNING) << "Failed to restart camera_reader when images timed out.";
    SetImageTimeout();
  });
  starter_.SetSuccessHandler([this]() {
    LOG(INFO) << "Finished restarting camera_reader.";
    SetImageTimeout();
  });

  image_timeout_ = event_loop_->AddTimer([this]() {
    LOG(INFO) << "Restarting camera_reader due to stale images.";
    starter_.SendCommands({{aos::starter::Command::RESTART,
                            "camera_reader",
                            {event_loop_->node()}}},
                          /*timeout=*/std::chrono::seconds(3));
  });
  // If for some reason camera_reader fails to start up at all, we want to
  // end up restarting things.
  event_loop_->OnRun([this]() { SetImageTimeout(); });
}

void CameraMonitor::SetImageTimeout() {
  image_timeout_->Setup(event_loop_->context().monotonic_event_time +
                        kImageTimeout);
}
}  // namespace y2023::vision
