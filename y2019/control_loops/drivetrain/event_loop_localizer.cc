#include "y2019/control_loops/drivetrain/event_loop_localizer.h"

#include <functional>

namespace y2019 {
namespace control_loops {
namespace drivetrain {
constexpr size_t EventLoopLocalizer::kMaxTargetsPerFrame;

::std::array<EventLoopLocalizer::Camera, constants::Values::kNumCameras>
MakeCameras(EventLoopLocalizer::Pose *pose) {
  constants::Field field;
  ::std::array<EventLoopLocalizer::Camera, constants::Values::kNumCameras>
      cameras;
  for (size_t ii = 0; ii < constants::Values::kNumCameras; ++ii) {
    constants::Values::CameraCalibration camera_info =
        constants::GetValues().cameras[ii];
    EventLoopLocalizer::Pose camera_pose = camera_info.pose.Rebase(pose);
    cameras[ii] = EventLoopLocalizer::Camera(
        camera_pose, camera_info.fov,
        constants::GetValues().camera_noise_parameters, field.targets(),
        field.obstacles());
  }
  return cameras;
}

EventLoopLocalizer::EventLoopLocalizer(
    ::aos::EventLoop *event_loop,
    const ::frc971::control_loops::drivetrain::DrivetrainConfig<double>
        &dt_config)
    : event_loop_(event_loop),
      cameras_(MakeCameras(&robot_pose_)),
      localizer_(dt_config, &robot_pose_),
      target_selector_(event_loop) {
  const ::aos::monotonic_clock::time_point monotonic_now =
      event_loop_->monotonic_now();
  localizer_.ResetInitialState(monotonic_now, Localizer::State::Zero(),
                               localizer_.P());
  ResetPosition(monotonic_now, 0.5, 3.4, 0.0, 0.0, true);
  event_loop->OnRun([this]() {
    // Reset time now that we have an official start time.  Reset the states to
    // their current state since nothing will have moved.
    const ::aos::monotonic_clock::time_point monotonic_now =
        event_loop_->monotonic_now();
    localizer_.ResetInitialState(monotonic_now, localizer_.X_hat(),
                                 localizer_.P());
  });
  frame_fetcher_ = event_loop_->MakeFetcher<CameraFrame>("/camera");
}

void EventLoopLocalizer::Reset(::aos::monotonic_clock::time_point now,
                               const Localizer::State &state,
                               double theta_uncertainty) {
  Localizer::StateSquare newP = localizer_.P();
  if (theta_uncertainty > 0.0) {
    newP(StateIdx::kTheta, StateIdx::kTheta) = theta_uncertainty;
  }
  localizer_.ResetInitialState(now, state, newP);
}

void EventLoopLocalizer::Update(
    const ::Eigen::Matrix<double, 2, 1> &U,
    ::aos::monotonic_clock::time_point now, double left_encoder,
    double right_encoder, double gyro_rate,
    double /*longitudinal_accelerometer*/) {
  AOS_CHECK(U.allFinite());
  AOS_CHECK(::std::isfinite(left_encoder));
  AOS_CHECK(::std::isfinite(right_encoder));
  AOS_CHECK(::std::isfinite(gyro_rate));
  localizer_.UpdateEncodersAndGyro(left_encoder, right_encoder, gyro_rate, U,
                                   now);
  while (frame_fetcher_.FetchNext()) {
    HandleFrame(frame_fetcher_.get());
  }
}

void EventLoopLocalizer::HandleFrame(const CameraFrame *frame) {
  // We need to construct TargetView's and pass them to the localizer:
  ::aos::SizedArray<TargetView, kMaxTargetsPerFrame> views;
  // Note: num_targets and camera are unsigned integers and so don't need to be
  // checked for < 0.
  size_t camera = frame->camera();
  if (!frame->has_targets() || frame->targets()->size() > kMaxTargetsPerFrame) {
    AOS_LOG(ERROR, "Got bad num_targets %d\n",
            frame->has_targets() ? frame->targets()->size() : 0);
    return;
  }
  if (camera > cameras_.size()) {
    AOS_LOG(ERROR, "Got bad camera number %zu\n", camera);
    return;
  }
  for (const CameraTarget *target : *frame->targets()) {
    TargetView view;
    view.reading.heading = target->heading();
    view.reading.distance = target->distance();
    view.reading.skew = target->skew();
    view.reading.height = target->height();
    if (view.reading.distance < 2.25) {
      cameras_[camera].PopulateNoise(&view);
      views.push_back(view);
    }
  }
  ::aos::monotonic_clock::time_point t(
      ::std::chrono::nanoseconds(frame->timestamp()));
  localizer_.UpdateTargets(cameras_[camera], views, t);
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2019
