#ifndef Y2019_CONTROL_LOOPS_DRIVETRAIN_EVENT_LOOP_LOCALIZER_H_
#define Y2019_CONTROL_LOOPS_DRIVETRAIN_EVENT_LOOP_LOCALIZER_H_

#include "frc971/control_loops/drivetrain/localizer.h"
#include "y2019/constants.h"
#include "y2019/control_loops/drivetrain/camera.q.h"
#include "y2019/control_loops/drivetrain/localizer.h"
#include "y2019/control_loops/drivetrain/target_selector.h"

namespace y2019 {
namespace control_loops {
namespace drivetrain {

// Wrap the localizer to allow it to fetch camera frames from the queues.
// TODO(james): Provide a way of resetting the current position and
// localizer.
class EventLoopLocalizer
    : public ::frc971::control_loops::drivetrain::LocalizerInterface {
 public:
  static constexpr size_t kMaxTargetsPerFrame = 3;
  typedef TypedLocalizer<
      constants::Values::kNumCameras, y2019::constants::Field::kNumTargets,
      y2019::constants::Field::kNumObstacles, kMaxTargetsPerFrame, double>
      Localizer;
  typedef typename Localizer::StateIdx StateIdx;

  typedef typename Localizer::Camera Camera;
  typedef typename Localizer::TargetView TargetView;
  typedef typename Localizer::Pose Pose;

  EventLoopLocalizer(
      const ::frc971::control_loops::drivetrain::DrivetrainConfig<double>
          &dt_config,
      ::aos::EventLoop *event_loop);

  void Reset(const Localizer::State &state);
  void ResetPosition(double x, double y, double theta) override {
    Localizer::State new_state = localizer_.X_hat();
    new_state.x() = x;
    new_state.y() = y;
    new_state(2, 0) = theta;
    new_state(4, 0) = 0.0;
    new_state(6, 0) = 0.0;
    new_state(7, 0) = 0.0;
    new_state(8, 0) = 0.0;
    new_state(9, 0) = 0.0;
    Reset(new_state);
  }

  void Update(const ::Eigen::Matrix<double, 2, 1> &U,
              ::aos::monotonic_clock::time_point now, double left_encoder,
              double right_encoder, double gyro_rate,
              double /*longitudinal_accelerometer*/) override;

  double x() const override {
    return localizer_.X_hat(StateIdx::kX); }
  double y() const override {
    return localizer_.X_hat(StateIdx::kY); }
  double theta() const override {
    return localizer_.X_hat(StateIdx::kTheta); }
  double left_velocity() const override {
    return localizer_.X_hat(StateIdx::kLeftVelocity);
  }
  double right_velocity() const override {
    return localizer_.X_hat(StateIdx::kRightVelocity);
  }
  double left_voltage_error() const override {
    return localizer_.X_hat(StateIdx::kLeftVoltageError);
  }
  double right_voltage_error() const override {
    return localizer_.X_hat(StateIdx::kRightVoltageError);
  }

  TargetSelector *target_selector() override {
    return &target_selector_;
  }

 private:
  void HandleFrame(const CameraFrame &frame);

  ::aos::EventLoop *event_loop_;
  // TODO(james): Make this use Watchers once we have them working in our
  // simulation framework.
  ::aos::Fetcher<CameraFrame> frame_fetcher_;

  // Pose used as base for cameras; note that the exact value populated here
  // is not overly important as it will be overridden by the Localizer.
  Pose robot_pose_;
  const ::std::array<Camera, constants::Values::kNumCameras> cameras_;
  Localizer localizer_;
  TargetSelector target_selector_;
};

// Constructs the cameras based on the constants in the //y2019/constants.*
// files and uses the provided Pose as the robot pose for the cameras.
::std::array<EventLoopLocalizer::Camera, constants::Values::kNumCameras>
MakeCameras(EventLoopLocalizer::Pose *pose);

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2019
#endif  // Y2019_CONTROL_LOOPS_DRIVETRAIN_EVENT_LOOP_LOCALIZER_H_
