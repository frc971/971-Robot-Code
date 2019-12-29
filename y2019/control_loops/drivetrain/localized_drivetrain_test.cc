#include <queue>

#include "gtest/gtest.h"

#include "aos/controls/control_loop_test.h"
#include "aos/network/team_number.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "frc971/control_loops/drivetrain/drivetrain_test_lib.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "y2019/control_loops/drivetrain/camera_generated.h"
#include "y2019/control_loops/drivetrain/drivetrain_base.h"
#include "y2019/control_loops/drivetrain/event_loop_localizer.h"

// This file tests that the full Localizer, when used with queues within the
// drivetrain, will behave properly. The purpose of this test is to make sure
// that all the pieces fit together, not to test the algorithms themselves.

namespace y2019 {
namespace control_loops {
namespace drivetrain {
namespace testing {

using frc971::control_loops::drivetrain::DrivetrainConfig;
using frc971::control_loops::drivetrain::Goal;
using frc971::control_loops::drivetrain::LocalizerControl;

namespace {
DrivetrainConfig<double> GetTest2019DrivetrainConfig() {
  DrivetrainConfig<double> config = GetDrivetrainConfig();
  config.gyro_type =
      ::frc971::control_loops::drivetrain::GyroType::SPARTAN_GYRO;
  return config;
}
};  // namespace

namespace chrono = ::std::chrono;
using ::frc971::control_loops::drivetrain::testing::DrivetrainSimulation;
using ::frc971::control_loops::drivetrain::DrivetrainLoop;
using ::frc971::control_loops::drivetrain::testing::GetTestDrivetrainConfig;
using ::aos::monotonic_clock;

class LocalizedDrivetrainTest : public ::aos::testing::ControlLoopTest {
 protected:
  // We must use the 2019 drivetrain config so that we don't have to deal
  // with shifting:
  LocalizedDrivetrainTest()
      : ::aos::testing::ControlLoopTest(
            aos::configuration::ReadConfig("y2019/config.json"),
            GetTest2019DrivetrainConfig().dt),
        test_event_loop_(MakeEventLoop("test")),
        drivetrain_goal_sender_(
            test_event_loop_->MakeSender<Goal>("/drivetrain")),
        drivetrain_goal_fetcher_(
            test_event_loop_->MakeFetcher<Goal>("/drivetrain")),
        localizer_control_sender_(
            test_event_loop_->MakeSender<LocalizerControl>("/drivetrain")),
        drivetrain_event_loop_(MakeEventLoop("drivetrain")),
        dt_config_(GetTest2019DrivetrainConfig()),
        camera_sender_(
            test_event_loop_->MakeSender<CameraFrame>("/camera")),
        localizer_(drivetrain_event_loop_.get(), dt_config_),
        drivetrain_(dt_config_, drivetrain_event_loop_.get(), &localizer_),
        drivetrain_plant_event_loop_(MakeEventLoop("plant")),
        drivetrain_plant_(drivetrain_plant_event_loop_.get(), dt_config_),
        cameras_(MakeCameras(&robot_pose_)),
        last_frame_(monotonic_now()) {
    set_team_id(::frc971::control_loops::testing::kTeamNumber);
    // Because 0, 0 is on the top level of the Hab and right on the edge of an
    // obstacle, it ends up confusing the localizer; as such, starting there
    // is just prone to causing confusion.
    SetStartingPosition({3.0, 2.0, 0.0});
    set_battery_voltage(12.0);

    test_event_loop_->MakeWatcher(
        "/drivetrain",
        [this](const ::frc971::control_loops::drivetrain::Status &) {
          // Needs to do camera updates right after we run the control loop.
          if (enable_cameras_) {
            SendDelayedFrames();
            if (last_frame_ + ::std::chrono::milliseconds(33) <
                monotonic_now()) {
              CaptureFrames();
              last_frame_ = monotonic_now();
            }
          }
        });
  }

  virtual ~LocalizedDrivetrainTest() {}

  void SetStartingPosition(const Eigen::Matrix<double, 3, 1> &xytheta) {
    *drivetrain_plant_.mutable_state() << xytheta.x(), xytheta.y(),
        xytheta(2, 0), 0.0, 0.0;
    Eigen::Matrix<double, EventLoopLocalizer::Localizer::kNStates, 1>
        localizer_state;
    localizer_state.setZero();
    localizer_state.block<3, 1>(0, 0) = xytheta;
    localizer_.Reset(monotonic_now(), localizer_state, 0.0);
  }

  void VerifyNearGoal() {
    drivetrain_goal_fetcher_.Fetch();
    EXPECT_NEAR(drivetrain_goal_fetcher_->left_goal(),
                drivetrain_plant_.GetLeftPosition(), 1e-3);
    EXPECT_NEAR(drivetrain_goal_fetcher_->right_goal(),
                drivetrain_plant_.GetRightPosition(), 1e-3);
  }

  void VerifyEstimatorAccurate(double eps) {
    const Eigen::Matrix<double, 5, 1> true_state = drivetrain_plant_.state();
    EXPECT_NEAR(localizer_.x(), true_state(0, 0), eps);
    EXPECT_NEAR(localizer_.y(), true_state(1, 0), eps);
    EXPECT_NEAR(localizer_.theta(), true_state(2, 0), eps);
    EXPECT_NEAR(localizer_.left_velocity(), true_state(3, 0), eps);
    EXPECT_NEAR(localizer_.right_velocity(), true_state(4, 0), eps);
  }

  void CaptureFrames() {
    *robot_pose_.mutable_pos() << drivetrain_plant_.GetPosition().x(),
        drivetrain_plant_.GetPosition().y(), 0.0;
    robot_pose_.set_theta(drivetrain_plant_.state()(2, 0));
    for (size_t ii = 0; ii < cameras_.size(); ++ii) {
      const auto target_views = cameras_[ii].target_views();
      std::unique_ptr<CameraFrameT> frame(new CameraFrameT());

      for (size_t jj = 0;
           jj < ::std::min(EventLoopLocalizer::kMaxTargetsPerFrame,
                           target_views.size());
           ++jj) {
        EventLoopLocalizer::TargetView view = target_views[jj];
        std::unique_ptr<CameraTargetT> camera_target(new CameraTargetT());

        const float nan = ::std::numeric_limits<float>::quiet_NaN();
        if (send_bad_frames_) {
          camera_target->heading = nan;
          camera_target->distance = nan;
          camera_target->skew = nan;
          camera_target->height = nan;
        } else {
          camera_target->heading = view.reading.heading;
          camera_target->distance = view.reading.distance;
          camera_target->skew = view.reading.skew;
          camera_target->height = view.reading.height;
        }
        frame->targets.emplace_back(std::move(camera_target));
      }

      frame->timestamp = chrono::duration_cast<chrono::nanoseconds>(
                             monotonic_now().time_since_epoch())
                             .count();
      frame->camera = ii;

      camera_delay_queue_.emplace(monotonic_now(), std::move(frame));
    }
  }

  void SendDelayedFrames() {
    const ::std::chrono::milliseconds camera_latency(50);
    while (!camera_delay_queue_.empty() &&
           ::std::get<0>(camera_delay_queue_.front()) <
               monotonic_now() - camera_latency) {
      auto builder = camera_sender_.MakeBuilder();
      ASSERT_TRUE(builder.Send(CameraFrame::Pack(
          *builder.fbb(), ::std::get<1>(camera_delay_queue_.front()).get())));
      camera_delay_queue_.pop();
    }
  }

  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;
  ::aos::Sender<Goal> drivetrain_goal_sender_;
  ::aos::Fetcher<Goal> drivetrain_goal_fetcher_;
  ::aos::Sender<LocalizerControl> localizer_control_sender_;

  ::std::unique_ptr<::aos::EventLoop> drivetrain_event_loop_;
  const ::frc971::control_loops::drivetrain::DrivetrainConfig<double>
      dt_config_;

  ::aos::Sender<CameraFrame> camera_sender_;

  EventLoopLocalizer localizer_;
  DrivetrainLoop drivetrain_;

  ::std::unique_ptr<::aos::EventLoop> drivetrain_plant_event_loop_;
  DrivetrainSimulation drivetrain_plant_;
  EventLoopLocalizer::Pose robot_pose_;
  const ::std::array<EventLoopLocalizer::Camera, constants::Values::kNumCameras>
      cameras_;
  monotonic_clock::time_point last_frame_;

  // A queue of camera frames so that we can add a time delay to the data
  // coming from the cameras.
  ::std::queue<::std::tuple<::aos::monotonic_clock::time_point,
                            std::unique_ptr<CameraFrameT>>>
      camera_delay_queue_;

  void set_enable_cameras(bool enable) { enable_cameras_ = enable; }
  void set_bad_frames(bool enable) { send_bad_frames_ = enable; }

 private:
  bool enable_cameras_ = false;
  bool send_bad_frames_ = false;
};

// Tests that no camera updates, combined with a perfect model, results in no
// error.
TEST_F(LocalizedDrivetrainTest, NoCameraUpdate) {
  SetEnabled(true);
  set_enable_cameras(false);
  VerifyEstimatorAccurate(1e-10);
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();

    Goal::Builder drivetrain_builder = builder.MakeBuilder<Goal>();
    drivetrain_builder.add_controller_type(
        frc971::control_loops::drivetrain::ControllerType::MOTION_PROFILE);
    drivetrain_builder.add_left_goal(-1.0);
    drivetrain_builder.add_right_goal(1.0);

    EXPECT_TRUE(builder.Send(drivetrain_builder.Finish()));
  }
  RunFor(chrono::seconds(3));
  VerifyNearGoal();
  VerifyEstimatorAccurate(1e-10);
}

// Bad camera updates (NaNs) should have no effect.
TEST_F(LocalizedDrivetrainTest, BadCameraUpdate) {
  SetEnabled(true);
  set_enable_cameras(true);
  set_bad_frames(true);
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();

    Goal::Builder drivetrain_builder = builder.MakeBuilder<Goal>();
    drivetrain_builder.add_controller_type(
        frc971::control_loops::drivetrain::ControllerType::MOTION_PROFILE);
    drivetrain_builder.add_left_goal(-1.0);
    drivetrain_builder.add_right_goal(1.0);

    EXPECT_TRUE(builder.Send(drivetrain_builder.Finish()));
  }
  RunFor(chrono::seconds(3));
  VerifyNearGoal();
  VerifyEstimatorAccurate(1e-10);
}

// Tests that camera udpates with a perfect models results in no errors.
TEST_F(LocalizedDrivetrainTest, PerfectCameraUpdate) {
  SetEnabled(true);
  set_enable_cameras(true);
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();

    Goal::Builder drivetrain_builder = builder.MakeBuilder<Goal>();
    drivetrain_builder.add_controller_type(
        frc971::control_loops::drivetrain::ControllerType::MOTION_PROFILE);
    drivetrain_builder.add_left_goal(-1.0);
    drivetrain_builder.add_right_goal(1.0);

    EXPECT_TRUE(builder.Send(drivetrain_builder.Finish()));
  }
  RunFor(chrono::seconds(3));
  VerifyNearGoal();
  VerifyEstimatorAccurate(1e-7);
}

// Tests that not having cameras with an initial disturbance results in
// estimator error. This is to test the test.
TEST_F(LocalizedDrivetrainTest, NoCameraWithDisturbanceFails) {
  SetEnabled(true);
  set_enable_cameras(false);
  (*drivetrain_plant_.mutable_state())(0, 0) += 0.05;
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();

    Goal::Builder drivetrain_builder = builder.MakeBuilder<Goal>();
    drivetrain_builder.add_controller_type(
        frc971::control_loops::drivetrain::ControllerType::MOTION_PROFILE);
    drivetrain_builder.add_left_goal(-1.0);
    drivetrain_builder.add_right_goal(1.0);

    EXPECT_TRUE(builder.Send(drivetrain_builder.Finish()));
  }
  RunFor(chrono::seconds(3));
  // VerifyNearGoal succeeds because it is just checking wheel positions:
  VerifyNearGoal();
  const Eigen::Matrix<double, 5, 1> true_state = drivetrain_plant_.state();
  // Everything but X-value should be correct:
  EXPECT_NEAR(true_state.x(), localizer_.x() + 0.05, 1e-5);
  EXPECT_NEAR(true_state.y(), localizer_.y(), 1e-5);
  EXPECT_NEAR(true_state(2, 0), localizer_.theta(), 1e-5);
  EXPECT_NEAR(true_state(3, 0), localizer_.left_velocity(), 1e-5);
  EXPECT_NEAR(true_state(4, 0), localizer_.right_velocity(), 1e-5);
}

// Tests that when we reset the position of the localizer via the queue to
// compensate for the disturbance, the estimator behaves perfectly.
TEST_F(LocalizedDrivetrainTest, ResetLocalizer) {
  SetEnabled(true);
  set_enable_cameras(false);
  (*drivetrain_plant_.mutable_state())(0, 0) += 0.05;
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();

    Goal::Builder drivetrain_builder = builder.MakeBuilder<Goal>();
    drivetrain_builder.add_controller_type(
        frc971::control_loops::drivetrain::ControllerType::MOTION_PROFILE);
    drivetrain_builder.add_left_goal(-1.0);
    drivetrain_builder.add_right_goal(1.0);

    EXPECT_TRUE(builder.Send(drivetrain_builder.Finish()));
  }

  {
    auto builder = localizer_control_sender_.MakeBuilder();

    LocalizerControl::Builder localizer_control_builder =
        builder.MakeBuilder<LocalizerControl>();

    localizer_control_builder.add_x(drivetrain_plant_.state().x());
    localizer_control_builder.add_y(drivetrain_plant_.state().y());
    localizer_control_builder.add_theta(drivetrain_plant_.state()(2, 0));

    EXPECT_TRUE(builder.Send(localizer_control_builder.Finish()));
  }
  RunFor(chrono::seconds(3));
  VerifyNearGoal();
  VerifyEstimatorAccurate(1e-5);
}

// Tests that, when a small error in X is introduced, the camera corrections do
// correct for it.
TEST_F(LocalizedDrivetrainTest, CameraUpdate) {
  SetEnabled(true);
  set_enable_cameras(true);
  SetStartingPosition({4.0, 0.5, 0.0});
  (*drivetrain_plant_.mutable_state())(0, 0) += 0.05;
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();

    Goal::Builder drivetrain_builder = builder.MakeBuilder<Goal>();
    drivetrain_builder.add_controller_type(
        frc971::control_loops::drivetrain::ControllerType::MOTION_PROFILE);
    drivetrain_builder.add_left_goal(-1.0);
    drivetrain_builder.add_right_goal(1.0);

    EXPECT_TRUE(builder.Send(drivetrain_builder.Finish()));
  }
  RunFor(chrono::seconds(5));
  VerifyNearGoal();
  VerifyEstimatorAccurate(5e-3);
}

namespace {
EventLoopLocalizer::Pose HPSlotLeft() {
  return constants::Field().targets()[7].pose();
}
}  // namespace

// Tests that using the line following drivetrain and just driving straight
// forward from roughly the right spot gets us to the HP slot.
TEST_F(LocalizedDrivetrainTest, LineFollowToHPSlot) {
  SetEnabled(true);
  set_enable_cameras(false);
  SetStartingPosition({4, HPSlotLeft().abs_pos().y(), M_PI});
  {
    auto builder = drivetrain_goal_sender_.MakeBuilder();

    Goal::Builder drivetrain_builder = builder.MakeBuilder<Goal>();
    drivetrain_builder.add_controller_type(
        frc971::control_loops::drivetrain::ControllerType::LINE_FOLLOWER);
    drivetrain_builder.add_throttle(0.5);

    EXPECT_TRUE(builder.Send(drivetrain_builder.Finish()));
  }
  RunFor(chrono::seconds(6));

  VerifyEstimatorAccurate(1e-8);
  // Due to the fact that we aren't modulating the throttle, we don't try to hit
  // the target exactly. Instead, just run slightly past the target:
  EXPECT_LT(
      ::std::abs(::aos::math::DiffAngle(M_PI, drivetrain_plant_.state()(2, 0))),
      1e-5);
  EXPECT_GT(HPSlotLeft().abs_pos().x(), drivetrain_plant_.state().x());
  EXPECT_NEAR(HPSlotLeft().abs_pos().y(), drivetrain_plant_.state().y(), 1e-4);
}

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2019
