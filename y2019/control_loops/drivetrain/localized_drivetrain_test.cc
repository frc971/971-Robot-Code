#include <queue>

#include "gtest/gtest.h"

#include "aos/controls/control_loop_test.h"
#include "aos/network/team_number.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "frc971/control_loops/drivetrain/drivetrain_test_lib.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "y2019/control_loops/drivetrain/camera.q.h"
#include "y2019/control_loops/drivetrain/event_loop_localizer.h"
#include "y2019/control_loops/drivetrain/drivetrain_base.h"

// This file tests that the full Localizer, when used with queues within the
// drivetrain, will behave properly. The purpose of this test is to make sure
// that all the pieces fit together, not to test the algorithms themselves.

namespace y2019 {
namespace control_loops {
namespace drivetrain {
namespace testing {

using ::frc971::control_loops::drivetrain::DrivetrainConfig;

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
      : dt_config_(GetTest2019DrivetrainConfig()),
        my_drivetrain_queue_(".frc971.control_loops.drivetrain_queue",
                             ".frc971.control_loops.drivetrain_queue.goal",
                             ".frc971.control_loops.drivetrain_queue.position",
                             ".frc971.control_loops.drivetrain_queue.output",
                             ".frc971.control_loops.drivetrain_queue.status"),
        camera_queue_(::y2019::control_loops::drivetrain::camera_frames.name()),
        localizer_(dt_config_, &event_loop_),
        drivetrain_motor_(dt_config_, &event_loop_, &localizer_),
        drivetrain_motor_plant_(dt_config_),
        cameras_(MakeCameras(&robot_pose_)),
        last_frame_(monotonic_clock::now()) {
    set_team_id(::frc971::control_loops::testing::kTeamNumber);
    // Because 0, 0 is on the top level of the Hab and right on the edge of an
    // obstacle, it ends up confusing the localizer; as such, starting there
    // is just prone to causing confusion.
    SetStartingPosition({3.0, 2.0, 0.0});
    ::frc971::sensors::gyro_reading.Clear();
    ::y2019::control_loops::drivetrain::camera_frames.Clear();
    set_battery_voltage(12.0);
  }

  void SetStartingPosition(const Eigen::Matrix<double, 3, 1> &xytheta) {
    *drivetrain_motor_plant_.mutable_state() << xytheta.x(), xytheta.y(),
        xytheta(2, 0), 0.0, 0.0;
    Eigen::Matrix<double, 10, 1> localizer_state;
    localizer_state.setZero();
    localizer_state.block<3, 1>(0, 0) = xytheta;
    localizer_.Reset(localizer_state);
  }

  void RunIteration() {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
    if (enable_cameras_) {
      SendDelayedFrames();
      if (last_frame_ + ::std::chrono::milliseconds(33) <
          monotonic_clock::now()) {
        CaptureFrames();
        last_frame_ = monotonic_clock::now();
      }
    }
    drivetrain_motor_plant_.Simulate();
    SimulateTimestep(true, dt_config_.dt);
  }

  void RunForTime(monotonic_clock::duration run_for) {
    const auto end_time = monotonic_clock::now() + run_for;
    while (monotonic_clock::now() < end_time) {
      RunIteration();
    }
    // Let the estimator catch up with the final simulation step:
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Iterate();
  }

  void VerifyNearGoal() {
    my_drivetrain_queue_.goal.FetchLatest();
    my_drivetrain_queue_.position.FetchLatest();
    EXPECT_NEAR(my_drivetrain_queue_.goal->left_goal,
                drivetrain_motor_plant_.GetLeftPosition(), 1e-3);
    EXPECT_NEAR(my_drivetrain_queue_.goal->right_goal,
                drivetrain_motor_plant_.GetRightPosition(), 1e-3);
  }

  void VerifyEstimatorAccurate(double eps) {
    const Eigen::Matrix<double, 5, 1> true_state =
        drivetrain_motor_plant_.state();
    EXPECT_NEAR(localizer_.x(), true_state(0, 0), eps);
    EXPECT_NEAR(localizer_.y(), true_state(1, 0), eps);
    EXPECT_NEAR(localizer_.theta(), true_state(2, 0), eps);
    EXPECT_NEAR(localizer_.left_velocity(), true_state(3, 0), eps);
    EXPECT_NEAR(localizer_.right_velocity(), true_state(4, 0), eps);
  }

  void CaptureFrames() {
    *robot_pose_.mutable_pos() << drivetrain_motor_plant_.GetPosition().x(),
        drivetrain_motor_plant_.GetPosition().y(), 0.0;
    robot_pose_.set_theta(drivetrain_motor_plant_.state()(2, 0));
    for (size_t ii = 0; ii < cameras_.size(); ++ii) {
      const auto target_views = cameras_[ii].target_views();
      CameraFrame frame;
      frame.timestamp = chrono::duration_cast<chrono::nanoseconds>(
                            monotonic_clock::now().time_since_epoch())
                            .count();
      frame.camera = ii;
      frame.num_targets = 0;
      for (size_t jj = 0;
           jj < ::std::min(EventLoopLocalizer::kMaxTargetsPerFrame,
                           target_views.size());
           ++jj) {
        EventLoopLocalizer::TargetView view = target_views[jj];
        ++frame.num_targets;
        frame.targets[jj].heading = view.reading.heading;
        frame.targets[jj].distance = view.reading.distance;
        frame.targets[jj].skew = view.reading.skew;
        frame.targets[jj].height = view.reading.height;
      }
      camera_delay_queue_.emplace(monotonic_clock::now(), frame);
    }
  }

  void SendDelayedFrames() {
    const ::std::chrono::milliseconds camera_latency(50);
    while (!camera_delay_queue_.empty() &&
           ::std::get<0>(camera_delay_queue_.front()) <
               monotonic_clock::now() - camera_latency) {
      ::aos::ScopedMessagePtr<CameraFrame> message =
          camera_queue_.MakeMessage();
      *message = ::std::get<1>(camera_delay_queue_.front());
      ASSERT_TRUE(message.Send());
      camera_delay_queue_.pop();
    }
  }

  virtual ~LocalizedDrivetrainTest() {
    ::frc971::sensors::gyro_reading.Clear();
    ::y2019::control_loops::drivetrain::camera_frames.Clear();
  }

  const ::frc971::control_loops::drivetrain::DrivetrainConfig<double>
      dt_config_;

  ::frc971::control_loops::DrivetrainQueue my_drivetrain_queue_;
  ::aos::Queue<CameraFrame> camera_queue_;
  ::aos::ShmEventLoop event_loop_;

  EventLoopLocalizer localizer_;
  DrivetrainLoop drivetrain_motor_;
  DrivetrainSimulation drivetrain_motor_plant_;
  EventLoopLocalizer::Pose robot_pose_;
  const ::std::array<EventLoopLocalizer::Camera, constants::Values::kNumCameras>
      cameras_;
  monotonic_clock::time_point last_frame_;

  // A queue of camera frames so that we can add a time delay to the data
  // coming from the cameras.
  ::std::queue<::std::tuple<::aos::monotonic_clock::time_point, CameraFrame>>
      camera_delay_queue_;

  void set_enable_cameras(bool enable) { enable_cameras_ = enable; }

 private:
  bool enable_cameras_ = false;
};

// Tests that no camera updates, combined with a perfect model, results in no
// error.
TEST_F(LocalizedDrivetrainTest, NoCameraUpdate) {
  set_enable_cameras(false);
  my_drivetrain_queue_.goal.MakeWithBuilder()
      .controller_type(1)
      .left_goal(-1.0)
      .right_goal(1.0)
      .Send();
  RunForTime(chrono::seconds(3));
  VerifyNearGoal();
  VerifyEstimatorAccurate(1e-10);
}

// Tests that camera udpates with a perfect models results in no errors.
TEST_F(LocalizedDrivetrainTest, PerfectCameraUpdate) {
  set_enable_cameras(true);
  my_drivetrain_queue_.goal.MakeWithBuilder()
      .controller_type(1)
      .left_goal(-1.0)
      .right_goal(1.0)
      .Send();
  RunForTime(chrono::seconds(3));
  VerifyNearGoal();
  VerifyEstimatorAccurate(1e-7);
}

// Tests that not having cameras with an initial disturbance results in
// estimator error. This is to test the test.
TEST_F(LocalizedDrivetrainTest, NoCameraWithDisturbanceFails) {
  set_enable_cameras(false);
  (*drivetrain_motor_plant_.mutable_state())(0, 0) += 0.05;
  my_drivetrain_queue_.goal.MakeWithBuilder()
      .controller_type(1)
      .left_goal(-1.0)
      .right_goal(1.0)
      .Send();
  RunForTime(chrono::seconds(3));
  // VerifyNearGoal succeeds because it is just checking wheel positions:
  VerifyNearGoal();
  const Eigen::Matrix<double, 5, 1> true_state =
      drivetrain_motor_plant_.state();
  // Everything but X-value should be correct:
  EXPECT_NEAR(true_state.x(), localizer_.x() + 0.05, 1e-5);
  EXPECT_NEAR(true_state.y(), localizer_.y(), 1e-5);
  EXPECT_NEAR(true_state(2, 0), localizer_.theta(), 1e-5);
  EXPECT_NEAR(true_state(3, 0), localizer_.left_velocity(), 1e-5);
  EXPECT_NEAR(true_state(4, 0), localizer_.right_velocity(), 1e-5);
}

// Tests that, when a small error in X is introduced, the camera corrections do
// correct for it.
TEST_F(LocalizedDrivetrainTest, CameraUpdate) {
  set_enable_cameras(true);
  (*drivetrain_motor_plant_.mutable_state())(0, 0) += 0.05;
  my_drivetrain_queue_.goal.MakeWithBuilder()
      .controller_type(1)
      .left_goal(-1.0)
      .right_goal(1.0)
      .Send();
  RunForTime(chrono::seconds(3));
  VerifyNearGoal();
  VerifyEstimatorAccurate(1e-5);
}

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2019
