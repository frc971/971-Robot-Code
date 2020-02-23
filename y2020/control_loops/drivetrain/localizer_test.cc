#include <queue>

#include "gtest/gtest.h"

#include "aos/controls/control_loop_test.h"
#include "aos/events/logging/logger.h"
#include "aos/network/team_number.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "frc971/control_loops/drivetrain/drivetrain_test_lib.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "y2020/control_loops/drivetrain/drivetrain_base.h"
#include "y2020/control_loops/drivetrain/localizer.h"

DEFINE_string(output_file, "",
              "If set, logs all channels to the provided logfile.");

// This file tests that the full 2020 localizer behaves sanely.

namespace y2020 {
namespace control_loops {
namespace drivetrain {
namespace testing {

using frc971::control_loops::drivetrain::DrivetrainConfig;
using frc971::control_loops::drivetrain::Goal;
using frc971::control_loops::drivetrain::LocalizerControl;
using frc971::vision::sift::ImageMatchResult;
using frc971::vision::sift::ImageMatchResultT;
using frc971::vision::sift::CameraPoseT;
using frc971::vision::sift::CameraCalibrationT;
using frc971::vision::sift::TransformationMatrixT;

namespace {
DrivetrainConfig<double> GetTest2020DrivetrainConfig() {
  DrivetrainConfig<double> config = GetDrivetrainConfig();
  return config;
}

// Copies an Eigen matrix into a row-major vector of the data.
std::vector<float> MatrixToVector(const Eigen::Matrix<double, 4, 4> &H) {
  std::vector<float> data;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      data.push_back(H(row, col));
    }
  }
  return data;
}

// Provides the location of the turret to use for simulation. Mostly we care
// about providing a location that is not perfectly aligned with the robot's
// origin.
Eigen::Matrix<double, 4, 4> TurretRobotTransformation() {
  Eigen::Matrix<double, 4, 4> H;
  H.setIdentity();
  H.block<3, 1>(0, 3) << 1, 1, 1;
  return H;
}

// Provides the location of the camera on the turret.
// TODO(james): Also simulate a fixed camera that is *not* on the turret.
Eigen::Matrix<double, 4, 4> CameraTurretTransformation() {
  Eigen::Matrix<double, 4, 4> H;
  H.setIdentity();
  H.block<3, 1>(0, 3) << 0.1, 0, 0;
  // Introduce a bit of pitch to make sure that we're exercising all the code.
  H.block<3, 3>(0, 0) =
      Eigen::AngleAxis<double>(M_PI_2, Eigen::Vector3d::UnitY()) *
      H.block<3, 3>(0, 0);
  return H;
}

// The absolute target location to use. Not meant to correspond with a
// particular field target.
// TODO(james): Make more targets.
Eigen::Matrix<double, 4, 4> TargetLocation() {
  Eigen::Matrix<double, 4, 4> H;
  H.setIdentity();
  H.block<3, 1>(0, 3) << 10.0, 0, 0;
  return H;
}
}  // namespace

namespace chrono = std::chrono;
using frc971::control_loops::drivetrain::testing::DrivetrainSimulation;
using frc971::control_loops::drivetrain::DrivetrainLoop;
using frc971::control_loops::drivetrain::testing::GetTestDrivetrainConfig;
using aos::monotonic_clock;

class LocalizedDrivetrainTest : public ::aos::testing::ControlLoopTest {
 protected:
  // We must use the 2020 drivetrain config so that we don't have to deal
  // with shifting:
  LocalizedDrivetrainTest()
      : aos::testing::ControlLoopTest(
            aos::configuration::ReadConfig(
                "y2020/control_loops/drivetrain/simulation_config.json"),
            GetTest2020DrivetrainConfig().dt),
        test_event_loop_(MakeEventLoop("test")),
        drivetrain_goal_sender_(
            test_event_loop_->MakeSender<Goal>("/drivetrain")),
        drivetrain_goal_fetcher_(
            test_event_loop_->MakeFetcher<Goal>("/drivetrain")),
        localizer_control_sender_(
            test_event_loop_->MakeSender<LocalizerControl>("/drivetrain")),
        drivetrain_event_loop_(MakeEventLoop("drivetrain")),
        dt_config_(GetTest2020DrivetrainConfig()),
        camera_sender_(
            test_event_loop_->MakeSender<ImageMatchResult>("/camera")),
        localizer_(drivetrain_event_loop_.get(), dt_config_),
        drivetrain_(dt_config_, drivetrain_event_loop_.get(), &localizer_),
        drivetrain_plant_event_loop_(MakeEventLoop("plant")),
        drivetrain_plant_(drivetrain_plant_event_loop_.get(), dt_config_),
        last_frame_(monotonic_now()) {
    set_team_id(frc971::control_loops::testing::kTeamNumber);
    SetStartingPosition({3.0, 2.0, 0.0});
    set_battery_voltage(12.0);

    if (!FLAGS_output_file.empty()) {
      log_buffer_writer_ = std::make_unique<aos::logger::DetachedBufferWriter>(
          FLAGS_output_file);
      logger_event_loop_ = MakeEventLoop("logger");
      logger_ = std::make_unique<aos::logger::Logger>(log_buffer_writer_.get(),
                                                      logger_event_loop_.get());
    }

    test_event_loop_->MakeWatcher(
        "/drivetrain",
        [this](const frc971::control_loops::drivetrain::Status &) {
          // Needs to do camera updates right after we run the control loop.
          if (enable_cameras_) {
            SendDelayedFrames();
            if (last_frame_ + std::chrono::milliseconds(100) <
                monotonic_now()) {
              CaptureFrames();
              last_frame_ = monotonic_now();
            }
          }
        });

    // Run for enough time to allow the gyro/imu zeroing code to run.
    RunFor(std::chrono::seconds(10));
  }

  virtual ~LocalizedDrivetrainTest() override {}

  void SetStartingPosition(const Eigen::Matrix<double, 3, 1> &xytheta) {
    *drivetrain_plant_.mutable_state() << xytheta.x(), xytheta.y(),
        xytheta(2, 0), 0.0, 0.0;
    Eigen::Matrix<double, Localizer::HybridEkf::kNStates, 1> localizer_state;
    localizer_state.setZero();
    localizer_state.block<3, 1>(0, 0) = xytheta;
    localizer_.Reset(monotonic_now(), localizer_state);
  }

  void VerifyNearGoal(double eps = 1e-3) {
    drivetrain_goal_fetcher_.Fetch();
    EXPECT_NEAR(drivetrain_goal_fetcher_->left_goal(),
                drivetrain_plant_.GetLeftPosition(), eps);
    EXPECT_NEAR(drivetrain_goal_fetcher_->right_goal(),
                drivetrain_plant_.GetRightPosition(), eps);
  }

  void VerifyEstimatorAccurate(double eps) {
    const Eigen::Matrix<double, 5, 1> true_state = drivetrain_plant_.state();
    EXPECT_NEAR(localizer_.x(), true_state(0, 0), eps);
    EXPECT_NEAR(localizer_.y(), true_state(1, 0), eps);
    EXPECT_NEAR(localizer_.theta(), true_state(2, 0), eps);
    EXPECT_NEAR(localizer_.left_velocity(), true_state(3, 0), eps);
    EXPECT_NEAR(localizer_.right_velocity(), true_state(4, 0), eps);
  }

  // Goes through and captures frames on the camera(s), queueing them up to be
  // sent by SendDelayedFrames().
  void CaptureFrames() {
    const frc971::control_loops::Pose robot_pose(
        {drivetrain_plant_.GetPosition().x(),
         drivetrain_plant_.GetPosition().y(), 0.0},
        drivetrain_plant_.state()(2, 0));
    std::unique_ptr<ImageMatchResultT> frame(new ImageMatchResultT());

    // TODO(james): Test with more than one (and no) target(s).
    {
      std::unique_ptr<CameraPoseT> camera_target(new CameraPoseT());

      camera_target->field_to_target.reset(new TransformationMatrixT());
      camera_target->field_to_target->data = MatrixToVector(TargetLocation());

      // TODO(james): Use non-zero turret angles.
      camera_target->camera_to_target.reset(new TransformationMatrixT());
      camera_target->camera_to_target->data = MatrixToVector(
          (robot_pose.AsTransformationMatrix() * TurretRobotTransformation() *
           CameraTurretTransformation())
              .inverse() *
          TargetLocation());

      frame->camera_poses.emplace_back(std::move(camera_target));
    }

    frame->image_monotonic_timestamp_ns =
        chrono::duration_cast<chrono::nanoseconds>(
            monotonic_now().time_since_epoch())
            .count();
    frame->camera_calibration.reset(new CameraCalibrationT());
    {
      frame->camera_calibration->fixed_extrinsics.reset(
          new TransformationMatrixT());
      TransformationMatrixT *H_turret_robot =
          frame->camera_calibration->fixed_extrinsics.get();
      H_turret_robot->data = MatrixToVector(TurretRobotTransformation());
    }
    {
      frame->camera_calibration->turret_extrinsics.reset(
          new TransformationMatrixT());
      TransformationMatrixT *H_camera_turret =
          frame->camera_calibration->turret_extrinsics.get();
      H_camera_turret->data = MatrixToVector(CameraTurretTransformation());
    }

    camera_delay_queue_.emplace(monotonic_now(), std::move(frame));
  }

  // Actually sends out all the camera frames.
  void SendDelayedFrames() {
    const std::chrono::milliseconds camera_latency(150);
    while (!camera_delay_queue_.empty() &&
           std::get<0>(camera_delay_queue_.front()) <
               monotonic_now() - camera_latency) {
      auto builder = camera_sender_.MakeBuilder();
      ASSERT_TRUE(builder.Send(ImageMatchResult::Pack(
          *builder.fbb(), std::get<1>(camera_delay_queue_.front()).get())));
      camera_delay_queue_.pop();
    }
  }

  std::unique_ptr<aos::EventLoop> test_event_loop_;
  aos::Sender<Goal> drivetrain_goal_sender_;
  aos::Fetcher<Goal> drivetrain_goal_fetcher_;
  aos::Sender<LocalizerControl> localizer_control_sender_;

  std::unique_ptr<aos::EventLoop> drivetrain_event_loop_;
  const frc971::control_loops::drivetrain::DrivetrainConfig<double>
      dt_config_;

  aos::Sender<ImageMatchResult> camera_sender_;

  Localizer localizer_;
  DrivetrainLoop drivetrain_;

  std::unique_ptr<aos::EventLoop> drivetrain_plant_event_loop_;
  DrivetrainSimulation drivetrain_plant_;
  monotonic_clock::time_point last_frame_;

  // A queue of camera frames so that we can add a time delay to the data
  // coming from the cameras.
  std::queue<std::tuple<aos::monotonic_clock::time_point,
                        std::unique_ptr<ImageMatchResultT>>>
      camera_delay_queue_;

  void set_enable_cameras(bool enable) { enable_cameras_ = enable; }

 private:
  bool enable_cameras_ = false;

  std::unique_ptr<aos::EventLoop> logger_event_loop_;
  std::unique_ptr<aos::logger::DetachedBufferWriter> log_buffer_writer_;
  std::unique_ptr<aos::logger::Logger> logger_;
};

// Tests that no camera updates, combined with a perfect model, results in no
// error.
TEST_F(LocalizedDrivetrainTest, NoCameraUpdate) {
  SetEnabled(true);
  set_enable_cameras(false);
  VerifyEstimatorAccurate(1e-7);
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
  VerifyEstimatorAccurate(5e-4);
}

// Tests that camera udpates with a perfect models results in no errors.
TEST_F(LocalizedDrivetrainTest, PerfectCameraUpdate) {
  SetEnabled(true);
  set_enable_cameras(true);
  auto builder = drivetrain_goal_sender_.MakeBuilder();

  Goal::Builder drivetrain_builder = builder.MakeBuilder<Goal>();
  drivetrain_builder.add_controller_type(
      frc971::control_loops::drivetrain::ControllerType::MOTION_PROFILE);
  drivetrain_builder.add_left_goal(-1.0);
  drivetrain_builder.add_right_goal(1.0);

  EXPECT_TRUE(builder.Send(drivetrain_builder.Finish()));
  RunFor(chrono::seconds(3));
  VerifyNearGoal();
  VerifyEstimatorAccurate(5e-4);
}

// Tests that camera udpates with a constant initial error in the position
// results in convergence.
TEST_F(LocalizedDrivetrainTest, InitialPositionError) {
  SetEnabled(true);
  set_enable_cameras(true);
  drivetrain_plant_.mutable_state()->topRows(3) +=
      Eigen::Vector3d(0.1, 0.1, 0.01);
  auto builder = drivetrain_goal_sender_.MakeBuilder();

  Goal::Builder drivetrain_builder = builder.MakeBuilder<Goal>();
  drivetrain_builder.add_controller_type(
      frc971::control_loops::drivetrain::ControllerType::MOTION_PROFILE);
  drivetrain_builder.add_left_goal(-1.0);
  drivetrain_builder.add_right_goal(1.0);

  EXPECT_TRUE(builder.Send(drivetrain_builder.Finish()));
  // Give the filters enough time to converge.
  RunFor(chrono::seconds(10));
  VerifyNearGoal(5e-3);
  VerifyEstimatorAccurate(1e-2);
}

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2020
