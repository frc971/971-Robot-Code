#include "y2020/control_loops/drivetrain/localizer.h"

#include <queue>

#include "aos/events/logging/log_writer.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/network/team_number.h"
#include "aos/network/testing_time_converter.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "frc971/control_loops/drivetrain/drivetrain_test_lib.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"
#include "y2020/control_loops/drivetrain/drivetrain_base.h"
#include "y2020/control_loops/superstructure/superstructure_status_generated.h"

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
using frc971::vision::sift::CameraCalibrationT;
using frc971::vision::sift::CameraPoseT;
using frc971::vision::sift::ImageMatchResult;
using frc971::vision::sift::ImageMatchResultT;
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
  H.block<3, 1>(0, 3) << 1, 1.1, 0.9;
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
      Eigen::AngleAxis<double>(0.1, Eigen::Vector3d::UnitY()) *
      H.block<3, 3>(0, 0);
  return H;
}

// The absolute target location to use. Not meant to correspond with a
// particular field target.
// TODO(james): Make more targets.
std::vector<Eigen::Matrix<double, 4, 4>> TargetLocations() {
  std::vector<Eigen::Matrix<double, 4, 4>> locations;
  Eigen::Matrix<double, 4, 4> H;
  H.setIdentity();
  H.block<3, 1>(0, 3) << 10.0, 0, 0;
  locations.push_back(H);
  H.block<3, 1>(0, 3) << -10.0, 0, 0;
  locations.push_back(H);
  return locations;
}

constexpr std::chrono::seconds kPiTimeOffset(-10);
}  // namespace

namespace chrono = std::chrono;
using aos::monotonic_clock;
using frc971::control_loops::drivetrain::DrivetrainLoop;
using frc971::control_loops::drivetrain::testing::DrivetrainSimulation;

class LocalizedDrivetrainTest : public frc971::testing::ControlLoopTest {
 protected:
  // We must use the 2020 drivetrain config so that we don't have to deal
  // with shifting:
  LocalizedDrivetrainTest()
      : frc971::testing::ControlLoopTest(
            aos::configuration::ReadConfig(
                "y2020/control_loops/drivetrain/simulation_config.json"),
            GetTest2020DrivetrainConfig().dt),
        time_converter_(aos::configuration::NodesCount(configuration())),
        roborio_(aos::configuration::GetNode(configuration(), "roborio")),
        pi1_(aos::configuration::GetNode(configuration(), "pi1")),
        test_event_loop_(MakeEventLoop("test", roborio_)),
        drivetrain_goal_sender_(
            test_event_loop_->MakeSender<Goal>("/drivetrain")),
        drivetrain_goal_fetcher_(
            test_event_loop_->MakeFetcher<Goal>("/drivetrain")),
        localizer_control_sender_(
            test_event_loop_->MakeSender<LocalizerControl>("/drivetrain")),
        superstructure_status_sender_(
            test_event_loop_->MakeSender<superstructure::Status>(
                "/superstructure")),
        server_statistics_sender_(
            test_event_loop_->MakeSender<aos::message_bridge::ServerStatistics>(
                "/aos")),
        drivetrain_event_loop_(MakeEventLoop("drivetrain", roborio_)),
        dt_config_(GetTest2020DrivetrainConfig()),
        pi1_event_loop_(MakeEventLoop("test", pi1_)),
        camera_sender_(
            pi1_event_loop_->MakeSender<ImageMatchResult>("/pi1/camera")),
        localizer_(drivetrain_event_loop_.get(), dt_config_),
        drivetrain_(dt_config_, drivetrain_event_loop_.get(), &localizer_),
        drivetrain_plant_event_loop_(MakeEventLoop("plant", roborio_)),
        drivetrain_plant_(drivetrain_plant_event_loop_.get(), dt_config_),
        last_frame_(monotonic_now()) {
    event_loop_factory()->SetTimeConverter(&time_converter_);
    CHECK_EQ(aos::configuration::GetNodeIndex(configuration(), roborio_), 6);
    CHECK_EQ(aos::configuration::GetNodeIndex(configuration(), pi1_), 1);
    time_converter_.AddMonotonic({monotonic_clock::epoch() + kPiTimeOffset,
                                  monotonic_clock::epoch() + kPiTimeOffset,
                                  monotonic_clock::epoch() + kPiTimeOffset,
                                  monotonic_clock::epoch() + kPiTimeOffset,
                                  monotonic_clock::epoch() + kPiTimeOffset,
                                  monotonic_clock::epoch() + kPiTimeOffset,
                                  monotonic_clock::epoch()});
    set_team_id(frc971::control_loops::testing::kTeamNumber);
    set_battery_voltage(12.0);

    if (!FLAGS_output_file.empty()) {
      logger_event_loop_ = MakeEventLoop("logger", roborio_);
      logger_ = std::make_unique<aos::logger::Logger>(logger_event_loop_.get());
      logger_->StartLoggingLocalNamerOnRun(FLAGS_output_file);
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

    test_event_loop_->AddPhasedLoop(
        [this](int) {
          auto builder = server_statistics_sender_.MakeBuilder();
          auto name_offset = builder.fbb()->CreateString("pi1");
          auto node_builder = builder.MakeBuilder<aos::Node>();
          node_builder.add_name(name_offset);
          auto node_offset = node_builder.Finish();
          auto connection_builder =
              builder.MakeBuilder<aos::message_bridge::ServerConnection>();
          connection_builder.add_node(node_offset);
          connection_builder.add_monotonic_offset(
              chrono::duration_cast<chrono::nanoseconds>(kPiTimeOffset)
                  .count());
          auto connection_offset = connection_builder.Finish();
          auto connections_offset =
              builder.fbb()->CreateVector(&connection_offset, 1);
          auto statistics_builder =
              builder.MakeBuilder<aos::message_bridge::ServerStatistics>();
          statistics_builder.add_connections(connections_offset);
          builder.Send(statistics_builder.Finish());
        },
        chrono::milliseconds(500));

    test_event_loop_->AddPhasedLoop(
        [this](int) {
          // Also use the opportunity to send out turret messages.
          UpdateTurretPosition();
          auto builder = superstructure_status_sender_.MakeBuilder();
          auto turret_builder =
              builder
                  .MakeBuilder<frc971::control_loops::
                                   PotAndAbsoluteEncoderProfiledJointStatus>();
          turret_builder.add_position(turret_position_);
          turret_builder.add_velocity(turret_velocity_);
          auto turret_offset = turret_builder.Finish();
          auto status_builder = builder.MakeBuilder<superstructure::Status>();
          status_builder.add_turret(turret_offset);
          builder.Send(status_builder.Finish());
        },
        chrono::milliseconds(5));

    test_event_loop_->OnRun([this]() { SetStartingPosition({3.0, 2.0, 0.0}); });

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

  ::testing::AssertionResult IsNear(double expected, double actual,
                                    double epsilon) {
    if (std::abs(expected - actual) < epsilon) {
      return ::testing::AssertionSuccess();
    } else {
      return ::testing::AssertionFailure()
             << "Expected " << expected << " but got " << actual
             << " with a max difference of " << epsilon
             << " and an actual difference of " << std::abs(expected - actual);
    }
  }
  ::testing::AssertionResult VerifyEstimatorAccurate(double eps) {
    const Eigen::Matrix<double, 5, 1> true_state = drivetrain_plant_.state();
    ::testing::AssertionResult result(true);
    if (!(result = IsNear(localizer_.x(), true_state(0), eps))) {
      return result;
    }
    if (!(result = IsNear(localizer_.y(), true_state(1), eps))) {
      return result;
    }
    if (!(result = IsNear(localizer_.theta(), true_state(2), eps))) {
      return result;
    }
    if (!(result = IsNear(localizer_.left_velocity(), true_state(3), eps))) {
      return result;
    }
    if (!(result = IsNear(localizer_.right_velocity(), true_state(4), eps))) {
      return result;
    }
    return result;
  }

  // Goes through and captures frames on the camera(s), queueing them up to be
  // sent by SendDelayedFrames().
  void CaptureFrames() {
    const frc971::control_loops::Pose robot_pose(
        {drivetrain_plant_.GetPosition().x(),
         drivetrain_plant_.GetPosition().y(), 0.0},
        drivetrain_plant_.state()(2, 0));
    std::unique_ptr<ImageMatchResultT> frame(new ImageMatchResultT());

    for (const auto &H_field_target : TargetLocations()) {
      std::unique_ptr<CameraPoseT> camera_target(new CameraPoseT());

      camera_target->field_to_target.reset(new TransformationMatrixT());
      camera_target->field_to_target->data = MatrixToVector(H_field_target);

      Eigen::Matrix<double, 4, 4> H_turret_camera =
          Eigen::Matrix<double, 4, 4>::Identity();
      if (is_turreted_) {
        H_turret_camera = frc971::control_loops::TransformationMatrixForYaw(
                              turret_position_) *
                          CameraTurretTransformation();
      }

      // TODO(james): Use non-zero turret angles.
      camera_target->camera_to_target.reset(new TransformationMatrixT());
      camera_target->camera_to_target->data = MatrixToVector(
          (robot_pose.AsTransformationMatrix() * TurretRobotTransformation() *
           H_turret_camera * camera_calibration_offset_)
              .inverse() *
          H_field_target);

      frame->camera_poses.emplace_back(std::move(camera_target));
    }

    frame->image_monotonic_timestamp_ns =
        chrono::duration_cast<chrono::nanoseconds>(
            event_loop_factory()
                ->GetNodeEventLoopFactory(pi1_)
                ->monotonic_now()
                .time_since_epoch())
            .count();
    frame->camera_calibration.reset(new CameraCalibrationT());
    {
      frame->camera_calibration->fixed_extrinsics.reset(
          new TransformationMatrixT());
      TransformationMatrixT *H_robot_turret =
          frame->camera_calibration->fixed_extrinsics.get();
      H_robot_turret->data = MatrixToVector(TurretRobotTransformation());
    }

    if (is_turreted_) {
      frame->camera_calibration->turret_extrinsics.reset(
          new TransformationMatrixT());
      TransformationMatrixT *H_turret_camera =
          frame->camera_calibration->turret_extrinsics.get();
      H_turret_camera->data = MatrixToVector(CameraTurretTransformation());
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

  aos::message_bridge::TestingTimeConverter time_converter_;

  const aos::Node *const roborio_;
  const aos::Node *const pi1_;

  std::unique_ptr<aos::EventLoop> test_event_loop_;
  aos::Sender<Goal> drivetrain_goal_sender_;
  aos::Fetcher<Goal> drivetrain_goal_fetcher_;
  aos::Sender<LocalizerControl> localizer_control_sender_;
  aos::Sender<superstructure::Status> superstructure_status_sender_;
  aos::Sender<aos::message_bridge::ServerStatistics> server_statistics_sender_;

  std::unique_ptr<aos::EventLoop> drivetrain_event_loop_;
  const frc971::control_loops::drivetrain::DrivetrainConfig<double> dt_config_;

  std::unique_ptr<aos::EventLoop> pi1_event_loop_;
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

  // Offset to add to the camera for actually taking the images, to simulate an
  // inaccurate calibration.
  Eigen::Matrix<double, 4, 4> camera_calibration_offset_ =
      Eigen::Matrix<double, 4, 4>::Identity();

  void set_enable_cameras(bool enable) { enable_cameras_ = enable; }
  void set_camera_is_turreted(bool turreted) { is_turreted_ = turreted; }

  void set_turret(double position, double velocity) {
    turret_position_ = position;
    turret_velocity_ = velocity;
  }

  void SendGoal(double left, double right) {
    auto builder = drivetrain_goal_sender_.MakeBuilder();

    Goal::Builder drivetrain_builder = builder.MakeBuilder<Goal>();
    drivetrain_builder.add_controller_type(
        frc971::control_loops::drivetrain::ControllerType::MOTION_PROFILE);
    drivetrain_builder.add_left_goal(left);
    drivetrain_builder.add_right_goal(right);

    EXPECT_TRUE(builder.Send(drivetrain_builder.Finish()));
  }

 private:
  void UpdateTurretPosition() {
    turret_position_ +=
        turret_velocity_ *
        aos::time::DurationInSeconds(monotonic_now() - last_turret_update_);
    last_turret_update_ = monotonic_now();
  }

  bool enable_cameras_ = false;
  // Whether to make the camera be on the turret or not.
  bool is_turreted_ = true;

  // The time at which we last incremented turret_position_.
  monotonic_clock::time_point last_turret_update_ = monotonic_clock::min_time;
  // Current turret position and velocity. These are set directly by the user in
  // the test, and if velocity is non-zero, then we will automatically increment
  // turret_position_ with every timestep.
  double turret_position_ = 0.0;  // rad
  double turret_velocity_ = 0.0;  // rad / sec

  std::unique_ptr<aos::EventLoop> logger_event_loop_;
  std::unique_ptr<aos::logger::Logger> logger_;
};

// Tests that no camera updates, combined with a perfect model, results in no
// error.
TEST_F(LocalizedDrivetrainTest, NoCameraUpdate) {
  SetEnabled(true);
  set_enable_cameras(false);
  EXPECT_TRUE(VerifyEstimatorAccurate(1e-7));

  SendGoal(-1.0, 1.0);

  RunFor(chrono::seconds(3));
  VerifyNearGoal();
  EXPECT_TRUE(VerifyEstimatorAccurate(5e-4));
}

// Tests that we can drive in a straight line and have things estimate
// correctly.
TEST_F(LocalizedDrivetrainTest, NoCameraUpdateStraightLine) {
  SetEnabled(true);
  set_enable_cameras(false);
  EXPECT_TRUE(VerifyEstimatorAccurate(1e-7));

  SendGoal(1.0, 1.0);

  RunFor(chrono::seconds(1));
  VerifyNearGoal();
  // Due to accelerometer drift, the straight-line driving tends to be less
  // accurate...
  EXPECT_TRUE(VerifyEstimatorAccurate(0.15));
}

// Tests that camera updates with a perfect models results in no errors.
TEST_F(LocalizedDrivetrainTest, PerfectCameraUpdate) {
  SetEnabled(true);
  set_enable_cameras(true);
  set_camera_is_turreted(true);

  SendGoal(-1.0, 1.0);

  RunFor(chrono::seconds(3));
  VerifyNearGoal();
  EXPECT_TRUE(VerifyEstimatorAccurate(2e-3));
}

// Tests that camera updates with a perfect model but incorrect camera pitch
// results in no errors.
TEST_F(LocalizedDrivetrainTest, PerfectCameraUpdateWithBadPitch) {
  // Introduce some camera pitch.
  camera_calibration_offset_.template block<3, 3>(0, 0) =
      Eigen::AngleAxis<double>(0.1, Eigen::Vector3d::UnitY())
          .toRotationMatrix();
  SetEnabled(true);
  set_enable_cameras(true);
  set_camera_is_turreted(true);

  SendGoal(-1.0, 1.0);

  RunFor(chrono::seconds(3));
  VerifyNearGoal();
  EXPECT_TRUE(VerifyEstimatorAccurate(2e-3));
}

// Tests that camera updates with a constant initial error in the position
// results in convergence.
TEST_F(LocalizedDrivetrainTest, InitialPositionError) {
  SetEnabled(true);
  set_enable_cameras(true);
  set_camera_is_turreted(true);
  drivetrain_plant_.mutable_state()->topRows(3) +=
      Eigen::Vector3d(0.1, 0.1, 0.01);

  // Confirm that some translational movement does get handled correctly.
  SendGoal(-0.9, 1.0);

  // Give the filters enough time to converge.
  RunFor(chrono::seconds(10));
  VerifyNearGoal(5e-3);
  EXPECT_TRUE(VerifyEstimatorAccurate(4e-2));
}

// Tests that camera updates using a non-turreted camera work.
TEST_F(LocalizedDrivetrainTest, InitialPositionErrorNoTurret) {
  SetEnabled(true);
  set_enable_cameras(true);
  set_camera_is_turreted(false);
  drivetrain_plant_.mutable_state()->topRows(3) +=
      Eigen::Vector3d(0.1, 0.1, 0.01);

  SendGoal(-1.0, 1.0);

  // Give the filters enough time to converge.
  RunFor(chrono::seconds(10));
  VerifyNearGoal(5e-3);
  EXPECT_TRUE(VerifyEstimatorAccurate(4e-2));
}

// Tests that we are able to handle a constant, non-zero turret angle.
TEST_F(LocalizedDrivetrainTest, NonZeroTurret) {
  SetEnabled(true);
  set_enable_cameras(true);
  set_camera_is_turreted(true);
  set_turret(1.0, 0.0);
  drivetrain_plant_.mutable_state()->topRows(3) +=
      Eigen::Vector3d(0.1, 0.1, 0.0);

  SendGoal(-1.0, 1.0);

  // Give the filters enough time to converge.
  RunFor(chrono::seconds(10));
  VerifyNearGoal(5e-3);
  EXPECT_TRUE(VerifyEstimatorAccurate(1e-2));
}

// Tests that we are able to handle a constant velocity turret.
TEST_F(LocalizedDrivetrainTest, MovingTurret) {
  SetEnabled(true);
  set_enable_cameras(true);
  set_camera_is_turreted(true);
  set_turret(0.0, 0.2);
  drivetrain_plant_.mutable_state()->topRows(3) +=
      Eigen::Vector3d(0.1, 0.1, 0.0);

  SendGoal(-1.0, 1.0);

  // Give the filters enough time to converge.
  RunFor(chrono::seconds(10));
  VerifyNearGoal(5e-3);
  EXPECT_TRUE(VerifyEstimatorAccurate(1e-2));
}

// Tests that we reject camera measurements when the turret is spinning too
// fast.
TEST_F(LocalizedDrivetrainTest, TooFastTurret) {
  SetEnabled(true);
  set_enable_cameras(true);
  set_camera_is_turreted(true);
  set_turret(0.0, -10.0);
  const Eigen::Vector3d disturbance(0.1, 0.1, 0.0);
  drivetrain_plant_.mutable_state()->topRows(3) += disturbance;

  SendGoal(-1.0, 1.0);

  RunFor(chrono::seconds(10));
  EXPECT_FALSE(VerifyEstimatorAccurate(1e-3));
  // If we remove the disturbance, we should now be correct.
  drivetrain_plant_.mutable_state()->topRows(3) -= disturbance;
  VerifyNearGoal(5e-3);
  EXPECT_TRUE(VerifyEstimatorAccurate(2e-3));
}

// Tests that we don't reject camera measurements when the turret is spinning
// too fast but we aren't using a camera attached to the turret.
TEST_F(LocalizedDrivetrainTest, TooFastTurretDoesntAffectFixedCamera) {
  SetEnabled(true);
  set_enable_cameras(true);
  set_camera_is_turreted(false);
  set_turret(0.0, -10.0);
  const Eigen::Vector3d disturbance(0.1, 0.1, 0.0);
  drivetrain_plant_.mutable_state()->topRows(3) += disturbance;

  SendGoal(-1.0, 1.0);

  RunFor(chrono::seconds(10));
  VerifyNearGoal(5e-3);
  EXPECT_TRUE(VerifyEstimatorAccurate(1e-2));
}

// Tests that we don't blow up if we stop getting updates for an extended period
// of time and fall behind on fetching fron the cameras.
TEST_F(LocalizedDrivetrainTest, FetchersHandleTimeGap) {
  set_enable_cameras(true);
  set_send_delay(std::chrono::seconds(0));
  event_loop_factory()->set_network_delay(std::chrono::nanoseconds(1));
  test_event_loop_
      ->AddTimer([this]() { drivetrain_plant_.set_send_messages(false); })
      ->Setup(test_event_loop_->monotonic_now());
  test_event_loop_->AddPhasedLoop(
      [this](int) {
        auto builder = camera_sender_.MakeBuilder();
        ImageMatchResultT image;
        ASSERT_TRUE(
            builder.Send(ImageMatchResult::Pack(*builder.fbb(), &image)));
      },
      std::chrono::milliseconds(20));
  test_event_loop_
      ->AddTimer([this]() {
        drivetrain_plant_.set_send_messages(true);
        SimulateSensorReset();
      })
      ->Setup(test_event_loop_->monotonic_now() + std::chrono::seconds(10));

  RunFor(chrono::seconds(20));
}

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2020
