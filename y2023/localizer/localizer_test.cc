#include "y2023/localizer/localizer.h"

#include "aos/events/logging/log_writer.h"
#include "aos/events/simulated_event_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain_test_lib.h"
#include "frc971/control_loops/drivetrain/localizer_generated.h"
#include "frc971/control_loops/pose.h"
#include "frc971/vision/target_map_generated.h"
#include "gtest/gtest.h"
#include "y2023/constants/simulated_constants_sender.h"
#include "y2023/control_loops/drivetrain/drivetrain_base.h"
#include "y2023/localizer/status_generated.h"
#include "y2023/localizer/utils.h"

DEFINE_string(output_folder, "",
              "If set, logs all channels to the provided logfile.");
DECLARE_bool(die_on_malloc);

namespace y2023::localizer::testing {

using frc971::control_loops::drivetrain::Output;

class LocalizerTest : public ::testing::Test {
 protected:
  static constexpr uint64_t kTargetId = 1;
  LocalizerTest()
      : configuration_(aos::configuration::ReadConfig("y2023/aos_config.json")),
        event_loop_factory_(&configuration_.message()),
        roborio_node_([this]() {
          // Get the constants sent before anything else happens.
          // It has nothing to do with the roborio node.
          SendSimulationConstants(&event_loop_factory_, 7971,
                                  "y2023/constants/test_constants.json");
          return aos::configuration::GetNode(&configuration_.message(),
                                             "roborio");
        }()),
        imu_node_(
            aos::configuration::GetNode(&configuration_.message(), "imu")),
        camera_node_(
            aos::configuration::GetNode(&configuration_.message(), "pi1")),
        dt_config_(frc971::control_loops::drivetrain::testing::
                       GetTestDrivetrainConfig()),
        localizer_event_loop_(
            event_loop_factory_.MakeEventLoop("localizer", imu_node_)),
        localizer_(localizer_event_loop_.get(), dt_config_),
        drivetrain_plant_event_loop_(event_loop_factory_.MakeEventLoop(
            "drivetrain_plant", roborio_node_)),
        drivetrain_plant_imu_event_loop_(
            event_loop_factory_.MakeEventLoop("drivetrain_plant", imu_node_)),
        drivetrain_plant_(drivetrain_plant_event_loop_.get(),
                          drivetrain_plant_imu_event_loop_.get(), dt_config_,
                          std::chrono::microseconds(500)),
        roborio_test_event_loop_(
            event_loop_factory_.MakeEventLoop("test", roborio_node_)),
        imu_test_event_loop_(
            event_loop_factory_.MakeEventLoop("test", imu_node_)),
        camera_test_event_loop_(
            event_loop_factory_.MakeEventLoop("test", camera_node_)),
        logger_test_event_loop_(
            event_loop_factory_.GetNodeEventLoopFactory("logger")
                ->MakeEventLoop("test")),
        constants_fetcher_(imu_test_event_loop_.get()),
        output_sender_(
            roborio_test_event_loop_->MakeSender<Output>("/drivetrain")),
        target_sender_(
            camera_test_event_loop_->MakeSender<frc971::vision::TargetMap>(
                "/camera")),
        control_sender_(roborio_test_event_loop_->MakeSender<
                        frc971::control_loops::drivetrain::LocalizerControl>(
            "/drivetrain")),
        output_fetcher_(
            roborio_test_event_loop_
                ->MakeFetcher<frc971::controls::LocalizerOutput>("/localizer")),
        status_fetcher_(
            imu_test_event_loop_->MakeFetcher<Status>("/localizer")) {
    FLAGS_die_on_malloc = true;
    {
      aos::TimerHandler *timer = roborio_test_event_loop_->AddTimer([this]() {
        {
          auto builder = output_sender_.MakeBuilder();
          auto output_builder = builder.MakeBuilder<Output>();
          output_builder.add_left_voltage(output_voltages_(0));
          output_builder.add_right_voltage(output_voltages_(1));
          builder.CheckOk(builder.Send(output_builder.Finish()));
        }
      });
      roborio_test_event_loop_->OnRun([timer, this]() {
        timer->Setup(roborio_test_event_loop_->monotonic_now(),
                     std::chrono::milliseconds(5));
      });
    }
    {
      // Sanity check that the test calibration files look like what we
      // expect.
      CHECK_EQ("pi1", constants_fetcher_.constants()
                          .cameras()
                          ->Get(0)
                          ->calibration()
                          ->node_name()
                          ->string_view());
      const Eigen::Matrix<double, 4, 4> H_robot_camera =
          frc971::control_loops::drivetrain::FlatbufferToTransformationMatrix(
              *constants_fetcher_.constants()
                   .cameras()
                   ->Get(0)
                   ->calibration()
                   ->fixed_extrinsics());

      CHECK_EQ(kTargetId, constants_fetcher_.constants()
                              .target_map()
                              ->target_poses()
                              ->Get(0)
                              ->id());
      const Eigen::Matrix<double, 4, 4> H_field_target = PoseToTransform(
          constants_fetcher_.constants().target_map()->target_poses()->Get(0));
      // For reference, the camera should pointed straight forwards on the
      // robot, offset by 1 meter.
      aos::TimerHandler *timer = camera_test_event_loop_->AddTimer(
          [this, H_robot_camera, H_field_target]() {
            if (!send_targets_) {
              return;
            }
            const frc971::control_loops::Pose robot_pose(
                {drivetrain_plant_.GetPosition().x(),
                 drivetrain_plant_.GetPosition().y(), 0.0},
                drivetrain_plant_.state()(2, 0));

            const Eigen::Matrix<double, 4, 4> H_field_camera =
                robot_pose.AsTransformationMatrix() * H_robot_camera;
            const Eigen::Matrix<double, 4, 4> H_camera_target =
                H_field_camera.inverse() * H_field_target;

            const Eigen::Quaterniond quat(H_camera_target.block<3, 3>(0, 0));
            const Eigen::Vector3d translation(
                H_camera_target.block<3, 1>(0, 3));

            auto builder = target_sender_.MakeBuilder();
            frc971::vision::Quaternion::Builder quat_builder(*builder.fbb());
            quat_builder.add_w(quat.w());
            quat_builder.add_x(quat.x());
            quat_builder.add_y(quat.y());
            quat_builder.add_z(quat.z());
            auto quat_offset = quat_builder.Finish();
            frc971::vision::Position::Builder position_builder(*builder.fbb());
            position_builder.add_x(translation.x());
            position_builder.add_y(translation.y());
            position_builder.add_z(translation.z());
            auto position_offset = position_builder.Finish();

            frc971::vision::TargetPoseFbs::Builder target_builder(
                *builder.fbb());
            target_builder.add_id(send_target_id_);
            target_builder.add_position(position_offset);
            target_builder.add_orientation(quat_offset);
            target_builder.add_pose_error(pose_error_);
            auto target_offset = target_builder.Finish();

            auto targets_offset = builder.fbb()->CreateVector({target_offset});
            frc971::vision::TargetMap::Builder map_builder(*builder.fbb());
            map_builder.add_target_poses(targets_offset);
            map_builder.add_monotonic_timestamp_ns(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    camera_test_event_loop_->monotonic_now().time_since_epoch())
                    .count());

            builder.CheckOk(builder.Send(map_builder.Finish()));
          });
      camera_test_event_loop_->OnRun([timer, this]() {
        timer->Setup(camera_test_event_loop_->monotonic_now(),
                     std::chrono::milliseconds(50));
      });
    }

    localizer_control_send_timer_ =
        roborio_test_event_loop_->AddTimer([this]() {
          auto builder = control_sender_.MakeBuilder();
          auto control_builder = builder.MakeBuilder<
              frc971::control_loops::drivetrain::LocalizerControl>();
          control_builder.add_x(localizer_control_x_);
          control_builder.add_y(localizer_control_y_);
          control_builder.add_theta(localizer_control_theta_);
          control_builder.add_theta_uncertainty(0.01);
          control_builder.add_keep_current_theta(false);
          builder.CheckOk(builder.Send(control_builder.Finish()));
        });

    // Get things zeroed.
    event_loop_factory_.RunFor(std::chrono::seconds(10));
    CHECK(status_fetcher_.Fetch());
    CHECK(status_fetcher_->imu()->zeroed());

    if (!FLAGS_output_folder.empty()) {
      logger_event_loop_ =
          event_loop_factory_.MakeEventLoop("logger", imu_node_);
      logger_ = std::make_unique<aos::logger::Logger>(logger_event_loop_.get());
      logger_->StartLoggingOnRun(FLAGS_output_folder);
    }
  }

  void SendLocalizerControl(double x, double y, double theta) {
    localizer_control_x_ = x;
    localizer_control_y_ = y;
    localizer_control_theta_ = theta;
    localizer_control_send_timer_->Setup(
        roborio_test_event_loop_->monotonic_now());
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
    status_fetcher_.Fetch();
    if (!(result = IsNear(status_fetcher_->state()->x(), true_state(0), eps))) {
      return result;
    }
    if (!(result = IsNear(status_fetcher_->state()->y(), true_state(1), eps))) {
      return result;
    }
    if (!(result =
              IsNear(status_fetcher_->state()->theta(), true_state(2), eps))) {
      return result;
    }
    return result;
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> configuration_;
  aos::SimulatedEventLoopFactory event_loop_factory_;
  const aos::Node *const roborio_node_;
  const aos::Node *const imu_node_;
  const aos::Node *const camera_node_;
  const frc971::control_loops::drivetrain::DrivetrainConfig<double> dt_config_;
  std::unique_ptr<aos::EventLoop> localizer_event_loop_;
  Localizer localizer_;

  std::unique_ptr<aos::EventLoop> drivetrain_plant_event_loop_;
  std::unique_ptr<aos::EventLoop> drivetrain_plant_imu_event_loop_;
  frc971::control_loops::drivetrain::testing::DrivetrainSimulation
      drivetrain_plant_;

  std::unique_ptr<aos::EventLoop> roborio_test_event_loop_;
  std::unique_ptr<aos::EventLoop> imu_test_event_loop_;
  std::unique_ptr<aos::EventLoop> camera_test_event_loop_;
  std::unique_ptr<aos::EventLoop> logger_test_event_loop_;

  frc971::constants::ConstantsFetcher<Constants> constants_fetcher_;

  aos::Sender<Output> output_sender_;
  aos::Sender<frc971::vision::TargetMap> target_sender_;
  aos::Sender<frc971::control_loops::drivetrain::LocalizerControl>
      control_sender_;
  aos::Fetcher<frc971::controls::LocalizerOutput> output_fetcher_;
  aos::Fetcher<Status> status_fetcher_;

  Eigen::Vector2d output_voltages_ = Eigen::Vector2d::Zero();

  aos::TimerHandler *localizer_control_send_timer_;

  bool send_targets_ = false;

  double localizer_control_x_ = 0.0;
  double localizer_control_y_ = 0.0;
  double localizer_control_theta_ = 0.0;

  std::unique_ptr<aos::EventLoop> logger_event_loop_;
  std::unique_ptr<aos::logger::Logger> logger_;

  uint64_t send_target_id_ = kTargetId;
  double pose_error_ = 1e-7;

  gflags::FlagSaver flag_saver_;
};

// Test a simple scenario with no errors where the robot should just drive
// straight forwards.
TEST_F(LocalizerTest, Nominal) {
  output_voltages_ << 1.0, 1.0;
  event_loop_factory_.RunFor(std::chrono::seconds(2));
  CHECK(output_fetcher_.Fetch());
  CHECK(status_fetcher_.Fetch());
  // The two can be different because they may've been sent at different
  // times.
  EXPECT_NEAR(output_fetcher_->x(), status_fetcher_->state()->x(), 1e-2);
  EXPECT_NEAR(output_fetcher_->y(), status_fetcher_->state()->y(), 1e-6);
  EXPECT_NEAR(output_fetcher_->theta(), status_fetcher_->state()->theta(),
              1e-6);
  // Confirm that we did indeed drive forwards (and straight), as expected.
  EXPECT_LT(0.1, output_fetcher_->x());
  EXPECT_NEAR(0.0, output_fetcher_->y(), 1e-10);
  EXPECT_NEAR(0.0, output_fetcher_->theta(), 1e-10);
  EXPECT_NEAR(0.0, status_fetcher_->state()->left_voltage_error(), 1e-1);
  EXPECT_NEAR(0.0, status_fetcher_->state()->right_voltage_error(), 1e-1);

  // And check that we actually think that we are near where the simulator
  // says we are.
  EXPECT_TRUE(VerifyEstimatorAccurate(1e-2));
}

// Confirm that when the robot drives backwards that we localize correctly.
TEST_F(LocalizerTest, NominalReverse) {
  output_voltages_ << -1.0, -1.0;
  event_loop_factory_.RunFor(std::chrono::seconds(2));
  CHECK(output_fetcher_.Fetch());
  CHECK(status_fetcher_.Fetch());
  // The two can be different because they may've been sent at different
  // times.
  EXPECT_NEAR(output_fetcher_->x(), status_fetcher_->state()->x(), 1e-2);
  EXPECT_NEAR(output_fetcher_->y(), status_fetcher_->state()->y(), 1e-6);
  EXPECT_NEAR(output_fetcher_->theta(), status_fetcher_->state()->theta(),
              1e-6);
  // Confirm that we did indeed drive backwards (and straight), as expected.
  EXPECT_GT(-0.1, output_fetcher_->x());
  EXPECT_NEAR(0.0, output_fetcher_->y(), 1e-10);
  EXPECT_NEAR(0.0, output_fetcher_->theta(), 1e-10);
  EXPECT_NEAR(0.0, status_fetcher_->state()->left_voltage_error(), 1e-1);
  EXPECT_NEAR(0.0, status_fetcher_->state()->right_voltage_error(), 1e-1);

  // And check that we actually think that we are near where the simulator
  // says we are.
  EXPECT_TRUE(VerifyEstimatorAccurate(1e-2));
}

// Confirm that when the robot turns counter-clockwise that we localize
// correctly.
TEST_F(LocalizerTest, NominalSpinInPlace) {
  output_voltages_ << -1.0, 1.0;
  // Go 1 ms over 2 sec to make sure we actually see relatively recent messages
  // on each channel.
  event_loop_factory_.RunFor(std::chrono::milliseconds(2001));
  CHECK(output_fetcher_.Fetch());
  CHECK(status_fetcher_.Fetch());
  // The two can be different because they may've been sent at different
  // times.
  EXPECT_NEAR(output_fetcher_->x(), status_fetcher_->state()->x(), 1e-6);
  EXPECT_NEAR(output_fetcher_->y(), status_fetcher_->state()->y(), 1e-6);
  EXPECT_NEAR(output_fetcher_->theta(), status_fetcher_->state()->theta(),
              1e-2);
  // Confirm that we did indeed turn counter-clockwise.
  EXPECT_NEAR(0.0, output_fetcher_->x(), 1e-10);
  EXPECT_NEAR(0.0, output_fetcher_->y(), 1e-10);
  EXPECT_LT(0.1, output_fetcher_->theta());
  EXPECT_NEAR(0.0, status_fetcher_->state()->left_voltage_error(), 1e-1);
  EXPECT_NEAR(0.0, status_fetcher_->state()->right_voltage_error(), 1e-1);

  // And check that we actually think that we are near where the simulator
  // says we are.
  EXPECT_TRUE(VerifyEstimatorAccurate(1e-2));
}

// Confirm that when the robot drives in a curve that we localize
// successfully.
TEST_F(LocalizerTest, NominalCurve) {
  output_voltages_ << 2.0, 3.0;
  event_loop_factory_.RunFor(std::chrono::seconds(4));
  CHECK(output_fetcher_.Fetch());
  CHECK(status_fetcher_.Fetch());
  // The two can be different because they may've been sent at different
  // times.
  EXPECT_NEAR(output_fetcher_->x(), status_fetcher_->state()->x(), 1e-2);
  EXPECT_NEAR(output_fetcher_->y(), status_fetcher_->state()->y(), 1e-2);
  EXPECT_NEAR(output_fetcher_->theta(), status_fetcher_->state()->theta(),
              1e-2);
  // Confirm that we did indeed drive in a rough, counter-clockwise, curve.
  EXPECT_LT(0.1, output_fetcher_->x());
  EXPECT_LT(0.1, output_fetcher_->y());
  EXPECT_LT(0.1, output_fetcher_->theta());

  // And check that we actually think that we are near where the simulator
  // says we are.
  EXPECT_TRUE(VerifyEstimatorAccurate(1e-2));
}

// Tests that, in the presence of a non-zero voltage error, that we correct
// for it.
TEST_F(LocalizerTest, VoltageErrorDisabled) {
  output_voltages_ << 0.0, 0.0;
  drivetrain_plant_.set_left_voltage_offset(2.0);
  drivetrain_plant_.set_right_voltage_offset(2.0);

  event_loop_factory_.RunFor(std::chrono::seconds(2));
  CHECK(output_fetcher_.Fetch());
  CHECK(status_fetcher_.Fetch());
  // We should've just ended up driving straight forwards.
  EXPECT_LT(0.1, output_fetcher_->x());
  EXPECT_NEAR(0.0, output_fetcher_->y(), 1e-10);
  EXPECT_NEAR(0.0, output_fetcher_->theta(), 1e-10);
  EXPECT_NEAR(2.0, status_fetcher_->state()->left_voltage_error(), 1.0);
  EXPECT_NEAR(2.0, status_fetcher_->state()->right_voltage_error(), 1.0);

  // And check that we actually think that we are near where the simulator
  // says we are.
  EXPECT_TRUE(VerifyEstimatorAccurate(0.05));
}

// Tests that image corrections in the nominal case (no errors) causes no
// issues.
TEST_F(LocalizerTest, NominalImageCorrections) {
  output_voltages_ << 3.0, 2.0;
  send_targets_ = true;

  event_loop_factory_.RunFor(std::chrono::seconds(4));
  CHECK(status_fetcher_.Fetch());
  EXPECT_TRUE(VerifyEstimatorAccurate(1e-2));
  ASSERT_TRUE(status_fetcher_->has_statistics());
  ASSERT_EQ(4u /* number of cameras */, status_fetcher_->statistics()->size());
  ASSERT_EQ(status_fetcher_->statistics()->Get(0)->total_candidates(),
            status_fetcher_->statistics()->Get(0)->total_accepted());
  ASSERT_LT(10, status_fetcher_->statistics()->Get(0)->total_candidates());
}

// Tests that image corrections when there is an error at the start results
// in us actually getting corrected over time.
TEST_F(LocalizerTest, ImageCorrections) {
  output_voltages_ << 0.0, 0.0;
  drivetrain_plant_.mutable_state()->x() = 2.0;
  drivetrain_plant_.mutable_state()->y() = 2.0;
  SendLocalizerControl(5.0, 3.0, 0.0);
  event_loop_factory_.RunFor(std::chrono::seconds(4));
  CHECK(output_fetcher_.Fetch());
  ASSERT_NEAR(5.0, output_fetcher_->x(), 1e-5);
  ASSERT_NEAR(3.0, output_fetcher_->y(), 1e-5);
  ASSERT_NEAR(0.0, output_fetcher_->theta(), 1e-5);

  send_targets_ = true;

  event_loop_factory_.RunFor(std::chrono::seconds(4));
  CHECK(status_fetcher_.Fetch());
  EXPECT_TRUE(VerifyEstimatorAccurate(0.1));
  ASSERT_TRUE(status_fetcher_->has_statistics());
  ASSERT_EQ(4u /* number of cameras */, status_fetcher_->statistics()->size());
  ASSERT_EQ(status_fetcher_->statistics()->Get(0)->total_candidates(),
            status_fetcher_->statistics()->Get(0)->total_accepted());
  ASSERT_LT(10, status_fetcher_->statistics()->Get(0)->total_candidates());
}

// Tests that we correctly reject an invalid target.
TEST_F(LocalizerTest, InvalidTargetId) {
  output_voltages_ << 0.0, 0.0;
  send_targets_ = true;
  send_target_id_ = 100;

  event_loop_factory_.RunFor(std::chrono::seconds(4));
  CHECK(status_fetcher_.Fetch());
  ASSERT_TRUE(status_fetcher_->has_statistics());
  ASSERT_EQ(4u /* number of cameras */, status_fetcher_->statistics()->size());
  ASSERT_EQ(0, status_fetcher_->statistics()->Get(0)->total_accepted());
  ASSERT_LT(10, status_fetcher_->statistics()->Get(0)->total_candidates());
  ASSERT_EQ(status_fetcher_->statistics()
                ->Get(0)
                ->rejection_reasons()
                ->Get(static_cast<size_t>(RejectionReason::NO_SUCH_TARGET))
                ->count(),
            status_fetcher_->statistics()->Get(0)->total_candidates());
}

// Tests that we correctly reject a detection with a high pose error.
TEST_F(LocalizerTest, HighPoseError) {
  output_voltages_ << 0.0, 0.0;
  send_targets_ = true;
  // Send the minimum pose error to be rejected
  constexpr double kEps = 1e-9;
  pose_error_ = 1e-6 + kEps;

  event_loop_factory_.RunFor(std::chrono::seconds(4));
  CHECK(status_fetcher_.Fetch());
  ASSERT_TRUE(status_fetcher_->has_statistics());
  ASSERT_EQ(4u /* number of cameras */, status_fetcher_->statistics()->size());
  ASSERT_EQ(0, status_fetcher_->statistics()->Get(0)->total_accepted());
  ASSERT_LT(10, status_fetcher_->statistics()->Get(0)->total_candidates());
  ASSERT_EQ(status_fetcher_->statistics()
                ->Get(0)
                ->rejection_reasons()
                ->Get(static_cast<size_t>(RejectionReason::HIGH_POSE_ERROR))
                ->count(),
            status_fetcher_->statistics()->Get(0)->total_candidates());
}

}  // namespace y2023::localizer::testing
