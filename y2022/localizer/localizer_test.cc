#include "y2022/localizer/localizer.h"

#include "aos/events/logging/log_writer.h"
#include "aos/events/simulated_event_loop.h"
#include "frc971/control_loops/drivetrain/drivetrain_test_lib.h"
#include "frc971/control_loops/pose.h"
#include "gtest/gtest.h"
#include "y2022/control_loops/drivetrain/drivetrain_base.h"
#include "y2022/control_loops/superstructure/superstructure_status_generated.h"
#include "y2022/vision/target_estimate_generated.h"

DEFINE_string(output_folder, "",
              "If set, logs all channels to the provided logfile.");

namespace frc971::controls::testing {
typedef ModelBasedLocalizer::ModelState ModelState;
typedef ModelBasedLocalizer::AccelState AccelState;
typedef ModelBasedLocalizer::ModelInput ModelInput;
typedef ModelBasedLocalizer::AccelInput AccelInput;

using frc971::control_loops::Pose;
using frc971::control_loops::drivetrain::DrivetrainConfig;
using frc971::control_loops::drivetrain::LocalizerControl;
using frc971::vision::calibration::CameraCalibrationT;
using frc971::vision::calibration::TransformationMatrixT;
using y2022::vision::TargetEstimate;
using y2022::vision::TargetEstimateT;

namespace {
constexpr size_t kX = ModelBasedLocalizer::kX;
constexpr size_t kY = ModelBasedLocalizer::kY;
constexpr size_t kTheta = ModelBasedLocalizer::kTheta;
constexpr size_t kVelocityX = ModelBasedLocalizer::kVelocityX;
constexpr size_t kVelocityY = ModelBasedLocalizer::kVelocityY;
constexpr size_t kAccelX = ModelBasedLocalizer::kAccelX;
constexpr size_t kAccelY = ModelBasedLocalizer::kAccelY;
constexpr size_t kThetaRate = ModelBasedLocalizer::kThetaRate;
constexpr size_t kLeftEncoder = ModelBasedLocalizer::kLeftEncoder;
constexpr size_t kLeftVelocity = ModelBasedLocalizer::kLeftVelocity;
constexpr size_t kLeftVoltageError = ModelBasedLocalizer::kLeftVoltageError;
constexpr size_t kRightEncoder = ModelBasedLocalizer::kRightEncoder;
constexpr size_t kRightVelocity = ModelBasedLocalizer::kRightVelocity;
constexpr size_t kRightVoltageError = ModelBasedLocalizer::kRightVoltageError;
constexpr size_t kLeftVoltage = ModelBasedLocalizer::kLeftVoltage;
constexpr size_t kRightVoltage = ModelBasedLocalizer::kRightVoltage;

Eigen::Matrix<double, 4, 4> TurretRobotTransformation() {
  Eigen::Matrix<double, 4, 4> H;
  H.setIdentity();
  H.block<3, 1>(0, 3) << 1, 1.1, 0.9;
  return H;
}

// Provides the location of the camera on the turret.
Eigen::Matrix<double, 4, 4> CameraTurretTransformation() {
  Eigen::Matrix<double, 4, 4> H;
  H.setIdentity();
  H.block<3, 1>(0, 3) << 0.1, 0, 0;
  H.block<3, 3>(0, 0) << 0, 0, 1, -1, 0, 0, 0, -1, 0;

  // Introduce a bit of pitch to make sure that we're exercising all the code.
  H.block<3, 3>(0, 0) =
      Eigen::AngleAxis<double>(0.1, Eigen::Vector3d::UnitY()) *
      H.block<3, 3>(0, 0);
  return H;
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

DrivetrainConfig<double> GetTest2022DrivetrainConfig() {
  DrivetrainConfig<double> config =
      y2022::control_loops::drivetrain::GetDrivetrainConfig();
  config.is_simulated = true;
  return config;
}
}  // namespace

class LocalizerTest : public ::testing::Test {
 protected:
  LocalizerTest()
      : dt_config_(GetTest2022DrivetrainConfig()), localizer_(dt_config_) {
    localizer_.set_longitudinal_offset(0.0);
  }
  ModelState CallDiffModel(const ModelState &state, const ModelInput &U) {
    return localizer_.DiffModel(state, U);
  }

  AccelState CallDiffAccel(const AccelState &state, const AccelInput &U) {
    return localizer_.DiffAccel(state, U);
  }

  const control_loops::drivetrain::DrivetrainConfig<double> dt_config_;
  ModelBasedLocalizer localizer_;
};

TEST_F(LocalizerTest, AccelIntegrationTest) {
  AccelState state;
  state.setZero();
  AccelInput input;
  input.setZero();

  EXPECT_EQ(0.0, CallDiffAccel(state, input).norm());
  // Non-zero x/y/theta states should still result in a zero derivative.
  state(kX) = 1.0;
  state(kY) = 1.0;
  state(kTheta) = 1.0;
  EXPECT_EQ(0.0, CallDiffAccel(state, input).norm());

  state.setZero();
  state(kVelocityX) = 1.0;
  state(kVelocityY) = 2.0;
  EXPECT_EQ((AccelState() << 1.0, 2.0, 0.0, 0.0, 0.0).finished(),
            CallDiffAccel(state, input));
  // Derivatives should be independent of theta.
  state(kTheta) = M_PI / 2.0;
  EXPECT_EQ((AccelState() << 1.0, 2.0, 0.0, 0.0, 0.0).finished(),
            CallDiffAccel(state, input));

  state.setZero();
  input(kAccelX) = 1.0;
  input(kAccelY) = 2.0;
  input(kThetaRate) = 3.0;
  EXPECT_EQ((AccelState() << 0.0, 0.0, 3.0, 1.0, 2.0).finished(),
            CallDiffAccel(state, input));
  state(kTheta) = M_PI / 2.0;
  EXPECT_EQ((AccelState() << 0.0, 0.0, 3.0, 1.0, 2.0).finished(),
            CallDiffAccel(state, input));
}

TEST_F(LocalizerTest, ModelIntegrationTest) {
  ModelState state;
  state.setZero();
  ModelInput input;
  input.setZero();
  ModelState diff;

  EXPECT_EQ(0.0, CallDiffModel(state, input).norm());
  // Non-zero x/y/theta/encoder states should still result in a zero derivative.
  state(kX) = 1.0;
  state(kY) = 1.0;
  state(kTheta) = 1.0;
  state(kLeftEncoder) = 1.0;
  state(kRightEncoder) = 1.0;
  EXPECT_EQ(0.0, CallDiffModel(state, input).norm());

  state.setZero();
  state(kLeftVelocity) = 1.0;
  state(kRightVelocity) = 1.0;
  diff = CallDiffModel(state, input);
  const ModelState mask_velocities =
      (ModelState() << 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0).finished();
  EXPECT_EQ(
      (ModelState() << 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0).finished(),
      diff.cwiseProduct(mask_velocities));
  EXPECT_EQ(diff(kLeftVelocity), diff(kRightVelocity));
  EXPECT_GT(0.0, diff(kLeftVelocity));
  state(kTheta) = M_PI / 2.0;
  diff = CallDiffModel(state, input);
  EXPECT_NEAR(0.0,
              ((ModelState() << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
                   .finished() -
               diff.cwiseProduct(mask_velocities))
                  .norm(),
              1e-12);
  EXPECT_EQ(diff(kLeftVelocity), diff(kRightVelocity));
  EXPECT_GT(0.0, diff(kLeftVelocity));

  state.setZero();
  state(kLeftVelocity) = -1.0;
  state(kRightVelocity) = 1.0;
  diff = CallDiffModel(state, input);
  EXPECT_EQ((ModelState() << 0.0, 0.0, 1.0 / dt_config_.robot_radius, -1.0, 0.0,
             0.0, 1.0, 0.0, 0.0)
                .finished(),
            diff.cwiseProduct(mask_velocities));
  EXPECT_EQ(-diff(kLeftVelocity), diff(kRightVelocity));
  EXPECT_LT(0.0, diff(kLeftVelocity));

  state.setZero();
  input(kLeftVoltage) = 5.0;
  input(kRightVoltage) = 6.0;
  diff = CallDiffModel(state, input);
  EXPECT_EQ(0, diff(kX));
  EXPECT_EQ(0, diff(kY));
  EXPECT_EQ(0, diff(kTheta));
  EXPECT_EQ(0, diff(kLeftEncoder));
  EXPECT_EQ(0, diff(kRightEncoder));
  EXPECT_EQ(0, diff(kLeftVoltageError));
  EXPECT_EQ(0, diff(kRightVoltageError));
  EXPECT_LT(0, diff(kLeftVelocity));
  EXPECT_LT(0, diff(kRightVelocity));
  EXPECT_LT(diff(kLeftVelocity), diff(kRightVelocity));

  state.setZero();
  state(kLeftVoltageError) = -1.0;
  state(kRightVoltageError) = -2.0;
  input(kLeftVoltage) = 1.0;
  input(kRightVoltage) = 2.0;
  EXPECT_EQ(ModelState::Zero(), CallDiffModel(state, input));
}

// Test that the HandleReset does indeed reset the state of the localizer.
TEST_F(LocalizerTest, LocalizerReset) {
  aos::monotonic_clock::time_point t = aos::monotonic_clock::epoch();
  localizer_.HandleReset(t, {1.0, 2.0, 3.0});
  EXPECT_EQ((Eigen::Vector3d{1.0, 2.0, 3.0}), localizer_.xytheta());
  localizer_.HandleReset(t, {4.0, 5.0, 6.0});
  EXPECT_EQ((Eigen::Vector3d{4.0, 5.0, 6.0}), localizer_.xytheta());
}

// Test that if we are moving only by accelerometer readings (and just assuming
// zero voltage/encoders) that we initially don't believe it but then latch into
// following the accelerometer.
// Note: this test is somewhat sensitive to the exact tuning values used for the
// filter.
TEST_F(LocalizerTest, AccelOnly) {
  const aos::monotonic_clock::time_point start = aos::monotonic_clock::epoch();
  const std::chrono::microseconds kDt{500};
  aos::monotonic_clock::time_point t = start - std::chrono::milliseconds(1000);
  Eigen::Vector3d gyro{0.0, 0.0, 0.0};
  const Eigen::Vector2d encoders{0.0, 0.0};
  const Eigen::Vector2d voltages{0.0, 0.0};
  Eigen::Vector3d accel{5.0, 2.0, 9.80665};
  Eigen::Vector3d accel_gs =
      dt_config_.imu_transform.inverse() * accel / 9.80665;
  while (t < start) {
    // Spin to fill up the buffer.
    localizer_.HandleImu(t, gyro, Eigen::Vector3d::UnitZ(), encoders, voltages);
    t += kDt;
  }
  while (t < start + std::chrono::milliseconds(100)) {
    localizer_.HandleImu(t, gyro, accel_gs, encoders, voltages);
    EXPECT_EQ(Eigen::Vector3d::Zero(), localizer_.xytheta());
    t += kDt;
  }
  while (t < start + std::chrono::milliseconds(500)) {
    // Too lazy to hard-code when the transition happens.
    localizer_.HandleImu(t, gyro, accel_gs, encoders, voltages);
    t += kDt;
  }
  while (t < start + std::chrono::milliseconds(1000)) {
    SCOPED_TRACE(t);
    localizer_.HandleImu(t, gyro, accel_gs, encoders, voltages);
    const Eigen::Vector3d xytheta = localizer_.xytheta();
    t += kDt;
    EXPECT_NEAR(
        0.5 * accel(0) * std::pow(aos::time::DurationInSeconds(t - start), 2),
        xytheta(0), 1e-4);
    EXPECT_NEAR(
        0.5 * accel(1) * std::pow(aos::time::DurationInSeconds(t - start), 2),
        xytheta(1), 1e-4);
    EXPECT_EQ(0.0, xytheta(2));
  }

  ASSERT_NEAR(accel(0), localizer_.accel_state()(kVelocityX), 1e-10);
  ASSERT_NEAR(accel(1), localizer_.accel_state()(kVelocityY), 1e-10);

  // Start going in a cirlce, and confirm that we
  // handle things correctly. We rotate the accelerometer readings by 90 degrees
  // and then leave them constant, which should make it look like we are going
  // around in a circle.
  accel = Eigen::Vector3d{-accel(1), accel(0), 9.80665};
  accel_gs = dt_config_.imu_transform.inverse() * accel / 9.80665;
  // v^2 / r = a
  // w * r = v
  // v^2 / v * w = a
  // w = a / v
  const double omega = accel.topRows<2>().norm() /
                       std::hypot(localizer_.accel_state()(kVelocityX),
                                  localizer_.accel_state()(kVelocityY));
  gyro << 0.0, 0.0, omega;
  // Due to the magic of math, omega works out to be 1.0 after having run at the
  // acceleration for one second.
  ASSERT_NEAR(1.0, omega, 1e-10);
  // Yes, we could save some operations here, but let's be at least somewhat
  // clear about what we're doing...
  const double radius = accel.topRows<2>().norm() / (omega * omega);
  const Eigen::Vector2d center = localizer_.xytheta().topRows<2>() +
                                 accel.topRows<2>().normalized() * radius;
  const double initial_theta = std::atan2(-accel(1), -accel(0));

  std::chrono::microseconds one_revolution_time(
      static_cast<int>(2 * M_PI / omega * 1e6));

  aos::monotonic_clock::time_point circle_start = t;

  while (t < circle_start + one_revolution_time) {
    SCOPED_TRACE(t);
    localizer_.HandleImu(t, gyro, accel_gs, encoders, voltages);
    t += kDt;
    const double t_circle = aos::time::DurationInSeconds(t - circle_start);
    ASSERT_NEAR(t_circle * omega, localizer_.xytheta()(2), 1e-5);
    const double theta_circle = t_circle * omega + initial_theta;
    const Eigen::Vector2d offset =
        radius *
        Eigen::Vector2d{std::cos(theta_circle), std::sin(theta_circle)};
    const Eigen::Vector2d expected = center + offset;
    const Eigen::Vector2d estimated = localizer_.xytheta().topRows<2>();
    const Eigen::Vector2d implied_offset = estimated - center;
    const double implied_theta =
        std::atan2(implied_offset.y(), implied_offset.x());
    VLOG(1) << "center: " << center.transpose() << " radius " << radius
            << "\nlocalizer " << localizer_.xytheta().transpose()
            << " t_circle " << t_circle << " omega " << omega << " theta "
            << theta_circle << "\noffset " << offset.transpose()
            << "\nexpected " << expected.transpose() << "\nimplied offset "
            << implied_offset << " implied_theta " << implied_theta << "\nvel "
            << localizer_.accel_state()(kVelocityX) << ", "
            << localizer_.accel_state()(kVelocityY);
    ASSERT_NEAR(0.0, (expected - localizer_.xytheta().topRows<2>()).norm(),
                1e-2);
  }

  // Set accelerometer back to zero and confirm that we recover (the
  // implementation decays the accelerometer speeds to zero when still, so
  // should recover).
  while (t <
         circle_start + one_revolution_time + std::chrono::milliseconds(3000)) {
    localizer_.HandleImu(t, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(),
                         encoders, voltages);
    t += kDt;
  }
  const Eigen::Vector3d final_pos = localizer_.xytheta();
  localizer_.HandleImu(t, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(),
                       encoders, voltages);
  ASSERT_NEAR(0.0, (final_pos - localizer_.xytheta()).norm(), 1e-10);
}

using control_loops::drivetrain::Output;

class EventLoopLocalizerTest : public ::testing::Test {
 protected:
  EventLoopLocalizerTest()
      : configuration_(aos::configuration::ReadConfig("y2022/aos_config.json")),
        event_loop_factory_(&configuration_.message()),
        roborio_node_(
            aos::configuration::GetNode(&configuration_.message(), "roborio")),
        imu_node_(
            aos::configuration::GetNode(&configuration_.message(), "imu")),
        camera_node_(
            aos::configuration::GetNode(&configuration_.message(), "pi1")),
        dt_config_(
            control_loops::drivetrain::testing::GetTestDrivetrainConfig()),
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
        output_sender_(
            roborio_test_event_loop_->MakeSender<Output>("/drivetrain")),
        turret_sender_(
            roborio_test_event_loop_
                ->MakeSender<y2022::control_loops::superstructure::Status>(
                    "/superstructure")),
        target_sender_(
            camera_test_event_loop_->MakeSender<y2022::vision::TargetEstimate>(
                "/camera")),
        control_sender_(roborio_test_event_loop_->MakeSender<LocalizerControl>(
            "/drivetrain")),
        output_fetcher_(roborio_test_event_loop_->MakeFetcher<LocalizerOutput>(
            "/localizer")),
        status_fetcher_(
            imu_test_event_loop_->MakeFetcher<LocalizerStatus>("/localizer")) {
    localizer_.localizer()->set_longitudinal_offset(0.0);
    {
      aos::TimerHandler *timer = roborio_test_event_loop_->AddTimer([this]() {
        {
          auto builder = output_sender_.MakeBuilder();
          auto output_builder = builder.MakeBuilder<Output>();
          output_builder.add_left_voltage(output_voltages_(0));
          output_builder.add_right_voltage(output_voltages_(1));
          builder.CheckOk(builder.Send(output_builder.Finish()));
        }
        {
          auto builder = turret_sender_.MakeBuilder();
          auto turret_estimator_builder =
              builder
                  .MakeBuilder<frc971::PotAndAbsoluteEncoderEstimatorState>();
          turret_estimator_builder.add_position(turret_position_);
          const flatbuffers::Offset<frc971::PotAndAbsoluteEncoderEstimatorState>
              turret_estimator_offset = turret_estimator_builder.Finish();
          auto turret_builder =
              builder
                  .MakeBuilder<frc971::control_loops::
                                   PotAndAbsoluteEncoderProfiledJointStatus>();
          turret_builder.add_position(turret_position_);
          turret_builder.add_velocity(turret_velocity_);
          turret_builder.add_zeroed(true);
          turret_builder.add_estimator_state(turret_estimator_offset);
          const auto turret_offset = turret_builder.Finish();
          auto status_builder =
              builder
                  .MakeBuilder<y2022::control_loops::superstructure::Status>();
          status_builder.add_turret(turret_offset);
          builder.CheckOk(builder.Send(status_builder.Finish()));
        }
      });
      roborio_test_event_loop_->OnRun([timer, this]() {
        timer->Setup(roborio_test_event_loop_->monotonic_now(),
                     std::chrono::milliseconds(5));
      });
    }
    {
      aos::TimerHandler *timer = camera_test_event_loop_->AddTimer([this]() {
        if (!send_targets_) {
          return;
        }
        const frc971::control_loops::Pose robot_pose(
            {drivetrain_plant_.GetPosition().x(),
             drivetrain_plant_.GetPosition().y(), 0.0},
            drivetrain_plant_.state()(2, 0));
        const Eigen::Matrix<double, 4, 4> H_turret_camera =
            frc971::control_loops::TransformationMatrixForYaw(
                turret_position_) *
            CameraTurretTransformation();

        const Eigen::Matrix<double, 4, 4> H_field_camera =
            robot_pose.AsTransformationMatrix() * TurretRobotTransformation() *
            H_turret_camera;
        const Eigen::Matrix<double, 4, 4> target_transform =
            Eigen::Matrix<double, 4, 4>::Identity();
        const Eigen::Matrix<double, 4, 4> H_camera_target =
            H_field_camera.inverse() * target_transform;
        const Eigen::Matrix<double, 4, 4> H_target_camera =
            H_camera_target.inverse();

        std::unique_ptr<y2022::vision::TargetEstimateT> estimate(
            new y2022::vision::TargetEstimateT());
        estimate->distance = H_target_camera.block<2, 1>(0, 3).norm();
        estimate->angle_to_target =
            std::atan2(-H_camera_target(0, 3), H_camera_target(2, 3));
        estimate->camera_calibration.reset(new CameraCalibrationT());
        {
          estimate->camera_calibration->fixed_extrinsics.reset(
              new TransformationMatrixT());
          TransformationMatrixT *H_robot_turret =
              estimate->camera_calibration->fixed_extrinsics.get();
          H_robot_turret->data = MatrixToVector(TurretRobotTransformation());
        }

        estimate->camera_calibration->turret_extrinsics.reset(
            new TransformationMatrixT());
        estimate->camera_calibration->turret_extrinsics->data =
            MatrixToVector(CameraTurretTransformation());

        estimate->confidence = 1.0;

        auto builder = target_sender_.MakeBuilder();
        builder.CheckOk(
            builder.Send(TargetEstimate::Pack(*builder.fbb(), estimate.get())));
      });
      camera_test_event_loop_->OnRun([timer, this]() {
        timer->Setup(camera_test_event_loop_->monotonic_now(),
                     std::chrono::milliseconds(50));
      });
    }

    localizer_control_send_timer_ =
        roborio_test_event_loop_->AddTimer([this]() {
          auto builder = control_sender_.MakeBuilder();
          auto control_builder = builder.MakeBuilder<LocalizerControl>();
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
    CHECK(status_fetcher_->zeroed());

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
    if (!(result = IsNear(status_fetcher_->model_based()->x(), true_state(0),
                          eps))) {
      return result;
    }
    if (!(result = IsNear(status_fetcher_->model_based()->y(), true_state(1),
                          eps))) {
      return result;
    }
    if (!(result = IsNear(status_fetcher_->model_based()->theta(),
                          true_state(2), eps))) {
      return result;
    }
    return result;
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> configuration_;
  aos::SimulatedEventLoopFactory event_loop_factory_;
  const aos::Node *const roborio_node_;
  const aos::Node *const imu_node_;
  const aos::Node *const camera_node_;
  const control_loops::drivetrain::DrivetrainConfig<double> dt_config_;
  std::unique_ptr<aos::EventLoop> localizer_event_loop_;
  EventLoopLocalizer localizer_;

  std::unique_ptr<aos::EventLoop> drivetrain_plant_event_loop_;
  std::unique_ptr<aos::EventLoop> drivetrain_plant_imu_event_loop_;
  control_loops::drivetrain::testing::DrivetrainSimulation drivetrain_plant_;

  std::unique_ptr<aos::EventLoop> roborio_test_event_loop_;
  std::unique_ptr<aos::EventLoop> imu_test_event_loop_;
  std::unique_ptr<aos::EventLoop> camera_test_event_loop_;
  std::unique_ptr<aos::EventLoop> logger_test_event_loop_;

  aos::Sender<Output> output_sender_;
  aos::Sender<y2022::control_loops::superstructure::Status> turret_sender_;
  aos::Sender<y2022::vision::TargetEstimate> target_sender_;
  aos::Sender<LocalizerControl> control_sender_;
  aos::Fetcher<LocalizerOutput> output_fetcher_;
  aos::Fetcher<LocalizerStatus> status_fetcher_;

  Eigen::Vector2d output_voltages_ = Eigen::Vector2d::Zero();

  aos::TimerHandler *localizer_control_send_timer_;

  bool send_targets_ = false;
  double turret_position_ = 0.0;
  double turret_velocity_ = 0.0;

  double localizer_control_x_ = 0.0;
  double localizer_control_y_ = 0.0;
  double localizer_control_theta_ = 0.0;

  std::unique_ptr<aos::EventLoop> logger_event_loop_;
  std::unique_ptr<aos::logger::Logger> logger_;
};

TEST_F(EventLoopLocalizerTest, Nominal) {
  output_voltages_ << 1.0, 1.0;
  event_loop_factory_.RunFor(std::chrono::seconds(2));
  drivetrain_plant_.set_accel_sin_magnitude(0.01);
  CHECK(output_fetcher_.Fetch());
  CHECK(status_fetcher_.Fetch());
  // The two can be different because they may've been sent at different times.
  ASSERT_NEAR(output_fetcher_->x(), status_fetcher_->model_based()->x(), 1e-6);
  ASSERT_NEAR(output_fetcher_->y(), status_fetcher_->model_based()->y(), 1e-6);
  ASSERT_NEAR(output_fetcher_->theta(), status_fetcher_->model_based()->theta(),
              1e-6);
  ASSERT_LT(0.1, output_fetcher_->x());
  ASSERT_NEAR(0.0, output_fetcher_->y(), 1e-10);
  ASSERT_NEAR(0.0, output_fetcher_->theta(), 1e-10);
  ASSERT_TRUE(status_fetcher_->has_model_based());
  ASSERT_TRUE(status_fetcher_->model_based()->using_model());
  ASSERT_LT(0.1, status_fetcher_->model_based()->accel_state()->velocity_x());
  ASSERT_NEAR(0.0, status_fetcher_->model_based()->accel_state()->velocity_y(),
              1e-10);
  ASSERT_NEAR(
      0.0, status_fetcher_->model_based()->model_state()->left_voltage_error(),
      1e-1);
  ASSERT_NEAR(
      0.0, status_fetcher_->model_based()->model_state()->right_voltage_error(),
      1e-1);
}

TEST_F(EventLoopLocalizerTest, Reverse) {
  output_voltages_ << -4.0, -4.0;
  drivetrain_plant_.set_accel_sin_magnitude(0.01);
  event_loop_factory_.RunFor(std::chrono::seconds(2));
  CHECK(output_fetcher_.Fetch());
  CHECK(status_fetcher_.Fetch());
  // The two can be different because they may've been sent at different times.
  ASSERT_NEAR(output_fetcher_->x(), status_fetcher_->model_based()->x(), 1e-6);
  ASSERT_NEAR(output_fetcher_->y(), status_fetcher_->model_based()->y(), 1e-6);
  ASSERT_NEAR(output_fetcher_->theta(), status_fetcher_->model_based()->theta(),
              1e-6);
  ASSERT_GT(-0.1, output_fetcher_->x());
  ASSERT_NEAR(0.0, output_fetcher_->y(), 1e-10);
  ASSERT_NEAR(0.0, output_fetcher_->theta(), 1e-10);
  ASSERT_TRUE(status_fetcher_->has_model_based());
  ASSERT_TRUE(status_fetcher_->model_based()->using_model());
  ASSERT_GT(-0.1, status_fetcher_->model_based()->accel_state()->velocity_x());
  ASSERT_NEAR(0.0, status_fetcher_->model_based()->accel_state()->velocity_y(),
              1e-10);
  ASSERT_NEAR(
      0.0, status_fetcher_->model_based()->model_state()->left_voltage_error(),
      1e-1);
  ASSERT_NEAR(
      0.0, status_fetcher_->model_based()->model_state()->right_voltage_error(),
      1e-1);
}

TEST_F(EventLoopLocalizerTest, SpinInPlace) {
  output_voltages_ << 4.0, -4.0;
  event_loop_factory_.RunFor(std::chrono::seconds(2));
  CHECK(output_fetcher_.Fetch());
  CHECK(status_fetcher_.Fetch());
  // The two can be different because they may've been sent at different times.
  ASSERT_NEAR(output_fetcher_->x(), status_fetcher_->model_based()->x(), 1e-6);
  ASSERT_NEAR(output_fetcher_->y(), status_fetcher_->model_based()->y(), 1e-6);
  ASSERT_NEAR(output_fetcher_->theta(), status_fetcher_->model_based()->theta(),
              1e-1);
  ASSERT_NEAR(0.0, output_fetcher_->x(), 1e-10);
  ASSERT_NEAR(0.0, output_fetcher_->y(), 1e-10);
  ASSERT_LT(0.1, std::abs(output_fetcher_->theta()));
  ASSERT_TRUE(status_fetcher_->has_model_based());
  ASSERT_TRUE(status_fetcher_->model_based()->using_model());
  ASSERT_NEAR(0.0, status_fetcher_->model_based()->accel_state()->velocity_x(),
              1e-10);
  ASSERT_NEAR(0.0, status_fetcher_->model_based()->accel_state()->velocity_y(),
              1e-10);
  ASSERT_NEAR(-status_fetcher_->model_based()->model_state()->left_velocity(),
              status_fetcher_->model_based()->model_state()->right_velocity(),
              1e-3);
  ASSERT_NEAR(
      0.0, status_fetcher_->model_based()->model_state()->left_voltage_error(),
      1e-1);
  ASSERT_NEAR(
      0.0, status_fetcher_->model_based()->model_state()->right_voltage_error(),
      1e-1);
  ASSERT_NEAR(0.0, status_fetcher_->model_based()->residual(), 1e-3);
}

TEST_F(EventLoopLocalizerTest, Curve) {
  output_voltages_ << 2.0, 4.0;
  event_loop_factory_.RunFor(std::chrono::seconds(2));
  CHECK(output_fetcher_.Fetch());
  CHECK(status_fetcher_.Fetch());
  // The two can be different because they may've been sent at different times.
  ASSERT_NEAR(output_fetcher_->x(), status_fetcher_->model_based()->x(), 1e-2);
  ASSERT_NEAR(output_fetcher_->y(), status_fetcher_->model_based()->y(), 1e-2);
  ASSERT_NEAR(output_fetcher_->theta(), status_fetcher_->model_based()->theta(),
              1e-1);
  ASSERT_LT(0.1, output_fetcher_->x());
  ASSERT_LT(0.1, output_fetcher_->y());
  ASSERT_LT(0.1, std::abs(output_fetcher_->theta()));
  ASSERT_TRUE(status_fetcher_->has_model_based());
  ASSERT_TRUE(status_fetcher_->model_based()->using_model());
  ASSERT_LT(0.0, status_fetcher_->model_based()->accel_state()->velocity_x());
  ASSERT_LT(0.0, status_fetcher_->model_based()->accel_state()->velocity_y());
  ASSERT_NEAR(
      0.0, status_fetcher_->model_based()->model_state()->left_voltage_error(),
      1e-1);
  ASSERT_NEAR(
      0.0, status_fetcher_->model_based()->model_state()->right_voltage_error(),
      1e-1);
  ASSERT_NEAR(0.0, status_fetcher_->model_based()->residual(), 1e-1)
      << aos::FlatbufferToJson(status_fetcher_.get(), {.multi_line = true});
}

// Tests that small amounts of voltage error are handled by the model-based
// half of the localizer.
TEST_F(EventLoopLocalizerTest, VoltageError) {
  output_voltages_ << 0.0, 0.0;
  drivetrain_plant_.set_left_voltage_offset(2.0);
  drivetrain_plant_.set_right_voltage_offset(2.0);
  drivetrain_plant_.set_accel_sin_magnitude(0.01);

  event_loop_factory_.RunFor(std::chrono::seconds(2));
  CHECK(output_fetcher_.Fetch());
  CHECK(status_fetcher_.Fetch());
  // Should still be using the model, but have a non-trivial residual.
  ASSERT_TRUE(status_fetcher_->model_based()->using_model());
  ASSERT_LT(0.02, status_fetcher_->model_based()->residual())
      << aos::FlatbufferToJson(status_fetcher_.get(), {.multi_line = true});

  // Afer running for a while, voltage error terms should converge and result in
  // low residuals.
  event_loop_factory_.RunFor(std::chrono::seconds(10));
  CHECK(output_fetcher_.Fetch());
  CHECK(status_fetcher_.Fetch());
  ASSERT_TRUE(status_fetcher_->model_based()->using_model());
  ASSERT_NEAR(
      2.0, status_fetcher_->model_based()->model_state()->left_voltage_error(),
      0.1)
      << aos::FlatbufferToJson(status_fetcher_.get(), {.multi_line = true});
  ASSERT_NEAR(
      2.0, status_fetcher_->model_based()->model_state()->right_voltage_error(),
      0.1)
      << aos::FlatbufferToJson(status_fetcher_.get(), {.multi_line = true});
  ASSERT_GT(0.02, status_fetcher_->model_based()->residual())
      << aos::FlatbufferToJson(status_fetcher_.get(), {.multi_line = true});
}

// Tests that large amounts of voltage error force us into the
// acceleration-based localizer.
TEST_F(EventLoopLocalizerTest, HighVoltageError) {
  output_voltages_ << 0.0, 0.0;
  drivetrain_plant_.set_left_voltage_offset(200.0);
  drivetrain_plant_.set_right_voltage_offset(200.0);
  drivetrain_plant_.set_accel_sin_magnitude(0.01);

  event_loop_factory_.RunFor(std::chrono::seconds(2));
  CHECK(output_fetcher_.Fetch());
  CHECK(status_fetcher_.Fetch());
  // Should still be using the model, but have a non-trivial residual.
  ASSERT_FALSE(status_fetcher_->model_based()->using_model());
  ASSERT_LT(0.1, status_fetcher_->model_based()->residual())
      << aos::FlatbufferToJson(status_fetcher_.get(), {.multi_line = true});
  ASSERT_NEAR(drivetrain_plant_.state()(0), status_fetcher_->model_based()->x(),
              1.0);
  ASSERT_NEAR(drivetrain_plant_.state()(1), status_fetcher_->model_based()->y(),
              1e-6);
}

// Tests that image corrections in the nominal case (no errors) causes no
// issues.
TEST_F(EventLoopLocalizerTest, NominalImageCorrections) {
  output_voltages_ << 3.0, 2.0;
  drivetrain_plant_.set_accel_sin_magnitude(0.01);
  send_targets_ = true;

  event_loop_factory_.RunFor(std::chrono::seconds(4));
  CHECK(status_fetcher_.Fetch());
  ASSERT_TRUE(status_fetcher_->model_based()->using_model());
  EXPECT_TRUE(VerifyEstimatorAccurate(1e-1));
  ASSERT_TRUE(status_fetcher_->model_based()->has_statistics());
  ASSERT_LT(10,
            status_fetcher_->model_based()->statistics()->total_candidates());
  ASSERT_EQ(status_fetcher_->model_based()->statistics()->total_candidates(),
            status_fetcher_->model_based()->statistics()->total_accepted());
}

// Tests that image corrections when there is an error at the start results
// in us actually getting corrected over time.
TEST_F(EventLoopLocalizerTest, ImageCorrections) {
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
  ASSERT_TRUE(status_fetcher_->model_based()->using_model());
  EXPECT_TRUE(VerifyEstimatorAccurate(1e-1));
  ASSERT_TRUE(status_fetcher_->model_based()->has_statistics());
  ASSERT_LT(10,
            status_fetcher_->model_based()->statistics()->total_candidates());
  ASSERT_EQ(status_fetcher_->model_based()->statistics()->total_candidates(),
            status_fetcher_->model_based()->statistics()->total_accepted());
}

// Tests that image corrections are ignored when the turret moves too fast.
TEST_F(EventLoopLocalizerTest, ImageCorrectionsTurretTooFast) {
  output_voltages_ << 0.0, 0.0;
  drivetrain_plant_.mutable_state()->x() = 2.0;
  drivetrain_plant_.mutable_state()->y() = 2.0;
  SendLocalizerControl(5.0, 3.0, 0.0);
  turret_velocity_ = 10.0;
  event_loop_factory_.RunFor(std::chrono::seconds(4));
  CHECK(output_fetcher_.Fetch());
  ASSERT_NEAR(5.0, output_fetcher_->x(), 1e-5);
  ASSERT_NEAR(3.0, output_fetcher_->y(), 1e-5);
  ASSERT_NEAR(0.0, output_fetcher_->theta(), 1e-5);

  send_targets_ = true;

  event_loop_factory_.RunFor(std::chrono::seconds(4));
  CHECK(status_fetcher_.Fetch());
  CHECK(output_fetcher_.Fetch());
  ASSERT_NEAR(5.0, output_fetcher_->x(), 1e-5);
  ASSERT_NEAR(3.0, output_fetcher_->y(), 1e-5);
  ASSERT_NEAR(0.0, output_fetcher_->theta(), 1e-5);
  ASSERT_TRUE(status_fetcher_->model_based()->has_statistics());
  ASSERT_LT(10,
            status_fetcher_->model_based()->statistics()->total_candidates());
  ASSERT_EQ(0, status_fetcher_->model_based()->statistics()->total_accepted());
  ASSERT_EQ(status_fetcher_->model_based()->statistics()->total_candidates(),
            status_fetcher_->model_based()
                ->statistics()
                ->rejection_reason_count()
                ->Get(static_cast<int>(RejectionReason::TURRET_TOO_FAST)));
  // We expect one more rejection to occur due to the time it takes all the
  // information to propagate.
  const int rejected_count =
      status_fetcher_->model_based()->statistics()->total_candidates() + 1;
  // Check that when we go back to being still we do successfully converge.
  turret_velocity_ = 0.0;
  turret_position_ = 1.0;
  event_loop_factory_.RunFor(std::chrono::seconds(4));
  CHECK(status_fetcher_.Fetch());
  ASSERT_TRUE(status_fetcher_->model_based()->using_model());
  EXPECT_TRUE(VerifyEstimatorAccurate(1e-1));
  ASSERT_TRUE(status_fetcher_->model_based()->has_statistics());
  ASSERT_EQ(status_fetcher_->model_based()->statistics()->total_candidates(),
            rejected_count +
                status_fetcher_->model_based()->statistics()->total_accepted());
}

// Tests that image corrections when we are in accel mode works.
TEST_F(EventLoopLocalizerTest, ImageCorrectionsInAccel) {
  output_voltages_ << 0.0, 0.0;
  drivetrain_plant_.set_left_voltage_offset(200.0);
  drivetrain_plant_.set_right_voltage_offset(200.0);
  drivetrain_plant_.set_accel_sin_magnitude(0.01);
  drivetrain_plant_.mutable_state()->x() = 2.0;
  drivetrain_plant_.mutable_state()->y() = 2.0;
  SendLocalizerControl(6.0, 3.0, 0.0);
  event_loop_factory_.RunFor(std::chrono::seconds(1));
  CHECK(output_fetcher_.Fetch());
  CHECK(status_fetcher_.Fetch());
  ASSERT_FALSE(status_fetcher_->model_based()->using_model());
  EXPECT_FALSE(VerifyEstimatorAccurate(3.0));

  send_targets_ = true;

  event_loop_factory_.RunFor(std::chrono::seconds(4));
  CHECK(status_fetcher_.Fetch());
  ASSERT_FALSE(status_fetcher_->model_based()->using_model());
  EXPECT_TRUE(VerifyEstimatorAccurate(3.0));
  // y should be noticeably more accurate than x, since we are just driving
  // straight.
  ASSERT_NEAR(drivetrain_plant_.state()(1), status_fetcher_->model_based()->y(),
              0.1);
  ASSERT_TRUE(status_fetcher_->model_based()->has_statistics());
  ASSERT_LT(10,
            status_fetcher_->model_based()->statistics()->total_candidates());
  ASSERT_EQ(status_fetcher_->model_based()->statistics()->total_candidates(),
            status_fetcher_->model_based()->statistics()->total_accepted());
}

TEST_F(EventLoopLocalizerTest, LedOutputs) {
  send_targets_ = true;

  event_loop_factory_.RunFor(std::chrono::milliseconds(10));
  CHECK(output_fetcher_.Fetch());
  EXPECT_EQ(output_fetcher_->led_outputs()->size(),
            ModelBasedLocalizer::kNumPis);
  for (LedOutput led_output : *output_fetcher_->led_outputs()) {
    EXPECT_EQ(led_output, LedOutput::ON);
  }
}

}  // namespace frc971::controls::testing
