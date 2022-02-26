#include "y2022/localizer/localizer.h"

#include "aos/events/simulated_event_loop.h"
#include "gtest/gtest.h"
#include "frc971/control_loops/drivetrain/drivetrain_test_lib.h"

namespace frc971::controls::testing {
typedef ModelBasedLocalizer::ModelState ModelState;
typedef ModelBasedLocalizer::AccelState AccelState;
typedef ModelBasedLocalizer::ModelInput ModelInput;
typedef ModelBasedLocalizer::AccelInput AccelInput;
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
}

class LocalizerTest : public ::testing::Test {
 protected:
  LocalizerTest()
      : dt_config_(
            control_loops::drivetrain::testing::GetTestDrivetrainConfig()),
        localizer_(dt_config_) {
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
  Eigen::Vector3d accel_gs = accel / 9.80665;
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
  accel_gs = accel / 9.80665;
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
        logger_test_event_loop_(
            event_loop_factory_.GetNodeEventLoopFactory("logger")
                ->MakeEventLoop("test")),
        output_sender_(
            roborio_test_event_loop_->MakeSender<Output>("/drivetrain")),
        output_fetcher_(roborio_test_event_loop_->MakeFetcher<LocalizerOutput>(
            "/localizer")),
        status_fetcher_(
            imu_test_event_loop_->MakeFetcher<LocalizerStatus>("/localizer")) {
    localizer_.localizer()->set_longitudinal_offset(0.0);
    aos::TimerHandler *timer = roborio_test_event_loop_->AddTimer([this]() {
      auto builder = output_sender_.MakeBuilder();
      auto output_builder = builder.MakeBuilder<Output>();
      output_builder.add_left_voltage(output_voltages_(0));
      output_builder.add_right_voltage(output_voltages_(1));
      builder.CheckOk(builder.Send(output_builder.Finish()));
    });
    roborio_test_event_loop_->OnRun([timer, this]() {
      timer->Setup(roborio_test_event_loop_->monotonic_now(),
                   std::chrono::milliseconds(5));
    });
    // Get things zeroed.
    event_loop_factory_.RunFor(std::chrono::seconds(10));
    CHECK(status_fetcher_.Fetch());
    CHECK(status_fetcher_->zeroed());
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> configuration_;
  aos::SimulatedEventLoopFactory event_loop_factory_;
  const aos::Node *const roborio_node_;
  const aos::Node *const imu_node_;
  const control_loops::drivetrain::DrivetrainConfig<double> dt_config_;
  std::unique_ptr<aos::EventLoop> localizer_event_loop_;
  EventLoopLocalizer localizer_;

  std::unique_ptr<aos::EventLoop> drivetrain_plant_event_loop_;
  std::unique_ptr<aos::EventLoop> drivetrain_plant_imu_event_loop_;
  control_loops::drivetrain::testing::DrivetrainSimulation drivetrain_plant_;

  std::unique_ptr<aos::EventLoop> roborio_test_event_loop_;
  std::unique_ptr<aos::EventLoop> imu_test_event_loop_;
  std::unique_ptr<aos::EventLoop> logger_test_event_loop_;

  aos::Sender<Output> output_sender_;
  aos::Fetcher<LocalizerOutput> output_fetcher_;
  aos::Fetcher<LocalizerStatus> status_fetcher_;

  Eigen::Vector2d output_voltages_ = Eigen::Vector2d::Zero();
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
  ASSERT_NEAR(drivetrain_plant_.state()(0),
              status_fetcher_->model_based()->x(), 1.0);
  ASSERT_NEAR(drivetrain_plant_.state()(1),
              status_fetcher_->model_based()->y(), 1e-6);
}

}  // namespace frc91::controls::testing
