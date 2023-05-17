#include "frc971/control_loops/drivetrain/localization/puppet_localizer.h"

#include <queue>

#include "aos/events/logging/log_writer.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/network/team_number.h"
#include "aos/network/testing_time_converter.h"
#include "frc971/control_loops/control_loop_test.h"
#include "frc971/control_loops/drivetrain/drivetrain.h"
#include "frc971/control_loops/team_number_test_environment.h"
#include "gtest/gtest.h"
#include "frc971/control_loops/drivetrain/localization/localizer_output_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_test_lib.h"
#include "y2022/control_loops/drivetrain/drivetrain_base.h"

DEFINE_string(output_folder, "",
              "If set, logs all channels to the provided logfile.");

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace testing {

using frc971::control_loops::drivetrain::DrivetrainConfig;
using frc971::control_loops::drivetrain::Goal;
using frc971::control_loops::drivetrain::LocalizerControl;

namespace {
DrivetrainConfig<double> GetTest2022DrivetrainConfig() {
  DrivetrainConfig<double> config =
      y2022::control_loops::drivetrain::GetDrivetrainConfig();
  return config;
}
}  // namespace

namespace chrono = std::chrono;
using aos::monotonic_clock;
using frc971::control_loops::drivetrain::DrivetrainLoop;
using frc971::control_loops::drivetrain::testing::DrivetrainSimulation;


// TODO(james): Make it so this actually tests the full system of the localizer.
class LocalizedDrivetrainTest : public frc971::testing::ControlLoopTest {
 protected:
  // We must use the 2022 drivetrain config so that we actually have a multi-nde
  // config with a LocalizerOutput message.
  // TODO(james): Refactor this test to be year-agnostic.
  LocalizedDrivetrainTest()
      : frc971::testing::ControlLoopTest(
            aos::configuration::ReadConfig(
                "y2022/control_loops/drivetrain/simulation_config.json"),
            GetTest2022DrivetrainConfig().dt),
        roborio_(aos::configuration::GetNode(configuration(), "roborio")),
        imu_(aos::configuration::GetNode(configuration(), "imu")),
        test_event_loop_(MakeEventLoop("test", roborio_)),
        imu_test_event_loop_(MakeEventLoop("test", imu_)),
        drivetrain_goal_sender_(
            test_event_loop_->MakeSender<Goal>("/drivetrain")),
        localizer_output_sender_(
            imu_test_event_loop_->MakeSender<frc971::controls::LocalizerOutput>(
                "/localizer")),
        drivetrain_goal_fetcher_(
            test_event_loop_->MakeFetcher<Goal>("/drivetrain")),
        drivetrain_status_fetcher_(
            test_event_loop_
                ->MakeFetcher<frc971::control_loops::drivetrain::Status>(
                    "/drivetrain")),
        localizer_control_sender_(
            test_event_loop_->MakeSender<LocalizerControl>("/drivetrain")),
        drivetrain_event_loop_(MakeEventLoop("drivetrain", roborio_)),
        dt_config_(GetTest2022DrivetrainConfig()),
        localizer_(drivetrain_event_loop_.get(), dt_config_),
        drivetrain_(dt_config_, drivetrain_event_loop_.get(), &localizer_),
        drivetrain_plant_event_loop_(MakeEventLoop("plant", roborio_)),
        drivetrain_plant_imu_event_loop_(MakeEventLoop("plant", imu_)),
        drivetrain_plant_(drivetrain_plant_event_loop_.get(),
                          drivetrain_plant_imu_event_loop_.get(), dt_config_,
                          std::chrono::microseconds(500)) {
    set_team_id(frc971::control_loops::testing::kTeamNumber);
    set_battery_voltage(12.0);

    if (!FLAGS_output_folder.empty()) {
      logger_event_loop_ = MakeEventLoop("logger", roborio_);
      logger_ = std::make_unique<aos::logger::Logger>(logger_event_loop_.get());
      logger_->StartLoggingOnRun(FLAGS_output_folder);
    }

    test_event_loop_->OnRun([this]() { SetStartingPosition({3.0, 2.0, 0.0}); });

    imu_test_event_loop_
        ->AddTimer([this]() {
          auto builder = localizer_output_sender_.MakeBuilder();
          frc971::controls::LocalizerOutput::Builder output_builder =
              builder.MakeBuilder<frc971::controls::LocalizerOutput>();
          output_builder.add_monotonic_timestamp_ns(
              imu_test_event_loop_->monotonic_now().time_since_epoch().count());
          output_builder.add_x(drivetrain_plant_.state()(0));
          output_builder.add_y(drivetrain_plant_.state()(1));
          output_builder.add_theta(drivetrain_plant_.state()(2));
          builder.CheckOk(builder.Send(output_builder.Finish()));
        })
        ->Setup(imu_test_event_loop_->monotonic_now(),
                std::chrono::milliseconds(5));
  }

  virtual ~LocalizedDrivetrainTest() override {}

  void SetStartingPosition(const Eigen::Matrix<double, 3, 1> &xytheta) {
    *drivetrain_plant_.mutable_state() << xytheta.x(), xytheta.y(),
        xytheta(2, 0), 0.0, 0.0;
    Eigen::Matrix<double, PuppetLocalizer::HybridEkf::kNStates, 1>
        localizer_state;
    localizer_state.setZero();
    localizer_state.block<3, 1>(0, 0) = xytheta;
    localizer_.Reset(monotonic_now(), localizer_state);
  }

  void VerifyNearGoal(double eps = 1e-2) {
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

  const aos::Node *const roborio_;
  const aos::Node *const imu_;

  std::unique_ptr<aos::EventLoop> test_event_loop_;
  std::unique_ptr<aos::EventLoop> imu_test_event_loop_;
  aos::Sender<Goal> drivetrain_goal_sender_;
  aos::Sender<frc971::controls::LocalizerOutput> localizer_output_sender_;
  aos::Fetcher<Goal> drivetrain_goal_fetcher_;
  aos::Fetcher<frc971::control_loops::drivetrain::Status>
      drivetrain_status_fetcher_;
  aos::Sender<LocalizerControl> localizer_control_sender_;

  std::unique_ptr<aos::EventLoop> drivetrain_event_loop_;
  const frc971::control_loops::drivetrain::DrivetrainConfig<double> dt_config_;

  PuppetLocalizer localizer_;
  DrivetrainLoop drivetrain_;

  std::unique_ptr<aos::EventLoop> drivetrain_plant_event_loop_;
  std::unique_ptr<aos::EventLoop> drivetrain_plant_imu_event_loop_;
  DrivetrainSimulation drivetrain_plant_;

  void SendGoal(double left, double right) {
    auto builder = drivetrain_goal_sender_.MakeBuilder();

    Goal::Builder drivetrain_builder = builder.MakeBuilder<Goal>();
    drivetrain_builder.add_controller_type(
        frc971::control_loops::drivetrain::ControllerType::MOTION_PROFILE);
    drivetrain_builder.add_left_goal(left);
    drivetrain_builder.add_right_goal(right);

    EXPECT_EQ(builder.Send(drivetrain_builder.Finish()),
              aos::RawSender::Error::kOk);
  }

 private:
  std::unique_ptr<aos::EventLoop> logger_event_loop_;
  std::unique_ptr<aos::logger::Logger> logger_;
};

TEST_F(LocalizedDrivetrainTest, Nominal) {
  SetEnabled(true);
  EXPECT_TRUE(VerifyEstimatorAccurate(1e-7));

  SendGoal(-1.0, 1.0);

  RunFor(chrono::seconds(10));
  VerifyNearGoal();
  EXPECT_TRUE(VerifyEstimatorAccurate(5e-3));
}

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
