#include "frc971/control_loops/drivetrain/drivetrain_test_lib.h"

#include <chrono>

#include "gtest/gtest.h"

#include "frc971/control_loops/drivetrain/trajectory.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "gflags/gflags.h"
#if defined(SUPPORT_PLOT)
#include "third_party/matplotlib-cpp/matplotlibcpp.h"
#endif
#include "y2016/constants.h"
#include "y2016/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2016/control_loops/drivetrain/hybrid_velocity_drivetrain.h"
#include "y2016/control_loops/drivetrain/kalman_drivetrain_motor_plant.h"
#include "y2016/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

DEFINE_bool(plot, false, "If true, plot");

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace testing {

namespace {
// TODO(Comran): Make one that doesn't depend on the actual values for a
// specific robot.
const constants::ShifterHallEffect kThreeStateDriveShifter{0.0, 0.0, 0.25,
                                                           0.75};

StateFeedbackPlant<4, 2, 2, double> MakePlantFromConfig(
    const DrivetrainConfig<double> &dt_config) {
  ::std::vector<
      ::std::unique_ptr<StateFeedbackPlantCoefficients<4, 2, 2, double>>>
      coefs;
  for (size_t ii = 0;
       ii < dt_config.make_drivetrain_loop().plant().coefficients_size();
       ++ii) {
    coefs.emplace_back(new StateFeedbackPlantCoefficients<4, 2, 2, double>(
        dt_config.make_drivetrain_loop().plant().coefficients(ii)));
  }
  return StateFeedbackPlant<4, 2, 2, double>(&coefs);
}

}  // namespace

namespace chrono = ::std::chrono;

const DrivetrainConfig<double> &GetTestDrivetrainConfig() {
  static DrivetrainConfig<double> kDrivetrainConfig{
      ::frc971::control_loops::drivetrain::ShifterType::HALL_EFFECT_SHIFTER,
      ::frc971::control_loops::drivetrain::LoopType::CLOSED_LOOP,
      ::frc971::control_loops::drivetrain::GyroType::SPARTAN_GYRO,
      IMUType::IMU_X,
      ::y2016::control_loops::drivetrain::MakeDrivetrainLoop,
      ::y2016::control_loops::drivetrain::MakeVelocityDrivetrainLoop,
      ::y2016::control_loops::drivetrain::MakeKFDrivetrainLoop,
      ::y2016::control_loops::drivetrain::MakeHybridVelocityDrivetrainLoop,

      chrono::duration_cast<chrono::nanoseconds>(
          chrono::duration<double>(::y2016::control_loops::drivetrain::kDt)),
      ::y2016::control_loops::drivetrain::kRobotRadius,
      ::y2016::control_loops::drivetrain::kWheelRadius,
      ::y2016::control_loops::drivetrain::kV,

      ::y2016::control_loops::drivetrain::kHighGearRatio,
      ::y2016::control_loops::drivetrain::kLowGearRatio,
      ::y2016::control_loops::drivetrain::kJ,
      ::y2016::control_loops::drivetrain::kMass,
      kThreeStateDriveShifter,
      kThreeStateDriveShifter,
      false,
      0,

      0.25,
      1.00,
      1.00};

  return kDrivetrainConfig;
};

void DrivetrainPlant::CheckU(const Eigen::Matrix<double, 2, 1> &U) {
  EXPECT_LE(U(0, 0), U_max(0, 0) + 0.00001 + left_voltage_offset_);
  EXPECT_GE(U(0, 0), U_min(0, 0) - 0.00001 + left_voltage_offset_);
  EXPECT_LE(U(1, 0), U_max(1, 0) + 0.00001 + right_voltage_offset_);
  EXPECT_GE(U(1, 0), U_min(1, 0) - 0.00001 + right_voltage_offset_);
}

DrivetrainSimulation::DrivetrainSimulation(
    ::aos::EventLoop *event_loop, const DrivetrainConfig<double> &dt_config)
    : event_loop_(event_loop),
      robot_state_fetcher_(event_loop_->MakeFetcher<::aos::RobotState>("/aos")),
      drivetrain_position_sender_(
          event_loop_
              ->MakeSender<::frc971::control_loops::drivetrain::Position>(
                  "/drivetrain")),
      drivetrain_output_fetcher_(
          event_loop_->MakeFetcher<::frc971::control_loops::drivetrain::Output>(
              "/drivetrain")),
      drivetrain_status_fetcher_(
          event_loop_->MakeFetcher<::frc971::control_loops::drivetrain::Status>(
              "/drivetrain")),
      gyro_reading_sender_(
          event_loop->MakeSender<::frc971::sensors::GyroReading>(
              "/drivetrain")),
      dt_config_(dt_config),
      drivetrain_plant_(MakePlantFromConfig(dt_config_)),
      velocity_drivetrain_(
          ::std::unique_ptr<StateFeedbackLoop<2, 2, 2, double,
                                              StateFeedbackHybridPlant<2, 2, 2>,
                                              HybridKalman<2, 2, 2>>>(
              new StateFeedbackLoop<2, 2, 2, double,
                                    StateFeedbackHybridPlant<2, 2, 2>,
                                    HybridKalman<2, 2, 2>>(
                  dt_config_.make_hybrid_drivetrain_velocity_loop()))) {
  Reinitialize();
  last_U_.setZero();

  event_loop_->AddPhasedLoop(
      [this](int) {
        // Skip this the first time.
        if (!first_) {
          Simulate();
          if (FLAGS_plot) {
            EXPECT_TRUE(drivetrain_status_fetcher_.Fetch());

            ::Eigen::Matrix<double, 2, 1> actual_position = GetPosition();
            actual_x_.push_back(actual_position(0));
            actual_y_.push_back(actual_position(1));

            trajectory_x_.push_back(
                drivetrain_status_fetcher_->trajectory_logging()->x());
            trajectory_y_.push_back(
                drivetrain_status_fetcher_->trajectory_logging()->y());
          }
        }
        first_ = false;
        SendPositionMessage();
      },
      dt_config_.dt);
}

void DrivetrainSimulation::Reinitialize() {
  drivetrain_plant_.mutable_X(0, 0) = 0.0;
  drivetrain_plant_.mutable_X(1, 0) = 0.0;
  drivetrain_plant_.mutable_X(2, 0) = 0.0;
  drivetrain_plant_.mutable_X(3, 0) = 0.0;
  drivetrain_plant_.mutable_Y() = drivetrain_plant_.C() * drivetrain_plant_.X();
  last_left_position_ = drivetrain_plant_.Y(0, 0);
  last_right_position_ = drivetrain_plant_.Y(1, 0);
}

void DrivetrainSimulation::SendPositionMessage() {
  const double left_encoder = GetLeftPosition();
  const double right_encoder = GetRightPosition();

  {
    ::aos::Sender<::frc971::control_loops::drivetrain::Position>::Builder
        builder = drivetrain_position_sender_.MakeBuilder();
    frc971::control_loops::drivetrain::Position::Builder position_builder =
        builder.MakeBuilder<frc971::control_loops::drivetrain::Position>();
    position_builder.add_left_encoder(left_encoder);
    position_builder.add_right_encoder(right_encoder);
    position_builder.add_left_shifter_position(left_gear_high_ ? 1.0 : 0.0);
    position_builder.add_right_shifter_position(right_gear_high_ ? 1.0 : 0.0);
    builder.Send(position_builder.Finish());
  }

  {
    auto builder = gyro_reading_sender_.MakeBuilder();
    frc971::sensors::GyroReading::Builder gyro_builder =
        builder.MakeBuilder<frc971::sensors::GyroReading>();
    gyro_builder.add_angle((right_encoder - left_encoder) /
                           (dt_config_.robot_radius * 2.0));
    gyro_builder.add_velocity(
        (drivetrain_plant_.X(3, 0) - drivetrain_plant_.X(1, 0)) /
        (dt_config_.robot_radius * 2.0));
    builder.Send(gyro_builder.Finish());
  }
}

// Simulates the drivetrain moving for one timestep.
void DrivetrainSimulation::Simulate() {
  last_left_position_ = drivetrain_plant_.Y(0, 0);
  last_right_position_ = drivetrain_plant_.Y(1, 0);
  EXPECT_TRUE(drivetrain_output_fetcher_.Fetch());
  ::Eigen::Matrix<double, 2, 1> U = last_U_;
  last_U_ << drivetrain_output_fetcher_->left_voltage(),
      drivetrain_output_fetcher_->right_voltage();
  {
    robot_state_fetcher_.Fetch();
    const double scalar = robot_state_fetcher_.get()
                              ? robot_state_fetcher_->voltage_battery() / 12.0
                              : 1.0;
    last_U_ *= scalar;
  }
  left_gear_high_ = drivetrain_output_fetcher_->left_high();
  right_gear_high_ = drivetrain_output_fetcher_->right_high();

  if (left_gear_high_) {
    if (right_gear_high_) {
      drivetrain_plant_.set_index(3);
    } else {
      drivetrain_plant_.set_index(2);
    }
  } else {
    if (right_gear_high_) {
      drivetrain_plant_.set_index(1);
    } else {
      drivetrain_plant_.set_index(0);
    }
  }

  U(0, 0) += drivetrain_plant_.left_voltage_offset();
  U(1, 0) += drivetrain_plant_.right_voltage_offset();
  drivetrain_plant_.Update(U);
  double dt_float = ::aos::time::DurationInSeconds(dt_config_.dt);
  state_ = RungeKuttaU(
      [this](const ::Eigen::Matrix<double, 5, 1> &X,
             const ::Eigen::Matrix<double, 2, 1> &U) {
        return ContinuousDynamics(velocity_drivetrain_->plant(),
                                  dt_config_.Tlr_to_la(), X, U);
      },
      state_, U, dt_float);
}

void DrivetrainSimulation::MaybePlot() {
#if defined(SUPPORT_PLOT)
  if (FLAGS_plot) {
    std::cout << "Plotting." << ::std::endl;
    matplotlibcpp::figure();
    matplotlibcpp::plot(actual_x_, actual_y_, {{"label", "actual position"}});
    matplotlibcpp::plot(trajectory_x_, trajectory_y_,
                        {{"label", "trajectory position"}});
    matplotlibcpp::legend();
    matplotlibcpp::show();
  }
#endif
}

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
