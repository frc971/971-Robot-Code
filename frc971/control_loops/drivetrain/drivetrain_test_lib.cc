#include "frc971/control_loops/drivetrain/drivetrain_test_lib.h"

#include <chrono>

#include "gtest/gtest.h"

#include "frc971/control_loops/drivetrain/trajectory.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "y2016/constants.h"
#include "y2016/control_loops/drivetrain/drivetrain_dog_motor_plant.h"
#include "y2016/control_loops/drivetrain/hybrid_velocity_drivetrain.h"
#include "y2016/control_loops/drivetrain/kalman_drivetrain_motor_plant.h"
#include "y2016/control_loops/drivetrain/polydrivetrain_dog_motor_plant.h"

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
      robot_state_fetcher_(
          event_loop_->MakeFetcher<::aos::RobotState>(".aos.robot_state")),
      dt_config_(dt_config),
      drivetrain_plant_(MakePlantFromConfig(dt_config_)),
      my_drivetrain_queue_(".frc971.control_loops.drivetrain_queue",
                           ".frc971.control_loops.drivetrain_queue.goal",
                           ".frc971.control_loops.drivetrain_queue.position",
                           ".frc971.control_loops.drivetrain_queue.output",
                           ".frc971.control_loops.drivetrain_queue.status"),
      gyro_reading_sender_(
          event_loop->MakeSender<::frc971::sensors::GyroReading>(
              ".frc971.sensors.gyro_reading")),
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
    ::aos::ScopedMessagePtr<::frc971::control_loops::DrivetrainQueue::Position>
        position = my_drivetrain_queue_.position.MakeMessage();
    position->left_encoder = left_encoder;
    position->right_encoder = right_encoder;
    position->left_shifter_position = left_gear_high_ ? 1.0 : 0.0;
    position->right_shifter_position = right_gear_high_ ? 1.0 : 0.0;
    position.Send();
  }

  {
    auto gyro = gyro_reading_sender_.MakeMessage();
    gyro->angle =
        (right_encoder - left_encoder) / (dt_config_.robot_radius * 2.0);
    gyro->velocity = (drivetrain_plant_.X(3, 0) - drivetrain_plant_.X(1, 0)) /
                     (dt_config_.robot_radius * 2.0);
    gyro.Send();
  }
}

// Simulates the drivetrain moving for one timestep.
void DrivetrainSimulation::Simulate() {
  last_left_position_ = drivetrain_plant_.Y(0, 0);
  last_right_position_ = drivetrain_plant_.Y(1, 0);
  EXPECT_TRUE(my_drivetrain_queue_.output.FetchLatest());
  ::Eigen::Matrix<double, 2, 1> U = last_U_;
  last_U_ << my_drivetrain_queue_.output->left_voltage,
      my_drivetrain_queue_.output->right_voltage;
  {
    robot_state_fetcher_.Fetch();
    const double scalar = robot_state_fetcher_.get()
                              ? robot_state_fetcher_->voltage_battery / 12.0
                              : 1.0;
    last_U_ *= scalar;
  }
  left_gear_high_ = my_drivetrain_queue_.output->left_high;
  right_gear_high_ = my_drivetrain_queue_.output->right_high;

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

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
