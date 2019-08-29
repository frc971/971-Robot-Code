#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_TEST_LIB_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_TEST_LIB_H_

#include "aos/events/event_loop.h"
#include "frc971/control_loops/control_loops_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_position_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/queues/gyro_generated.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace testing {

const DrivetrainConfig<double> &GetTestDrivetrainConfig();

class DrivetrainPlant : public StateFeedbackPlant<4, 2, 2> {
 public:
  explicit DrivetrainPlant(StateFeedbackPlant<4, 2, 2> &&other)
      : StateFeedbackPlant<4, 2, 2>(::std::move(other)) {}

  void CheckU(const Eigen::Matrix<double, 2, 1> &U) override;

  double left_voltage_offset() const { return left_voltage_offset_; }
  void set_left_voltage_offset(double left_voltage_offset) {
    left_voltage_offset_ = left_voltage_offset;
  }

  double right_voltage_offset() const { return right_voltage_offset_; }
  void set_right_voltage_offset(double right_voltage_offset) {
    right_voltage_offset_ = right_voltage_offset;
  }

 private:
  double left_voltage_offset_ = 0.0;
  double right_voltage_offset_ = 0.0;
};

// Class which simulates the drivetrain and sends out queue messages containing
// the position.
class DrivetrainSimulation {
 public:
  // Constructs a motor simulation.
  // TODO(aschuh) Do we want to test the clutch one too?
  DrivetrainSimulation(::aos::EventLoop *event_loop,
                       const DrivetrainConfig<double> &dt_config);

  // Resets the plant.
  void Reinitialize();

  // Returns the position of the drivetrain.
  double GetLeftPosition() const { return drivetrain_plant_.Y(0, 0); }
  double GetRightPosition() const { return drivetrain_plant_.Y(1, 0); }

  void set_left_voltage_offset(double left_voltage_offset) {
    drivetrain_plant_.set_left_voltage_offset(left_voltage_offset);
  }
  void set_right_voltage_offset(double right_voltage_offset) {
    drivetrain_plant_.set_right_voltage_offset(right_voltage_offset);
  }

  Eigen::Matrix<double, 5, 1> state() const { return state_; }

  Eigen::Matrix<double, 5, 1> *mutable_state() { return &state_; }

  ::Eigen::Matrix<double, 2, 1> GetPosition() const {
    return state_.block<2,1>(0,0);
  }

  void MaybePlot();

 private:
  // Sends out the position queue messages.
  void SendPositionMessage();

  // Simulates the drivetrain moving for one timestep.
  void Simulate();

  ::aos::EventLoop *event_loop_;
  ::aos::Fetcher<::aos::RobotState> robot_state_fetcher_;

  ::aos::Sender<::frc971::control_loops::drivetrain::Position>
      drivetrain_position_sender_;
  ::aos::Fetcher<::frc971::control_loops::drivetrain::Output>
      drivetrain_output_fetcher_;
  ::aos::Fetcher<::frc971::control_loops::drivetrain::Status>
      drivetrain_status_fetcher_;
  ::aos::Sender<::frc971::sensors::GyroReading> gyro_reading_sender_;

  DrivetrainConfig<double> dt_config_;

  DrivetrainPlant drivetrain_plant_;

  // This state is [x, y, theta, left_velocity, right_velocity].
  ::Eigen::Matrix<double, 5, 1> state_ = ::Eigen::Matrix<double, 5, 1>::Zero();
  ::std::unique_ptr<
      StateFeedbackLoop<2, 2, 2, double, StateFeedbackHybridPlant<2, 2, 2>,
                        HybridKalman<2, 2, 2>>>
      velocity_drivetrain_;
  double last_left_position_;
  double last_right_position_;

  Eigen::Matrix<double, 2, 1> last_U_;

  bool left_gear_high_ = false;
  bool right_gear_high_ = false;
  bool first_ = true;

  ::std::vector<double> actual_x_;
  ::std::vector<double> actual_y_;
  ::std::vector<double> trajectory_x_;
  ::std::vector<double> trajectory_y_;
};

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_TEST_LIB_H_
