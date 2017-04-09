#ifndef Y2017_CONTROL_LOOPS_SHOOTER_SHOOTER_H_
#define Y2017_CONTROL_LOOPS_SHOOTER_SHOOTER_H_

#include <array>
#include <memory>

#include "aos/common/controls/control_loop.h"
#include "aos/common/time.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "third_party/eigen/Eigen/Dense"

#include "y2017/control_loops/superstructure/shooter/shooter_integral_plant.h"
#include "y2017/control_loops/superstructure/superstructure.q.h"

namespace y2017 {
namespace control_loops {
namespace superstructure {
namespace shooter {

class ShooterController {
 public:
  ShooterController();

  // Sets the velocity goal in radians/sec
  void set_goal(double angular_velocity_goal);
  // Sets the current encoder position in radians
  void set_position(double current_position);

  // Populates the status structure.
  void SetStatus(control_loops::ShooterStatus *status);

  // Returns the control loop calculated voltage.
  double voltage() const;

  // Returns the instantaneous velocity.
  double velocity() const { return loop_->X_hat(2, 0); }
  double voltage_error() const { return loop_->X_hat(3, 0); }

  double dt_velocity() const { return dt_velocity_; }

  double error() const { return error_; }

  // Executes the control loop for a cycle.
  void Update(bool disabled, ::std::chrono::nanoseconds dt);

  // Resets the kalman filter and any other internal state.
  void Reset();

 private:
  // The current sensor measurement.
  Eigen::Matrix<double, 1, 1> Y_;
  // The control loop.
  ::std::unique_ptr<StateFeedbackLoop<
      4, 1, 1, StateFeedbackHybridPlant<4, 1, 1>, HybridKalman<4, 1, 1>>>
      loop_;

  // History array for calculating a filtered angular velocity.
  static constexpr int kHistoryLength = 5;
  ::std::array<double, kHistoryLength> history_;
  ptrdiff_t history_position_ = 0;

  double error_ = 0.0;
  double dt_position_ = 0.0;
  double dt_velocity_ = 0.0;
  double fixed_dt_velocity_ = 0.0;
  double last_position_ = 0.0;
  double average_angular_velocity_ = 0.0;
  double min_velocity_ = 0.0;
  double position_error_ = 0.0;

  Eigen::Matrix<double, 4, 1> X_hat_current_;

  bool ready_ = false;
  bool needs_reset_ = false;
  bool reset_ = false;

  bool last_ready_ = false;
  DISALLOW_COPY_AND_ASSIGN(ShooterController);
};

class Shooter {
 public:
  Shooter() {}

  // Iterates the shooter control loop one cycle.  position and status must
  // never be nullptr.  goal can be nullptr if no goal exists, and output should
  // be nullptr if disabled.
  void Iterate(const control_loops::ShooterGoal *goal, const double *position,
               ::aos::monotonic_clock::time_point position_time, double *output,
               control_loops::ShooterStatus *status);

  // Sets the shooter up to reset the kalman filter next time Iterate is called.
  void Reset();

 private:
  ShooterController wheel_;

  bool last_ready_ = false;
  double min_ = 0.0;
  ::aos::monotonic_clock::time_point last_time_ =
      ::aos::monotonic_clock::min_time;

  DISALLOW_COPY_AND_ASSIGN(Shooter);
};

}  // namespace shooter
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2017

#endif  // Y2017_CONTROL_LOOPS_SHOOTER_SHOOTER_H_
