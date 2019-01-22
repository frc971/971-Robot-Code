#ifndef Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_INTAKE_INTAKE_H_
#define Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_INTAKE_INTAKE_H_

#include "aos/commonmath.h"
#include "aos/controls/control_loop.h"
#include "frc971/control_loops/control_loops.q.h"
#include "frc971/zeroing/zeroing.h"
#include "y2018/constants.h"
#include "y2018/control_loops/superstructure/intake/intake_delayed_plant.h"
#include "y2018/control_loops/superstructure/intake/intake_plant.h"
#include "y2018/control_loops/superstructure/superstructure.q.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {
namespace intake {

class IntakeController {
 public:
  IntakeController();

  // Sets the current encoder position in radians
  void set_position(double spring_angle, double output_position);

  // Populates the status structure.
  void SetStatus(control_loops::IntakeSideStatus *status,
                 const double *unsafe_goal);

  // Returns the control loop calculated voltage.
  double voltage() const;

  double output_position() const { return loop_->X_hat(0); }

  // Executes the control loop for a cycle.
  void Update(bool disabled, const double *unsafe_goal);

  // Resets the kalman filter and any other internal state.
  void Reset();

  // Sets the goal angle from unsafe_goal.
  double goal_angle(const double *unsafe_goal);

  // The control loop.
  ::std::unique_ptr<
      StateFeedbackLoop<5, 1, 2, double, StateFeedbackPlant<5, 1, 2>,
                        StateFeedbackObserver<5, 1, 2>>>
      loop_;

  constexpr static double kDt =
      ::std::chrono::duration_cast<::std::chrono::duration<double>>(
          ::aos::controls::kLoopFrequency)
          .count();

  // Sets the offset of the controller to be the zeroing estimator offset when
  // possible otherwise zero.
  void UpdateOffset(double offset);

  const ::frc971::constants::Range intake_range_;

  // Stores the current zeroing estimator offset.
  double offset_ = 0.0;

 private:
  bool reset_ = true;

  // The current sensor measurement.
  Eigen::Matrix<double, 2, 1> Y_;

  DISALLOW_COPY_AND_ASSIGN(IntakeController);
};

class IntakeSide {
 public:
  IntakeSide(const ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants
                 &zeroing_constants);

  // The operating voltage.
  static constexpr double kOperatingVoltage() { return 12.0; }

  void Iterate(const double *unsafe_goal,
               const control_loops::IntakeElasticSensors *position,
               control_loops::IntakeVoltage *output,
               control_loops::IntakeSideStatus *status);

  void Reset();

  enum class State : int32_t {
    UNINITIALIZED,
    ZEROING,
    RUNNING,
    ESTOP,
  };

  State state() const { return state_; }

  bool clear_of_box() const {
    return controller_.output_position() < -0.1;
  }

  double output_position() const { return controller_.output_position(); }

 private:
  IntakeController controller_;

  State state_ = State::UNINITIALIZED;

  ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator zeroing_estimator_;

  double intake_last_position_ = 0.0;
};

}  // namespace intake
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018

#endif  // Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_INTAKE_INTAKE_H_
