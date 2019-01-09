#ifndef Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_INTAKE_INTAKE_H_
#define Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_INTAKE_INTAKE_H_

#include <math.h>

#include "aos/commonmath.h"
#include "aos/controls/control_loop.h"
#include "frc971/zeroing/zeroing.h"
#include "y2018/constants.h"
#include "y2018/control_loops/superstructure/intake/intake_delayed_plant.h"
#include "y2018/control_loops/superstructure/intake/intake_plant.h"
#include "y2018/control_loops/superstructure/intake/sensor_unwrap.h"
#include "y2018/control_loops/superstructure/superstructure_output_generated.h"
#include "y2018/control_loops/superstructure/superstructure_position_generated.h"
#include "y2018/control_loops/superstructure/superstructure_status_generated.h"

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
  void SetStatus(superstructure::IntakeSideStatus::Builder *status,
                 const double *unsafe_goal);

  // Returns the control loop calculated voltage.
  double voltage() const;

  double output_position() const { return loop_->X_hat(0); }
  double motor_position() const { return loop_->X_hat(2); }

  // Executes the control loop for a cycle.
  void Update(bool disabled, const double *unsafe_goal);

  // Resets the kalman filter and any other internal state.
  void Reset();

  // Sets the goal angle from unsafe_goal.
  double goal_angle(const double *unsafe_goal);

  constexpr static double kDt =
      ::aos::time::DurationInSeconds(::aos::controls::kLoopFrequency);

  // Sets the offset of the controller to be the zeroing estimator offset when
  // possible otherwise zero.
  void UpdateOffset(double offset);

  const ::frc971::constants::Range intake_range() const {
    return intake_range_;
  }

 private:
  // The control loop.
  ::std::unique_ptr<
      StateFeedbackLoop<5, 1, 2, double, StateFeedbackPlant<5, 1, 2>,
                        StateFeedbackObserver<5, 1, 2>>>
      loop_;

  const ::frc971::constants::Range intake_range_;

  // Stores the current zeroing estimator offset.
  double offset_ = 0.0;

  bool reset_ = true;

  // The current sensor measurement.
  Eigen::Matrix<double, 2, 1> Y_;

  DISALLOW_COPY_AND_ASSIGN(IntakeController);
};

class IntakeSide {
 public:
  IntakeSide(const ::frc971::constants::PotAndAbsoluteEncoderZeroingConstants
                 &zeroing_constants,
             const double spring_offset);

  // The operating voltage.
  static constexpr double kOperatingVoltage() { return 12.0; }

  flatbuffers::Offset<superstructure::IntakeSideStatus> Iterate(
      const double *unsafe_goal,
      const superstructure::IntakeElasticSensors *position,
      superstructure::IntakeVoltageT *output,
      flatbuffers::FlatBufferBuilder *fbb);

  void Reset();

  enum class State : int32_t {
    UNINITIALIZED,
    ZEROING,
    RUNNING,
    ESTOP,
  };

  State state() const { return state_; }

  bool estopped() const { return state_ == State::ESTOP; }

  bool zeroed() const { return zeroing_estimator_.zeroed(); }

  bool clear_of_box() const { return controller_.output_position() < -0.1; }

  double output_position() const { return controller_.output_position(); }

 private:
  IntakeController controller_;

  ::frc971::zeroing::PotAndAbsoluteEncoderZeroingEstimator zeroing_estimator_;

  const double spring_offset_;

  double spring_range() const {
    return ::y2018::constants::Values::kIntakeSpringRatio() * (2 * M_PI);
  }

  UnwrapSensor spring_unwrap_{spring_offset_, spring_range()};

  State state_ = State::UNINITIALIZED;

  double intake_last_position_ = 0.0;
};

}  // namespace intake
}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018

#endif  // Y2018_CONTROL_LOOPS_SUPERSTRUCTURE_INTAKE_INTAKE_H_
