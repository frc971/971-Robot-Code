#ifndef FRC971_CONTROL_LOOPS_STATIC_ZEROING_SINGLE_DOF_PROFILED_SUBSYSTEM_H_
#define FRC971_CONTROL_LOOPS_STATIC_ZEROING_SINGLE_DOF_PROFILED_SUBSYSTEM_H_

#include "frc971/control_loops/profiled_subsystem.h"

namespace frc971 {
namespace control_loops {

struct StaticZeroingSingleDOFProfiledSubsystemParams {
  // Maximum voltage while the subsystem is zeroing
  double zeroing_voltage;

  // Maximum voltage while the subsystem is running
  double operating_voltage;

  // Maximum velocity (units/s) and acceleration while State::ZEROING
  ::frc971::ProfileParameters zeroing_profile_params;

  // Maximum velocity (units/s) and acceleration while State::RUNNING if max
  // velocity or acceleration in goal profile_params is 0
  ::frc971::ProfileParameters default_profile_params;

  // Maximum range of the subsystem in meters
  const ::frc971::constants::Range range;

  // Zeroing constants for PotAndABsoluteEncoder estimator
  const typename zeroing::PotAndAbsoluteEncoderZeroingEstimator::
      ZeroingConstants zeroing_constants;

  // Function that makes the integral loop for the subsystem
  ::std::function<StateFeedbackLoop<3, 1, 1>()> make_integral_loop;
};

// Class for controlling and motion profiling a single degree of freedom
// subsystem with a zeroing strategy of not moving.
class StaticZeroingSingleDOFProfiledSubsystem {
 public:
  StaticZeroingSingleDOFProfiledSubsystem(
      const StaticZeroingSingleDOFProfiledSubsystemParams &params);

  // Returns the filtered goal of the profiled subsystem (R)
  double goal(int row) const { return profiled_subsystem_.goal(row, 0); }

  // Returns the zeroing voltage of the subsystem
  double zeroing_voltage() { return params_.zeroing_voltage; }

  // Returns the operating voltage of the subsystem
  double operating_voltage() { return params_.operating_voltage; }

  // Sets further constraints on the range constant
  void set_min_position(double min_position) { min_position_ = min_position; }

  void set_max_position(double max_position) { max_position_ = max_position; }

  // Resets constrained min/max position
  void clear_min_position() { min_position_ = params_.range.lower_hard; }

  void clear_max_position() { max_position_ = params_.range.upper_hard; }

  // Returns the current position
  double position() const { return profiled_subsystem_.position(); }

  void Iterate(
      const StaticZeroingSingleDOFProfiledSubsystemGoal *goal,
      const typename zeroing::PotAndAbsoluteEncoderZeroingEstimator::Position
          *position,
      double *output,
      ::frc971::control_loops::AbsoluteProfiledJointStatus *status);

  // Resets the profiled subsystem and returns to uninitialized
  void Reset();

  enum class State : int32_t {
    UNINITIALIZED,
    DISABLED_INITIALIZED,
    ZEROING,
    RUNNING,
    ESTOP,
  };

  State state() const { return state_; }

 private:
  State state_ = State::UNINITIALIZED;
  double min_position_, max_position_;

  const StaticZeroingSingleDOFProfiledSubsystemParams params_;

  ::frc971::control_loops::SingleDOFProfiledSubsystem<
      zeroing::PotAndAbsoluteEncoderZeroingEstimator>
      profiled_subsystem_;
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_STATIC_ZEROING_SINGLE_DOF_PROFILED_SUBSYSTEM_H_