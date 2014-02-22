#ifndef FRC971_CONTROL_LOOPS_shooter_shooter_H_
#define FRC971_CONTROL_LOOPS_shooter_shooter_H_

#include <memory>

#include "aos/common/control_loop/ControlLoop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "aos/common/time.h"

#include "frc971/constants.h"
#include "frc971/control_loops/shooter/shooter_motor_plant.h"
#include "frc971/control_loops/shooter/shooter.q.h"

namespace frc971 {
namespace control_loops {
namespace testing {
class ShooterTest_UnloadWindupPositive_Test;
class ShooterTest_UnloadWindupNegative_Test;
};

using ::aos::time::Time;

// Note: Everything in this file assumes that there is a 1 cycle delay between
// power being requested and it showing up at the motor.  It assumes that
// X_hat(2, 1) is the voltage being applied as well.  It will go unstable if
// that isn't true.

// This class implements the CapU function correctly given all the extra
// information that we know about from the wrist motor.
// It does not have any zeroing logic in it, only logic to deal with a delta U
// controller.
class ZeroedStateFeedbackLoop : public StateFeedbackLoop<3, 1, 1> {
 public:
  ZeroedStateFeedbackLoop(StateFeedbackLoop<3, 1, 1> loop)
      : StateFeedbackLoop<3, 1, 1>(loop),
        voltage_(0.0),
        last_voltage_(0.0),
        uncapped_voltage_(0.0),
        offset_(0.0),
        max_voltage_(12.0),
        capped_goal_(false) {}

  const static int kZeroingMaxVoltage = 5;

  // Caps U, but this time respects the state of the wrist as well.
  virtual void CapU();

  // Returns the accumulated voltage.
  double voltage() const { return voltage_; }

  // Returns the uncapped voltage.
  double uncapped_voltage() const { return uncapped_voltage_; }

  // Zeros the accumulator.
  void ZeroPower() { voltage_ = 0.0; }

  // Sets the calibration offset given the absolute angle and the corrisponding
  // encoder value.
  void SetCalibration(double encoder_val, double known_position);

  double offset() const { return offset_; }

  double absolute_position() const { return X_hat(0, 0) + kPositionOffset; }
  double absolute_velocity() const { return X_hat(1, 0); }

  void CorrectPosition(double position) {
    Eigen::Matrix<double, 1, 1> Y;
    Y << position + offset_ - kPositionOffset;
    LOG(DEBUG, "Setting position to %f\n", position);
    Correct(Y);
  }

  // Recomputes the power goal for the current controller and position/velocity.
  void RecalculatePowerGoal();

  double goal_position() const { return R(0, 0) + kPositionOffset; }
  double goal_velocity() const { return R(1, 0); }
  void InitializeState(double position) {
    X_hat(0, 0) = position - kPositionOffset;
  }

  void SetGoalPosition(double desired_position, double desired_velocity) {
    LOG(DEBUG, "Goal position: %f Goal velocity: %f\n", desired_position, desired_velocity);

    R << desired_position - kPositionOffset, desired_velocity,
        (-A(1, 0) / A(1, 2) * (desired_position - kPositionOffset) -
         A(1, 1) / A(1, 2) * desired_velocity);
  }

  double position() const { return X_hat(0, 0) - offset_ + kPositionOffset; }

  void set_max_voltage(const double max_voltage) { max_voltage_ = max_voltage; }
  bool capped_goal() const { return capped_goal_; }

  void CapGoal();

 private:
  // The offset between what is '0' (0 rate on the spring) and the 0 (all the
  // way cocked).
  constexpr static double kPositionOffset = 0.305054 + 0.0254;
  // The accumulated voltage to apply to the motor.
  double voltage_;
  double last_voltage_;
  double uncapped_voltage_;
  double offset_;
  double max_voltage_;
  bool capped_goal_;
};

const Time kUnloadTimeout = Time::InSeconds(10);
const Time kLoadTimeout = Time::InSeconds(10);
const Time kLoadProblemEndTimeout = Time::InSeconds(0.5);
const Time kShooterBrakeSetTime = Time::InSeconds(0.05);
const Time kShotEndTimeout = Time::InSeconds(1.0);
const Time kPrepareFireEndTime = Time::InMS(40);

class ShooterMotor
    : public aos::control_loops::ControlLoop<control_loops::ShooterGroup> {
 public:
  explicit ShooterMotor(control_loops::ShooterGroup *my_shooter =
                            &control_loops::shooter_queue_group);

  // True if the goal was moved to avoid goal windup.
  bool capped_goal() const { return shooter_.capped_goal(); }

  double PowerToPosition(double power);

  typedef enum {
    STATE_INITIALIZE = 0,
    STATE_REQUEST_LOAD = 1,
    STATE_LOAD_BACKTRACK = 2,
    STATE_LOAD = 3,
    STATE_LOADING_PROBLEM = 4,
    STATE_PREPARE_SHOT = 5,
    STATE_READY = 6,
    STATE_PREPARE_FIRE = 7,
    STATE_FIRE = 8,
    STATE_UNLOAD = 9,
    STATE_UNLOAD_MOVE = 10,
    STATE_READY_UNLOAD = 11,
    STATE_ESTOP = 12
  } State;

  State state() { return state_; }

 protected:
  virtual void RunIteration(
      const ShooterGroup::Goal *goal,
      const control_loops::ShooterGroup::Position *position,
      ShooterGroup::Output *output, ShooterGroup::Status *status);

 private:
  // Friend the test classes for acces to the internal state.
  friend class testing::ShooterTest_UnloadWindupPositive_Test;
  friend class testing::ShooterTest_UnloadWindupNegative_Test;

  // Enter state STATE_UNLOAD
  void Unload() {
    state_ = STATE_UNLOAD;
    unload_timeout_ = Time::Now() + kUnloadTimeout;
  }
  // Enter state STATE_LOAD
  void Load() {
    state_ = STATE_LOAD;
    load_timeout_ = Time::Now() + kLoadTimeout;
  }

  control_loops::ShooterGroup::Position last_position_;

  ZeroedStateFeedbackLoop shooter_;

  // state machine state
  State state_;

  // time to giving up on loading problem
  Time loading_problem_end_time_;

  // The end time when loading for it to timeout.
  Time load_timeout_;

  // wait for brake to set
  Time shooter_brake_set_time_;
  
  // The timeout for unloading.
  Time unload_timeout_;

  // we are attempting to take up some of the backlash
  // in the gears before the plunger hits
  Time prepare_fire_end_time_;

  // time that shot must have completed
  Time shot_end_time_;

  // track cycles that we are stuck to detect errors
  int cycles_not_moved_;

  double firing_starting_position_;

  // True if the latch should be engaged and the brake should be engaged.
  bool latch_piston_;
  bool brake_piston_;
  int32_t last_distal_posedge_count_;
  int32_t last_proximal_posedge_count_;

  DISALLOW_COPY_AND_ASSIGN(ShooterMotor);
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_shooter_shooter_H_
