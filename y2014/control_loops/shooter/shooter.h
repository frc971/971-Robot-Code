#ifndef Y2014_CONTROL_LOOPS_shooter_shooter_H_
#define Y2014_CONTROL_LOOPS_shooter_shooter_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "aos/common/time.h"

#include "y2014/constants.h"
#include "y2014/control_loops/shooter/shooter_motor_plant.h"
#include "y2014/control_loops/shooter/shooter.q.h"

namespace y2014 {
namespace control_loops {
namespace testing {
class ShooterTest_UnloadWindupPositive_Test;
class ShooterTest_UnloadWindupNegative_Test;
class ShooterTest_RezeroWhileUnloading_Test;
};

// Note: Everything in this file assumes that there is a 1 cycle delay between
// power being requested and it showing up at the motor.  It assumes that
// X_hat(2, 1) is the voltage being applied as well.  It will go unstable if
// that isn't true.

// This class implements the CapU function correctly given all the extra
// information that we know about.
// It does not have any zeroing logic in it, only logic to deal with a delta U
// controller.
class ZeroedStateFeedbackLoop : public StateFeedbackLoop<3, 1, 1> {
 public:
  ZeroedStateFeedbackLoop(StateFeedbackLoop<3, 1, 1> &&loop)
      : StateFeedbackLoop<3, 1, 1>(::std::move(loop)),
        voltage_(0.0),
        last_voltage_(0.0),
        uncapped_voltage_(0.0),
        offset_(0.0),
        max_voltage_(12.0),
        capped_goal_(false) {}

  const static int kZeroingMaxVoltage = 5;

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
    Correct(Y);
  }

  // Recomputes the power goal for the current controller and position/velocity.
  void RecalculatePowerGoal();

  double goal_position() const { return R(0, 0) + kPositionOffset; }
  double goal_velocity() const { return R(1, 0); }
  void InitializeState(double position) {
    mutable_X_hat(0, 0) = position - kPositionOffset;
    mutable_X_hat(1, 0) = 0.0;
    mutable_X_hat(2, 0) = 0.0;
  }

  void SetGoalPosition(double desired_position, double desired_velocity) {
    LOG(DEBUG, "Goal position: %f Goal velocity: %f\n", desired_position,
        desired_velocity);

    mutable_R() << desired_position - kPositionOffset, desired_velocity,
        (-plant().A(1, 0) / plant().A(1, 2) *
             (desired_position - kPositionOffset) -
         plant().A(1, 1) / plant().A(1, 2) * desired_velocity);
  }

  double position() const { return X_hat(0, 0) - offset_ + kPositionOffset; }

  void set_max_voltage(double max_voltage) { max_voltage_ = max_voltage; }
  bool capped_goal() const { return capped_goal_; }

  void CapGoal();

  // Friend the test classes for acces to the internal state.
  friend class testing::ShooterTest_RezeroWhileUnloading_Test;

 private:
  // The offset between what is '0' (0 rate on the spring) and the 0 (all the
  // way cocked).
  constexpr static double kPositionOffset =
      ::y2014::control_loops::shooter::kMaxExtension;
  // The accumulated voltage to apply to the motor.
  double voltage_;
  double last_voltage_;
  double uncapped_voltage_;
  double offset_;
  double max_voltage_;
  bool capped_goal_;
};

static constexpr ::std::chrono::nanoseconds kUnloadTimeout =
    ::std::chrono::seconds(10);
static constexpr ::std::chrono::nanoseconds kLoadTimeout =
    ::std::chrono::seconds(2);
static constexpr ::std::chrono::nanoseconds kLoadProblemEndTimeout =
    ::std::chrono::seconds(1);
static constexpr ::std::chrono::nanoseconds kShooterBrakeSetTime =
    ::std::chrono::milliseconds(50);
// Time to wait after releasing the latch piston before winching back again.
static constexpr ::std::chrono::nanoseconds kShotEndTimeout =
    ::std::chrono::milliseconds(200);
static constexpr ::std::chrono::nanoseconds kPrepareFireEndTime =
    ::std::chrono::milliseconds(40);

class ShooterMotor
    : public aos::controls::ControlLoop<::y2014::control_loops::ShooterQueue> {
 public:
  explicit ShooterMotor(::y2014::control_loops::ShooterQueue *my_shooter =
                            &::y2014::control_loops::shooter_queue);

  // True if the goal was moved to avoid goal windup.
  bool capped_goal() const { return shooter_.capped_goal(); }

  double PowerToPosition(double power);
  double PositionToPower(double position);
  void CheckCalibrations(
      const ::y2014::control_loops::ShooterQueue::Position *position);

  typedef enum {
    STATE_INITIALIZE = 0,
    STATE_REQUEST_LOAD = 1,
    STATE_LOAD_BACKTRACK = 2,
    STATE_LOAD = 3,
    STATE_LOADING_PROBLEM = 4,
    STATE_PREPARE_SHOT = 5,
    STATE_READY = 6,
    STATE_FIRE = 8,
    STATE_UNLOAD = 9,
    STATE_UNLOAD_MOVE = 10,
    STATE_READY_UNLOAD = 11,
    STATE_ESTOP = 12
  } State;

  State state() { return state_; }

 protected:
  virtual void RunIteration(
      const ::y2014::control_loops::ShooterQueue::Goal *goal,
      const ::y2014::control_loops::ShooterQueue::Position *position,
      ::y2014::control_loops::ShooterQueue::Output *output,
      ::y2014::control_loops::ShooterQueue::Status *status);

 private:
  // We have to override this to keep the pistons in the correct positions.
  virtual void ZeroOutputs();

  // Friend the test classes for acces to the internal state.
  friend class testing::ShooterTest_UnloadWindupPositive_Test;
  friend class testing::ShooterTest_UnloadWindupNegative_Test;
  friend class testing::ShooterTest_RezeroWhileUnloading_Test;

  // Enter state STATE_UNLOAD
  void Unload() {
    state_ = STATE_UNLOAD;
    unload_timeout_ = ::aos::monotonic_clock::now() + kUnloadTimeout;
  }
  // Enter state STATE_LOAD
  void Load() {
    state_ = STATE_LOAD;
    load_timeout_ = ::aos::monotonic_clock::now() + kLoadTimeout;
  }

  ::y2014::control_loops::ShooterQueue::Position last_position_;

  ZeroedStateFeedbackLoop shooter_;

  // state machine state
  State state_;

  // time to giving up on loading problem
  ::aos::monotonic_clock::time_point loading_problem_end_time_ =
      ::aos::monotonic_clock::min_time;

  // The end time when loading for it to timeout.
  ::aos::monotonic_clock::time_point load_timeout_ =
      ::aos::monotonic_clock::min_time;

  // wait for brake to set
  ::aos::monotonic_clock::time_point shooter_brake_set_time_ =
      ::aos::monotonic_clock::min_time;

  // The timeout for unloading.
  ::aos::monotonic_clock::time_point unload_timeout_ =
      ::aos::monotonic_clock::min_time;

  // time that shot must have completed
  ::aos::monotonic_clock::time_point shot_end_time_ =
      ::aos::monotonic_clock::min_time;

  // track cycles that we are stuck to detect errors
  int cycles_not_moved_;

  double firing_starting_position_;

  // True if the latch should be engaged and the brake should be engaged.
  bool latch_piston_;
  bool brake_piston_;
  int32_t last_distal_posedge_count_;
  int32_t last_proximal_posedge_count_;
  uint32_t shot_count_;
  bool zeroed_;
  int distal_posedge_validation_cycles_left_;
  int proximal_posedge_validation_cycles_left_;
  bool last_distal_current_;
  bool last_proximal_current_;

  DISALLOW_COPY_AND_ASSIGN(ShooterMotor);
};

}  // namespace control_loops
}  // namespace y2014

#endif  // Y2014_CONTROL_LOOPS_shooter_shooter_H_
