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
class ShooterTest_NoWindupPositive_Test;
class ShooterTest_NoWindupNegative_Test;
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
        encoder_(0.0),
        last_encoder_(0.0) {}

  const static int kZeroingMaxVoltage = 5;

  // Caps U, but this time respects the state of the wrist as well.
  virtual void CapU();

  // Returns the accumulated voltage.
  double voltage() const { return voltage_; }

  // Returns the uncapped voltage.
  double uncapped_voltage() const { return uncapped_voltage_; }

  // Zeros the accumulator.
  void ZeroPower() { voltage_ = 0.0; }

  enum JointZeroingState {
    // We don't know where the joint is at all.
    UNKNOWN_POSITION,
    // Ready for use during teleop.
    CALIBRATED
  };

  void set_zeroing_state(JointZeroingState zeroing_state) {
    zeroing_state_ = zeroing_state;
  }

  JointZeroingState zeroing_state() const { return zeroing_state_; }

  // Sets the calibration offset given the absolute angle and the corrisponding
  // encoder value.
  void SetCalibration(double encoder_val, double known_position) {
    offset_ = known_position - encoder_val;
  }

  bool SetCalibrationOnEdge(
      const constants::Values::ShooterLimits &shooter_values,
      JointZeroingState zeroing_state) {
    double edge_encoder;
    double known_position;
    if (GetPositionOfEdge(shooter_values, &edge_encoder, &known_position)) {
      LOG(INFO, "Calibration edge.\n");
      SetCalibration(edge_encoder, known_position);
      set_zeroing_state(zeroing_state);
      return true;
    }
    return false;
  }

  void SetPositionDirectly(double position) { X_hat(0, 0) = position; }

  void SetPositionValues(double position) {
    Eigen::Matrix<double, 1, 1> Y;
    Y << position;
    LOG(INFO, "Setting position to %f\n", position);
    Correct(Y);
  }

  void SetGoalPosition(double desired_position, double desired_velocity) {
    // austin said something about which matrix to set, but I didn't under
    // very much of it
    //some_matrix = {desired_position, desired_velocity};
    LOG(INFO, "ZSFL> dp: %.2f dz: %.2f\n", desired_position, desired_velocity);
    R << desired_position, desired_velocity, 0;
  }

  double position() const { return X_hat(0, 0); }
  double encoder() const { return encoder_; }
  double last_encoder() const { return last_encoder_; }

  // Returns true if an edge was detected in the last cycle and then sets the
  // edge_position to the absolute position of the edge.
  bool GetPositionOfEdge(const constants::Values::ShooterLimits &shooter,
                         double *edge_encoder, double *known_position);

#undef COUNT_SETTER_GETTER

 private:
  // The accumulated voltage to apply to the motor.
  double voltage_;
  double last_voltage_;
  double uncapped_voltage_;
  double offset_;

  double previous_position_;

  JointZeroingState zeroing_state_;
  double encoder_;
  double last_encoder_;
};

class ShooterMotor
    : public aos::control_loops::ControlLoop<control_loops::ShooterGroup> {
 public:
  explicit ShooterMotor(control_loops::ShooterGroup *my_shooter =
                            &control_loops::shooter_queue_group);

  // True if the goal was moved to avoid goal windup.
  //bool capped_goal() const { return shooter_.capped_goal(); }

  double PowerToPosition(double power) {
    //LOG(WARNING, "power to position not correctly implemented\n");
    const frc971::constants::Values &values = constants::GetValues();
    double new_pos =
        (power > values.shooter.upper_limit) ? values.shooter.upper_limit
                                             : (power < 0.0)
            ? 0.0
            : power;

    return new_pos;
  }

  typedef enum {
    STATE_INITIALIZE = 0,
    STATE_REQUEST_LOAD = 1,
    STATE_LOAD_BACKTRACK = 2,
    STATE_LOAD = 3,
    STATE_LOADING_PROBLEM = 4,
    STATE_PREPARE_SHOT = 5,
    STATE_READY = 6,
    STATE_REQUEST_FIRE = 7,
    STATE_PREPARE_FIRE = 8,
    STATE_FIRE = 9,
    STATE_UNLOAD = 10,
    STATE_UNLOAD_MOVE = 11,
    STATE_READY_UNLOAD = 12
  } State;

 protected:

  virtual void RunIteration(
      const ShooterGroup::Goal *goal,
      const control_loops::ShooterGroup::Position *position,
      ShooterGroup::Output *output, ShooterGroup::Status *status);

 private:
  // Friend the test classes for acces to the internal state.
  friend class testing::ShooterTest_NoWindupPositive_Test;
  friend class testing::ShooterTest_NoWindupNegative_Test;

  control_loops::ShooterGroup::Position last_position_;

  ZeroedStateFeedbackLoop shooter_;

  // position need to zero
  double calibration_position_;

  // state machine state
  State state_;

  // time to giving up on loading problem
  Time loading_problem_end_time_;

  // wait for brake to set
  Time shooter_brake_set_time_;

  // we are attempting to take up some of the backlash
  // in the gears before the plunger hits
  Time prepare_fire_end_time_;

  // time that shot must have completed
  Time shot_end_time_;

  // track cycles that we are stuck to detect errors
  int cycles_not_moved_;

  // setup on the intial loop may involve shortcuts
  bool initial_loop_;

  DISALLOW_COPY_AND_ASSIGN(ShooterMotor);
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_shooter_shooter_H_
