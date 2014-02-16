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
        last_encoder_(0.0){}

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


  bool SetCalibrationOnEdge(const constants::Values::ShooterLimits &shooter_values,
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


  void SetPositionValues(double position) {
    Eigen::Matrix<double, 1, 1> Y;
    Y << position;
    Correct(Y);
  }


  void SetGoalPosition(double desired_position,
  		  double desired_velocity) {
  	  // austin said something about which matrix to set, but I didn't under
	  // very much of it
	  //some_matrix = {desired_position, desired_velocity};
	  printf("%s:%d : seg fault (%.2f, %.2f)\n", __FILE__, __LINE__,
	  		  desired_position, desired_velocity);
	  *(const char **)(NULL) = "seg fault";
  }

  // apply a small amout of voltage outside the loop so we can
  // take up backlash in gears
  void ApplySomeVoltage() {
	  printf("%s:%d : seg fault\n", __FILE__, __LINE__);
	  *(const char **)(NULL) = "seg fault";
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
  explicit ShooterMotor(
      control_loops::ShooterGroup *my_shooter = &control_loops::shooter_queue_group);

  // True if the goal was moved to avoid goal windup.
  //bool capped_goal() const { return shooter_.capped_goal(); }

typedef enum {
	STATE_INITIALIZE,
	STATE_REQUEST_LOAD,
	STATE_LOAD_BACKTRACK,
	STATE_LOAD,
	STATE_LOADING_PROBLEM,
	STATE_PREPARE_SHOT,
	STATE_READY,
	STATE_REQUEST_FIRE,
	STATE_PREPARE_FIRE,
	STATE_FIRE,
	STATE_UNLOAD,
	STATE_UNLOAD_MOVE,
	STATE_READY_UNLOAD
} State;

 protected:

 double PowerToPosition(double power) { return power; }

  virtual void RunIteration(
      const ShooterGroup::Goal *goal,
      const control_loops::ShooterGroup::Position *position,
      ShooterGroup::Output *output,
      ShooterGroup::Status *status);

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

  DISALLOW_COPY_AND_ASSIGN(ShooterMotor);
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_shooter_shooter_H_
