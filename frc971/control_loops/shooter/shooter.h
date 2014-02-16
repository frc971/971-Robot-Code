#ifndef FRC971_CONTROL_LOOPS_shooter_shooter_H_
#define FRC971_CONTROL_LOOPS_shooter_shooter_H_

#include <memory>

#include "aos/common/control_loop/ControlLoop.h"
#include "frc971/control_loops/state_feedback_loop.h"

#include "frc971/control_loops/shooter/shooter_motor_plant.h"
#include "frc971/control_loops/shooter/shooter.q.h"

#include "frc971/control_loops/zeroed_joint.h"

namespace frc971 {
namespace control_loops {
namespace testing {
class ShooterTest_NoWindupPositive_Test;
class ShooterTest_NoWindupNegative_Test;
};

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


  bool SetCalibrationOnEdge(const constants::Values::Claw &claw_values,
                            JointZeroingState zeroing_state) {
    double edge_encoder;
    double known_position;
    if (GetPositionOfEdge(claw_values, &edge_encoder, &known_position)) {
      LOG(INFO, "Calibration edge.\n");
      SetCalibration(edge_encoder, known_position);
      set_zeroing_state(zeroing_state);
      return true;
    }
    return false;
  }


  void SetPositionValues(double poistion) {
    Eigen::Matrix<double, 1, 1> Y;
    Y << position;
    Correct(Y);
  }


  void SetGoalPositionVelocity(double desired_position,
  		  double desired_velocity) {
  	  // austin said something about which matrix to set, but I didn't under
	  // very much of it
	  //some_matrix = {desired_position, desired_velocity};
	  printf("%s:%d : seg fault?\n", __FILE__, __LINE__);
	  *(const char **)(NULL) = "seg fault";
  }

  double position() const { return X_hat(0, 0); }
  double encoder() const { return encoder_; }
  double last_encoder() const { return last_encoder_; }

  // Returns true if an edge was detected in the last cycle and then sets the
  // edge_position to the absolute position of the edge.
  bool GetPositionOfEdge(const constants::Values::Shooter &shooter,
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
    : public aos::control_loops::ControlLoop<control_loops::ShooterLoop> {
 public:
  explicit ShooterMotor(
      control_loops::ShooterLoop *my_shooter = &control_loops::shooter);

  // True if the goal was moved to avoid goal windup.
  bool capped_goal() const { return zeroed_joint_.capped_goal(); }

  // True if the shooter is zeroing.
  bool is_zeroing() const { return zeroed_joint_.is_zeroing(); }

  // True if the shooter is zeroing.
  bool is_moving_off() const { return zeroed_joint_.is_moving_off(); }

  // True if the state machine is uninitialized.
  bool is_uninitialized() const { return zeroed_joint_.is_uninitialized(); }

  // True if the state machine is ready.
  bool is_ready() const { return zeroed_joint_.is_ready(); }

enum {
	STATE_INITIALIZE,
	STATE_REQUEST_LOAD,
	STATE_LOAD_BACKTRACK,
	STATE_LOAD,
	STATE_LOADING_PROBLEM,
	STATE_PREPARE_SHOT,
	STATE_BRAKE_SET,
	STATE_READY,
	STATE_REQUEST_FIRE,
	STATE_PREPARE_FIRE,
	STATE_FIRE,
	STATE_UNLOAD,
	STATE_UNLOAD_MOVE,
	STATE_READY_UNLOAD
} State;

 protected:
  virtual void RunIteration(
      const ShooterLoop::Goal *goal,
      const control_loops::ShooterLoop::Position *position,
      ShooterLoop::Output *output,
      ShooterLoop::Status *status);

 private:
  // Friend the test classes for acces to the internal state.
  friend class testing::ShooterTest_NoWindupPositive_Test;
  friend class testing::ShooterTest_NoWindupNegative_Test;

  // The zeroed joint to use.
  ZeroedJoint<1> zeroed_joint_;
  ZeroedStateFeedbackLoop shooter_;

  DISALLOW_COPY_AND_ASSIGN(ShooterMotor);
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_shooter_shooter_H_
