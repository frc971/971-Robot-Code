#ifndef FRC971_CONTROL_LOOPS_CLAW_CLAW_H_
#define FRC971_CONTROL_LOOPS_CLAW_CLAW_H_

#include <memory>

#include "aos/common/control_loop/ControlLoop.h"
#include "frc971/constants.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/claw/top_claw_motor_plant.h"
#include "frc971/control_loops/claw/bottom_claw_motor_plant.h"

namespace frc971 {
namespace control_loops {
namespace testing {
class ClawTest_NoWindupPositive_Test;
class ClawTest_NoWindupNegative_Test;
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
        front_hall_effect_posedge_count_(0.0),
        previous_front_hall_effect_posedge_count_(0.0),
        front_hall_effect_negedge_count_(0.0),
        previous_front_hall_effect_negedge_count_(0.0),
        calibration_hall_effect_posedge_count_(0.0),
        previous_calibration_hall_effect_posedge_count_(0.0),
        calibration_hall_effect_negedge_count_(0.0),
        previous_calibration_hall_effect_negedge_count_(0.0),
        back_hall_effect_posedge_count_(0.0),
        previous_back_hall_effect_posedge_count_(0.0),
        back_hall_effect_negedge_count_(0.0),
        previous_back_hall_effect_negedge_count_(0.0),
        zeroing_state_(UNKNOWN_POSITION),
        posedge_value_(0.0),
        negedge_value_(0.0),
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
    // We have an approximate position for where the claw is using.
    APPROXIMATE_CALIBRATION,
    // We observed the calibration edge while disabled. This is good enough for
    // autonomous mode.
    DISABLED_CALIBRATION,
    // Ready for use during teleop.
    CALIBRATED
  };

  void set_zeroing_state(JointZeroingState zeroing_state) {
    zeroing_state_ = zeroing_state;
  }
  JointZeroingState zeroing_state() const { return zeroing_state_; }

  // Sets the calibration offset given the absolute angle and the corrisponding
  // encoder value.
  void SetCalibration(double edge_encoder, double edge_angle) {
    offset_ = edge_angle - edge_encoder;
  }

  bool SetCalibrationOnEdge(const constants::Values::Claw &claw_values,
                            JointZeroingState zeroing_state) {
    double edge_encoder;
    double edge_angle;
    if (GetPositionOfEdge(claw_values, &edge_encoder, &edge_angle)) {
      LOG(INFO, "Calibration edge.\n");
      SetCalibration(edge_encoder, edge_angle);
      set_zeroing_state(zeroing_state);
      return true;
    }
    return false;
  }

  void SetPositionValues(const HalfClawPosition &claw) {
    set_front_hall_effect_posedge_count(claw.front_hall_effect_posedge_count);
    set_front_hall_effect_negedge_count(claw.front_hall_effect_negedge_count);
    set_calibration_hall_effect_posedge_count(
        claw.calibration_hall_effect_posedge_count);
    set_calibration_hall_effect_negedge_count(
        claw.calibration_hall_effect_negedge_count);
    set_back_hall_effect_posedge_count(claw.back_hall_effect_posedge_count);
    set_back_hall_effect_negedge_count(claw.back_hall_effect_negedge_count);

    posedge_value_ = claw.posedge_value;
    negedge_value_ = claw.negedge_value;
    Eigen::Matrix<double, 1, 1> Y;
    Y << claw.position;
    Correct(Y);
  }

#define COUNT_SETTER_GETTER(variable)              \
  void set_##variable(int32_t value) {             \
    previous_##variable##_ = variable##_;          \
    variable##_ = value;                           \
  }                                                \
  int32_t variable() const { return variable##_; } \
  bool variable##_changed() const {                \
    return previous_##variable##_ != variable##_;  \
  }

  // TODO(austin): Pull all this out of the new sub structure.
  COUNT_SETTER_GETTER(front_hall_effect_posedge_count);
  COUNT_SETTER_GETTER(front_hall_effect_negedge_count);
  COUNT_SETTER_GETTER(calibration_hall_effect_posedge_count);
  COUNT_SETTER_GETTER(calibration_hall_effect_negedge_count);
  COUNT_SETTER_GETTER(back_hall_effect_posedge_count);
  COUNT_SETTER_GETTER(back_hall_effect_negedge_count);

  bool any_hall_effect_changed() const {
    return front_hall_effect_posedge_count_changed() ||
           front_hall_effect_negedge_count_changed() ||
           calibration_hall_effect_posedge_count_changed() ||
           calibration_hall_effect_negedge_count_changed() ||
           back_hall_effect_posedge_count_changed() ||
           back_hall_effect_negedge_count_changed();
  }

  double position() const { return X_hat(0, 0); }
  double encoder() const { return encoder_; }
  double last_encoder() const { return last_encoder_; }

  // Returns true if an edge was detected in the last cycle and then sets the
  // edge_position to the absolute position of the edge.
  bool GetPositionOfEdge(const constants::Values::Claw &claw,
                         double *edge_encoder, double *edge_angle);

#undef COUNT_SETTER_GETTER

 private:
  // The accumulated voltage to apply to the motor.
  double voltage_;
  double last_voltage_;
  double uncapped_voltage_;
  double offset_;

  double previous_position_;

  int32_t front_hall_effect_posedge_count_;
  int32_t previous_front_hall_effect_posedge_count_;
  int32_t front_hall_effect_negedge_count_;
  int32_t previous_front_hall_effect_negedge_count_;
  int32_t calibration_hall_effect_posedge_count_;
  int32_t previous_calibration_hall_effect_posedge_count_;
  int32_t calibration_hall_effect_negedge_count_;
  int32_t previous_calibration_hall_effect_negedge_count_;
  int32_t back_hall_effect_posedge_count_;
  int32_t previous_back_hall_effect_posedge_count_;
  int32_t back_hall_effect_negedge_count_;
  int32_t previous_back_hall_effect_negedge_count_;

  JointZeroingState zeroing_state_;
  double posedge_value_;
  double negedge_value_;
  double encoder_;
  double last_encoder_;
};

class ClawMotor
    : public aos::control_loops::ControlLoop<control_loops::ClawGroup> {
 public:
  explicit ClawMotor(control_loops::ClawGroup *my_claw =
                         &control_loops::claw_queue_group);

  // True if the state machine is ready.
  bool is_ready() const { return false; }

 protected:
  virtual void RunIteration(const control_loops::ClawGroup::Goal *goal,
                            const control_loops::ClawGroup::Position *position,
                            control_loops::ClawGroup::Output *output,
                            ::aos::control_loops::Status *status);

  double top_absolute_position() const { return top_claw_.position(); }
  double bottom_absolute_position() const { return bottom_claw_.position(); }

 private:
  // Friend the test classes for acces to the internal state.
  friend class testing::ClawTest_NoWindupPositive_Test;
  friend class testing::ClawTest_NoWindupNegative_Test;

  // The zeroed joint to use.
  bool has_top_claw_goal_;
  double top_claw_goal_;
  ZeroedStateFeedbackLoop top_claw_;

  bool has_bottom_claw_goal_;
  double bottom_claw_goal_;
  ZeroedStateFeedbackLoop bottom_claw_;

  bool was_enabled_;

  DISALLOW_COPY_AND_ASSIGN(ClawMotor);
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_CLAW_CLAW_H_
