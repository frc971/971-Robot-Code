#ifndef FRC971_CONTROL_LOOPS_CLAW_CLAW_H_
#define FRC971_CONTROL_LOOPS_CLAW_CLAW_H_

#include <memory>

#include "aos/common/control_loop/ControlLoop.h"
#include "frc971/constants.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/claw/claw_motor_plant.h"

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

class ClawLimitedLoop : public StateFeedbackLoop<4, 2, 2> {
 public:
  ClawLimitedLoop(StateFeedbackLoop<4, 2, 2> loop)
      : StateFeedbackLoop<4, 2, 2>(loop) {}
  virtual void CapU();

  void ChangeTopOffset(double doffset);
  void ChangeBottomOffset(double doffset);
};

class ClawMotor;

// This class implements the CapU function correctly given all the extra
// information that we know about from the wrist motor.
// It does not have any zeroing logic in it, only logic to deal with a delta U
// controller.
class ZeroedStateFeedbackLoop {
 public:
  ZeroedStateFeedbackLoop(const char *name, ClawMotor *motor)
      : offset_(0.0),
        name_(name),
        motor_(motor),
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
        front_hall_effect_(false),
        calibration_hall_effect_(false),
        back_hall_effect_(false),
        zeroing_state_(UNKNOWN_POSITION),
        posedge_value_(0.0),
        negedge_value_(0.0),
        encoder_(0.0),
        last_encoder_(0.0) {}

  const static int kZeroingMaxVoltage = 5;

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
    last_encoder_ = encoder_;
    encoder_ = claw.position;

    front_hall_effect_ = claw.front_hall_effect;
    calibration_hall_effect_ = claw.calibration_hall_effect;
    back_hall_effect_ = claw.back_hall_effect;
  }

  double absolute_position() const { return encoder() + offset(); }

  bool front_hall_effect() const { return front_hall_effect_; }
  bool calibration_hall_effect() const { return calibration_hall_effect_; }
  bool back_hall_effect() const { return back_hall_effect_; }

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

  double encoder() const { return encoder_; }
  double last_encoder() const { return last_encoder_; }

  double offset() const { return offset_; }

  // Returns true if an edge was detected in the last cycle and then sets the
  // edge_position to the absolute position of the edge.
  bool GetPositionOfEdge(const constants::Values::Claws::Claw &claw,
                         double *edge_encoder, double *edge_angle);

#undef COUNT_SETTER_GETTER

 protected:
  // The accumulated voltage to apply to the motor.
  double offset_;
  const char *name_;

  ClawMotor *motor_;

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
  bool front_hall_effect_;
  bool calibration_hall_effect_;
  bool back_hall_effect_;

  JointZeroingState zeroing_state_;
  double posedge_value_;
  double negedge_value_;
  double encoder_;
  double last_encoder_;
};

class TopZeroedStateFeedbackLoop : public ZeroedStateFeedbackLoop {
 public:
  TopZeroedStateFeedbackLoop(ClawMotor *motor)
      : ZeroedStateFeedbackLoop("top", motor) {}
  // Sets the calibration offset given the absolute angle and the corrisponding
  // encoder value.
  void SetCalibration(double edge_encoder, double edge_angle);

  bool SetCalibrationOnEdge(const constants::Values::Claws::Claw &claw_values,
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
};

class BottomZeroedStateFeedbackLoop : public ZeroedStateFeedbackLoop {
 public:
  BottomZeroedStateFeedbackLoop(ClawMotor *motor)
      : ZeroedStateFeedbackLoop("bottom", motor) {}
  // Sets the calibration offset given the absolute angle and the corrisponding
  // encoder value.
  void SetCalibration(double edge_encoder, double edge_angle);

  bool SetCalibrationOnEdge(const constants::Values::Claws::Claw &claw_values,
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
};

class ClawMotor
    : public aos::control_loops::ControlLoop<control_loops::ClawGroup> {
 public:
  explicit ClawMotor(control_loops::ClawGroup *my_claw =
                         &control_loops::claw_queue_group);

  // True if the state machine is ready.
  bool is_ready() const { return false; }

  void ChangeTopOffset(double doffset);
  void ChangeBottomOffset(double doffset);

 protected:
  virtual void RunIteration(const control_loops::ClawGroup::Goal *goal,
                            const control_loops::ClawGroup::Position *position,
                            control_loops::ClawGroup::Output *output,
                            ::aos::control_loops::Status *status);

  double top_absolute_position() const {
    return claw_.X_hat(1, 0) + claw_.X_hat(0, 0);
  }
  double bottom_absolute_position() const { return claw_.X_hat(0, 0); }

 private:
  // Friend the test classes for acces to the internal state.
  friend class testing::ClawTest_NoWindupPositive_Test;
  friend class testing::ClawTest_NoWindupNegative_Test;

  // The zeroed joint to use.
  bool has_top_claw_goal_;
  double top_claw_goal_;
  TopZeroedStateFeedbackLoop top_claw_;

  bool has_bottom_claw_goal_;
  double bottom_claw_goal_;
  BottomZeroedStateFeedbackLoop bottom_claw_;

  // The claw loop.
  ClawLimitedLoop claw_;

  bool was_enabled_;
  bool doing_calibration_fine_tune_;

  // The initial seperation when disabled.  Used as the safe seperation
  // distance.
  double initial_seperation_;

  DISALLOW_COPY_AND_ASSIGN(ClawMotor);
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_CLAW_CLAW_H_
