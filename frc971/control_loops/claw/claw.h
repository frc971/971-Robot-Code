#ifndef FRC971_CONTROL_LOOPS_CLAW_CLAW_H_
#define FRC971_CONTROL_LOOPS_CLAW_CLAW_H_

#include <memory>

#include "aos/common/control_loop/ControlLoop.h"
#include "frc971/constants.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/claw/claw.q.h"
#include "frc971/control_loops/claw/claw_motor_plant.h"
#include "frc971/control_loops/hall_effect_tracker.h"

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
      : StateFeedbackLoop<4, 2, 2>(loop),
        uncapped_average_voltage_(0.0),
        is_zeroing_(true) {}
  virtual void CapU();

  void set_is_zeroing(bool is_zeroing) { is_zeroing_ = is_zeroing; }

  void ChangeTopOffset(double doffset);
  void ChangeBottomOffset(double doffset);

  double uncapped_average_voltage() const { return uncapped_average_voltage_; }

 private:
  double uncapped_average_voltage_;
  bool is_zeroing_;
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
    front_.Update(claw.front);
    calibration_.Update(claw.calibration);
    back_.Update(claw.back);

    posedge_value_ = claw.posedge_value;
    negedge_value_ = claw.negedge_value;
    last_encoder_ = encoder_;
    encoder_ = claw.position;
  }

  double absolute_position() const { return encoder() + offset(); }

  const HallEffectTracker &front() const { return front_; }
  const HallEffectTracker &calibration() const { return calibration_; }
  const HallEffectTracker &back() const { return back_; }

  bool any_hall_effect_changed() const {
    return front().either_count_changed() ||
           calibration().either_count_changed() ||
           back().either_count_changed();
  }
  bool front_or_back_triggered() const {
    return front().value() || back().value();
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

  HallEffectTracker front_, calibration_, back_;

  JointZeroingState zeroing_state_;
  double posedge_value_;
  double negedge_value_;
  double encoder_;
  double last_encoder_;

 private:
  // Does the edges of 1 sensor for GetPositionOfEdge.
  bool DoGetPositionOfEdge(const constants::Values::Claws::AnglePair &angles,
                           double *edge_encoder, double *edge_angle,
                           const HallEffectTracker &sensor,
                           const char *hall_effect_name);
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
  bool capped_goal() const { return capped_goal_; }

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

  bool capped_goal_;

  DISALLOW_COPY_AND_ASSIGN(ClawMotor);
};

}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_CLAW_CLAW_H_
