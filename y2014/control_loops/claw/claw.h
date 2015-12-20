#ifndef Y2014_CONTROL_LOOPS_CLAW_CLAW_H_
#define Y2014_CONTROL_LOOPS_CLAW_CLAW_H_

#include <memory>

#include "aos/common/controls/control_loop.h"
#include "aos/common/controls/polytope.h"
#include "y2014/constants.h"
#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/coerce_goal.h"
#include "y2014/control_loops/claw/claw.q.h"
#include "y2014/control_loops/claw/claw_motor_plant.h"
#include "frc971/control_loops/hall_effect_tracker.h"

namespace y2014 {
namespace control_loops {
namespace testing {
class WindupClawTest;
};

// Note: Everything in this file assumes that there is a 1 cycle delay between
// power being requested and it showing up at the motor.  It assumes that
// X_hat(2, 1) is the voltage being applied as well.  It will go unstable if
// that isn't true.

class ClawLimitedLoop : public StateFeedbackLoop<4, 2, 2> {
 public:
  ClawLimitedLoop(StateFeedbackLoop<4, 2, 2> &&loop);
  virtual void CapU();

  void set_is_zeroing(bool is_zeroing) { is_zeroing_ = is_zeroing; }
  void set_positions_known(bool top_known, bool bottom_known) {
    top_known_ = top_known;
    bottom_known_ = bottom_known;
  }

  void ChangeTopOffset(double doffset);
  void ChangeBottomOffset(double doffset);

  double uncapped_average_voltage() const { return uncapped_average_voltage_; }

 private:
  double uncapped_average_voltage_;
  bool is_zeroing_;

  bool top_known_ = false, bottom_known_ = false;

  const ::aos::controls::HPolytope<2> U_Poly_, U_Poly_zeroing_;
};

class ClawMotor;

// This class implements the CapU function correctly given all the extra
// information that we know about from the wrist motor.
// It does not have any zeroing logic in it, only logic to deal with a delta U
// controller.
class ZeroedStateFeedbackLoop {
 public:
  ZeroedStateFeedbackLoop(const char *name, ClawMotor *motor);

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

  void SetPositionValues(const ::y2014::control_loops::HalfClawPosition &claw);

  void Reset(const ::y2014::control_loops::HalfClawPosition &claw);

  double absolute_position() const { return encoder() + offset(); }

  const ::frc971::HallEffectTracker &front() const { return front_; }
  const ::frc971::HallEffectTracker &calibration() const { return calibration_; }
  const ::frc971::HallEffectTracker &back() const { return back_; }

  bool any_hall_effect_changed() const {
    return front().either_count_changed() ||
           calibration().either_count_changed() ||
           back().either_count_changed();
  }
  bool front_or_back_triggered() const {
    return front().value() || back().value();
  }
  bool any_triggered() const {
    return calibration().value() || front().value() || back().value();
  }

  double encoder() const { return encoder_; }
  double last_encoder() const { return last_encoder_; }

  double offset() const { return offset_; }

  // Returns true if an edge was detected in the last cycle and then sets the
  // edge_position to the absolute position of the edge.
  bool GetPositionOfEdge(const constants::Values::Claws::Claw &claw,
                         double *edge_encoder, double *edge_angle);

  bool SawFilteredPosedge(const ::frc971::HallEffectTracker &this_sensor,
                          const ::frc971::HallEffectTracker &sensorA,
                          const ::frc971::HallEffectTracker &sensorB);

  bool SawFilteredNegedge(const ::frc971::HallEffectTracker &this_sensor,
                          const ::frc971::HallEffectTracker &sensorA,
                          const ::frc971::HallEffectTracker &sensorB);

#undef COUNT_SETTER_GETTER

 protected:
  // The accumulated voltage to apply to the motor.
  double offset_;
  const char *name_;

  ClawMotor *motor_;

  ::frc971::HallEffectTracker front_, calibration_, back_;

  JointZeroingState zeroing_state_;
  double min_hall_effect_on_angle_;
  double max_hall_effect_on_angle_;
  double min_hall_effect_off_angle_;
  double max_hall_effect_off_angle_;
  double encoder_;
  double last_encoder_;
  double last_on_encoder_;
  double last_off_encoder_;
  bool any_triggered_last_;

  const ::frc971::HallEffectTracker* posedge_filter_ = nullptr;
  const ::frc971::HallEffectTracker* negedge_filter_ = nullptr;

 private:
  // Does the edges of 1 sensor for GetPositionOfEdge.
  bool DoGetPositionOfEdge(const constants::Values::Claws::AnglePair &angles,
                           double *edge_encoder, double *edge_angle,
                           const ::frc971::HallEffectTracker &sensor,
                           const ::frc971::HallEffectTracker &sensorA,
                           const ::frc971::HallEffectTracker &sensorB,
                           const char *hall_effect_name);
};

class TopZeroedStateFeedbackLoop : public ZeroedStateFeedbackLoop {
 public:
  TopZeroedStateFeedbackLoop(ClawMotor *motor)
      : ZeroedStateFeedbackLoop("top", motor) {}
  // Sets the calibration offset given the absolute angle and the corresponding
  // encoder value.
  void SetCalibration(double edge_encoder, double edge_angle);

  bool SetCalibrationOnEdge(const constants::Values::Claws::Claw &claw_values,
                            JointZeroingState zeroing_state);
  double ComputeCalibrationChange(double edge_encoder, double edge_angle);
  void HandleCalibrationError(
      const constants::Values::Claws::Claw &claw_values);
};

class BottomZeroedStateFeedbackLoop : public ZeroedStateFeedbackLoop {
 public:
  BottomZeroedStateFeedbackLoop(ClawMotor *motor)
      : ZeroedStateFeedbackLoop("bottom", motor) {}
  // Sets the calibration offset given the absolute angle and the corrisponding
  // encoder value.
  void SetCalibration(double edge_encoder, double edge_angle);

  bool SetCalibrationOnEdge(const constants::Values::Claws::Claw &claw_values,
                            JointZeroingState zeroing_state);
  double ComputeCalibrationChange(double edge_encoder, double edge_angle);
  void HandleCalibrationError(
      const constants::Values::Claws::Claw &claw_values);
};

class ClawMotor
    : public aos::controls::ControlLoop<::y2014::control_loops::ClawQueue> {
 public:
  explicit ClawMotor(::y2014::control_loops::ClawQueue *my_claw =
                         &::y2014::control_loops::claw_queue);

  // True if the state machine is ready.
  bool capped_goal() const { return capped_goal_; }

  double uncapped_average_voltage() const {
    return claw_.uncapped_average_voltage();
  }

  // True if the claw is zeroing.
  bool is_zeroing() const;

  // True if the state machine is ready.
  bool is_ready() const;

  void ChangeTopOffset(double doffset);
  void ChangeBottomOffset(double doffset);

  enum CalibrationMode {
    READY,
    PREP_FINE_TUNE_TOP,
    FINE_TUNE_TOP,
    PREP_FINE_TUNE_BOTTOM,
    FINE_TUNE_BOTTOM,
    UNKNOWN_LOCATION
  };

  CalibrationMode mode() const { return mode_; }

 protected:
  virtual void RunIteration(
      const ::y2014::control_loops::ClawQueue::Goal *goal,
      const ::y2014::control_loops::ClawQueue::Position *position,
      ::y2014::control_loops::ClawQueue::Output *output,
      ::y2014::control_loops::ClawQueue::Status *status);

  double top_absolute_position() const {
    return claw_.X_hat(1, 0) + claw_.X_hat(0, 0);
  }
  double bottom_absolute_position() const { return claw_.X_hat(0, 0); }

 private:
  // Friend the test classes for acces to the internal state.
  friend class testing::WindupClawTest;

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

  // The initial separation when disabled.  Used as the safe separation
  // distance.
  double initial_separation_;

  bool capped_goal_;
  CalibrationMode mode_;

  DISALLOW_COPY_AND_ASSIGN(ClawMotor);
};

// Modifies the bottom and top goal such that they are within the limits and
// their separation isn't too much or little.
void LimitClawGoal(double *bottom_goal, double *top_goal,
                   const constants::Values &values);

}  // namespace control_loops
}  // namespace y2014

#endif  // Y2014_CONTROL_LOOPS_CLAW_CLAW_H_
