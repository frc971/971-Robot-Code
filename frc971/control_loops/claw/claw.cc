#include "frc971/control_loops/claw/claw.h"

#include <stdio.h>

#include <algorithm>

#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/constants.h"
#include "frc971/control_loops/claw/top_claw_motor_plant.h"
#include "frc971/control_loops/claw/bottom_claw_motor_plant.h"

// Zeroing plan.
// There are 2 types of zeros.  Enabled and disabled ones.
// Disabled ones are only valid during auto mode, and can be used to speed up
// the enabled zero process.  We need to re-zero during teleop in case the auto
// zero was poor and causes us to miss all our shots.
//
// We need to be able to zero manually while disabled by moving the joint over
// the zeros.
// Zero on the down edge when disabled (gravity in the direction of motion)
//
// When enabled, zero on the up edge (gravity opposing the direction of motion)
// The enabled sequence needs to work as follows.  We can crash the claw if we
// bring them too close to each other or too far from each other.  The only safe
// thing to do is to move them in unison.
//
// Start by moving them both towards the front of the bot to either find either
// the middle hall effect on either jaw, or the front hall effect on the bottom
// jaw.  Any edge that isn't the desired edge will provide an approximate edge
// location that can be used for the fine tuning step.
// Once an edge is found on the front claw, move back the other way with both
// claws until an edge is found for the other claw.
// Now that we have an approximate zero, we can robustify the limits to keep
// both claws safe.  Then, we can move both claws to a position that is the
// correct side of the zero and go zero.

// Valid region plan.
// Difference between the arms has a range, and the values of each arm has a range.
// If a claw runs up against a static limit, don't let the goal change outside
// the limit.
// If a claw runs up against a movable limit, move both claws outwards to get
// out of the condition.

namespace frc971 {
namespace control_loops {

void ZeroedStateFeedbackLoop::CapU() {
  const double old_voltage = voltage_;
  voltage_ += U(0, 0);

  uncapped_voltage_ = voltage_;

  double limit = zeroing_state_ != UNKNOWN_POSITION ? 12.0 : kZeroingMaxVoltage;

  // Make sure that reality and the observer can't get too far off.  There is a
  // delay by one cycle between the applied voltage and X_hat(2, 0), so compare
  // against last cycle's voltage.
  if (X_hat(2, 0) > last_voltage_ + 2.0) {
    voltage_ -= X_hat(2, 0) - (last_voltage_ + 2.0);
    LOG(DEBUG, "X_hat(2, 0) = %f\n", X_hat(2, 0));
  } else if (X_hat(2, 0) < last_voltage_ -2.0) {
    voltage_ += X_hat(2, 0) - (last_voltage_ - 2.0);
    LOG(DEBUG, "X_hat(2, 0) = %f\n", X_hat(2, 0));
  }

  voltage_ = std::min(limit, voltage_);
  voltage_ = std::max(-limit, voltage_);
  U(0, 0) = voltage_ - old_voltage;
  //LOG(DEBUG, "abc %f\n", X_hat(2, 0) - voltage_);
  //LOG(DEBUG, "error %f\n", X_hat(0, 0) - R(0, 0));

  last_voltage_ = voltage_;
}

ClawMotor::ClawMotor(control_loops::ClawGroup *my_claw)
    : aos::control_loops::ControlLoop<control_loops::ClawGroup>(my_claw),
      has_top_claw_goal_(false),
      top_claw_goal_(0.0),
      top_claw_(MakeTopClawLoop()),
      has_bottom_claw_goal_(false),
      bottom_claw_goal_(0.0),
      bottom_claw_(MakeBottomClawLoop()),
      was_enabled_(false) {}

const int ZeroedStateFeedbackLoop::kZeroingMaxVoltage;

// Positive angle is up, and positive power is up.
void ClawMotor::RunIteration(const control_loops::ClawGroup::Goal *goal,
                             const control_loops::ClawGroup::Position *position,
                             control_loops::ClawGroup::Output *output,
                             ::aos::control_loops::Status *status) {

  // Disable the motors now so that all early returns will return with the
  // motors disabled.
  if (output) {
    output->top_claw_voltage = 0;
    output->bottom_claw_voltage = 0;
    output->intake_voltage = 0;
  }

  // TODO(austin): Handle the disabled state and the disabled -> enabled
  // transition in all of these states.
  // TODO(austin): Handle zeroing while disabled.

  // TODO(austin): Save all the counters so we know when something actually
  //               happens.
  // TODO(austin): Helpers to find the position of the claw on an edge.

  // TODO(austin): This may not be necesary because of the ControlLoop class.
  ::aos::robot_state.FetchLatest();
  if (::aos::robot_state.get() == nullptr) {
    return;
  }

  if (position) {
    if (!has_top_claw_goal_) {
      has_top_claw_goal_ = true;
      top_claw_goal_ = position->top.position;
    }
    if (!has_bottom_claw_goal_) {
      has_bottom_claw_goal_ = true;
      bottom_claw_goal_ = position->bottom.position;
    }

    top_claw_.set_front_hall_effect_posedge_count(
        position->top.front_hall_effect_posedge_count);
    top_claw_.set_front_hall_effect_negedge_count(
        position->top.front_hall_effect_negedge_count);
    top_claw_.set_calibration_hall_effect_posedge_count(
        position->top.calibration_hall_effect_posedge_count);
    top_claw_.set_calibration_hall_effect_negedge_count(
        position->top.calibration_hall_effect_negedge_count);
    top_claw_.set_back_hall_effect_posedge_count(
        position->top.back_hall_effect_posedge_count);
    top_claw_.set_back_hall_effect_negedge_count(
        position->top.back_hall_effect_negedge_count);

    bottom_claw_.set_front_hall_effect_posedge_count(
        position->bottom.front_hall_effect_posedge_count);
    bottom_claw_.set_front_hall_effect_negedge_count(
        position->bottom.front_hall_effect_negedge_count);
    bottom_claw_.set_calibration_hall_effect_posedge_count(
        position->bottom.calibration_hall_effect_posedge_count);
    bottom_claw_.set_calibration_hall_effect_negedge_count(
        position->bottom.calibration_hall_effect_negedge_count);
    bottom_claw_.set_back_hall_effect_posedge_count(
        position->bottom.back_hall_effect_posedge_count);
    bottom_claw_.set_back_hall_effect_negedge_count(
        position->bottom.back_hall_effect_negedge_count);
  }

  bool autonomous = ::aos::robot_state->autonomous;

  if ((top_claw_.zeroing_state() == ZeroedStateFeedbackLoop::CALIBRATED &&
       bottom_claw_.zeroing_state() == ZeroedStateFeedbackLoop::CALIBRATED) ||
      (autonomous &&
       ((top_claw_.zeroing_state() == ZeroedStateFeedbackLoop::CALIBRATED ||
         top_claw_.zeroing_state() ==
             ZeroedStateFeedbackLoop::DISABLED_CALIBRATION) &&
        (bottom_claw_.zeroing_state() == ZeroedStateFeedbackLoop::CALIBRATED ||
         bottom_claw_.zeroing_state() ==
             ZeroedStateFeedbackLoop::DISABLED_CALIBRATION)))) {
    // Ready to use the claw.
    // Limit the goals here.
  } else if (top_claw_.zeroing_state() !=
                 ZeroedStateFeedbackLoop::UNKNOWN_POSITION &&
             bottom_claw_.zeroing_state() !=
                 ZeroedStateFeedbackLoop::UNKNOWN_POSITION) {
    // Time to fine tune the zero.
    // Limit the goals here.
    if (bottom_claw_.zeroing_state() != ZeroedStateFeedbackLoop::CALIBRATED) {
    } else {
    }
  } else {
    if (!was_enabled_ && enabled) {
      
    }
    // Limit the goals here.
    if (top_claw_.zeroing_state() ==
        ZeroedStateFeedbackLoop::UNKNOWN_POSITION) {
    }
    if (bottom_claw_.zeroing_state() ==
        ZeroedStateFeedbackLoop::UNKNOWN_POSITION) {
    }

    if (bottom_claw_.zeroing_state() !=
        ZeroedStateFeedbackLoop::UNKNOWN_POSITION) {
      // Time to slowly move back up to find any position to narrow down the
      // zero.
    } else {
      // We don't know where either claw is.  Slowly start moving down to find
      // any hall effect.
      LOG(INFO, "Unknown position\n");
    }
  }

  // TODO(austin): Handle disabled.

  if (position) {
    top_claw_.Y << position->top.position;
    bottom_claw_.Y << position->bottom.position;
  }

  // TODO(austin): ...
  top_claw_.R << goal->bottom_angle + goal->seperation_angle, 0.0, 0.0;
  bottom_claw_.R << goal->bottom_angle, 0.0, 0.0;

  top_claw_.Update(position != nullptr, output == nullptr);
  bottom_claw_.Update(position != nullptr, output == nullptr);

  if (position) {
    //LOG(DEBUG, "pos: %f hall: %s absolute: %f\n", position->top_position,
        //position->top_calibration_hall_effect ? "true" : "false",
        //zeroed_joint_.absolute_position());
  }

  if (output) {
    output->top_claw_voltage = top_claw_.voltage();
    output->bottom_claw_voltage = bottom_claw_.voltage();
  }
  status->done = false;
      //::std::abs(zeroed_joint_.absolute_position() - goal->bottom_angle -
                 //goal->seperation_angle) < 0.004;

  was_enabled_ = ::aos::robot_state->enabled;
}

}  // namespace control_loops
}  // namespace frc971
