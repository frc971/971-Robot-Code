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

  double limit = zeroing_state() != UNKNOWN_POSITION ? 12.0 : kZeroingMaxVoltage;

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
  LOG(DEBUG, "abc %f\n", X_hat(2, 0) - voltage_);
  LOG(DEBUG, "error %f\n", X_hat(0, 0) - R(0, 0));

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

bool ZeroedStateFeedbackLoop::GetPositionOfEdge(
    const constants::Values::Claw &claw_values, double *edge_encoder,
    double *edge_angle) {

  // TODO(austin): Validate that the hall effect edge makes sense.
  // We must now be on the side of the edge that we expect to be, and the
  // encoder must have been on either side of the edge before and after.

  if (front_hall_effect_posedge_count_changed()) {
    if (encoder() - last_encoder() < 0) {
      *edge_angle = claw_values.front.upper_angle;
    } else {
      *edge_angle = claw_values.front.lower_angle;
    }
    *edge_encoder = posedge_value_;
    return true;
  }
  if (front_hall_effect_negedge_count_changed()) {
    if (encoder() - last_encoder() > 0) {
      *edge_angle = claw_values.front.upper_angle;
    } else {
      *edge_angle = claw_values.front.lower_angle;
    }
    *edge_encoder = negedge_value_;
    return true;
  }
  if (calibration_hall_effect_posedge_count_changed()) {
    if (encoder() - last_encoder() < 0) {
      *edge_angle = claw_values.calibration.upper_angle;
    } else {
      *edge_angle = claw_values.calibration.lower_angle;
    }
    *edge_encoder = posedge_value_;
    return true;
  }
  if (calibration_hall_effect_negedge_count_changed()) {
    if (encoder() - last_encoder() > 0) {
      *edge_angle = claw_values.calibration.upper_angle;
    } else {
      *edge_angle = claw_values.calibration.lower_angle;
    }
    *edge_encoder = negedge_value_;
    return true;
  }
  if (back_hall_effect_posedge_count_changed()) {
    if (encoder() - last_encoder() < 0) {
      *edge_angle = claw_values.back.upper_angle;
    } else {
      *edge_angle = claw_values.back.lower_angle;
    }
    *edge_encoder = posedge_value_;
    return true;
  }
  if (back_hall_effect_negedge_count_changed()) {
    if (encoder() - last_encoder() > 0) {
      *edge_angle = claw_values.back.upper_angle;
    } else {
      *edge_angle = claw_values.back.lower_angle;
    }
    *edge_encoder = negedge_value_;
    return true;
  }
  return false;
}

// Positive angle is up, and positive power is up.
void ClawMotor::RunIteration(const control_loops::ClawGroup::Goal *goal,
                             const control_loops::ClawGroup::Position *position,
                             control_loops::ClawGroup::Output *output,
                             ::aos::control_loops::Status *status) {
  constexpr double dt = 0.01;

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

  const frc971::constants::Values &values = constants::GetValues();

  if (position) {
    top_claw_.SetPositionValues(position->top);
    bottom_claw_.SetPositionValues(position->bottom);

    if (!has_top_claw_goal_) {
      has_top_claw_goal_ = true;
      top_claw_goal_ = position->top.position;
    }
    if (!has_bottom_claw_goal_) {
      has_bottom_claw_goal_ = true;
      bottom_claw_goal_ = position->bottom.position;
    }
  }

  bool autonomous = ::aos::robot_state->autonomous;
  bool enabled = ::aos::robot_state->enabled;

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
    bottom_claw_goal_ = goal->bottom_angle;
    top_claw_goal_ = goal->bottom_angle + goal->seperation_angle;
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
      if (position) {
        top_claw_goal_ = position->top.position;
        bottom_claw_goal_ = position->bottom.position;
      } else {
        has_top_claw_goal_ = false;
        has_bottom_claw_goal_ = false;
      }
    }

    // TODO(austin): Limit the goals here.
    // Need to prevent windup, limit voltage, deal with windup on only 1 claw,
    // ...
    if (top_claw_.zeroing_state() ==
        ZeroedStateFeedbackLoop::UNKNOWN_POSITION) {
    }
    if (bottom_claw_.zeroing_state() ==
        ZeroedStateFeedbackLoop::UNKNOWN_POSITION) {
    }

    if (bottom_claw_.zeroing_state() !=
        ZeroedStateFeedbackLoop::UNKNOWN_POSITION) {
      if (enabled) {
        // Time to slowly move back up to find any position to narrow down the
        // zero.
        top_claw_goal_ += values.claw_zeroing_off_speed * dt;
        bottom_claw_goal_ += values.claw_zeroing_off_speed * dt;
        // TODO(austin): Goal velocity too!
      }
    } else {
      // We don't know where either claw is.  Slowly start moving down to find
      // any hall effect.
      if (enabled) {
        top_claw_goal_-= values.claw_zeroing_off_speed * dt;
        bottom_claw_goal_ -= values.claw_zeroing_off_speed * dt;
        // TODO(austin): Goal velocity too!
      }
    }

    if (enabled) {
      top_claw_.SetCalibrationOnEdge(
          values.upper_claw, ZeroedStateFeedbackLoop::APPROXIMATE_CALIBRATION);
      bottom_claw_.SetCalibrationOnEdge(
          values.lower_claw, ZeroedStateFeedbackLoop::APPROXIMATE_CALIBRATION);
    } else {
      top_claw_.SetCalibrationOnEdge(
          values.upper_claw, ZeroedStateFeedbackLoop::DISABLED_CALIBRATION);
      bottom_claw_.SetCalibrationOnEdge(
          values.lower_claw, ZeroedStateFeedbackLoop::DISABLED_CALIBRATION);
    }
  }

  // TODO(austin): Handle disabled.

  // TODO(austin): ...
  if (has_top_claw_goal_ && has_bottom_claw_goal_) {
    top_claw_.R << top_claw_goal_, 0.0, 0.0;
    bottom_claw_.R << bottom_claw_goal_, 0.0, 0.0;

    top_claw_.Update(output == nullptr);
    bottom_claw_.Update(output == nullptr);
  } else {
    top_claw_.ZeroPower();
    bottom_claw_.ZeroPower();
  }

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
