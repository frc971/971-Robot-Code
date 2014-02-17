#include "frc971/control_loops/claw/claw.h"

#include <stdio.h>

#include <algorithm>

#include "aos/common/control_loop/control_loops.q.h"
#include "aos/common/logging/logging.h"

#include "frc971/constants.h"
#include "frc971/control_loops/claw/claw_motor_plant.h"

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

void ClawLimitedLoop::CapU() {
  uncapped_average_voltage_ = U(0, 0) + U(1, 0) / 2.0;
  if (is_zeroing_) {
    const frc971::constants::Values &values = constants::GetValues();
    if (uncapped_average_voltage_ > values.claw.max_zeroing_voltage) {
      const double difference =
          uncapped_average_voltage_ - values.claw.max_zeroing_voltage;
      U(0, 0) -= difference;
    } else if (uncapped_average_voltage_ < -values.claw.max_zeroing_voltage) {
      const double difference =
          -uncapped_average_voltage_ - values.claw.max_zeroing_voltage;
      U(0, 0) += difference;
    }
  }

  double max_value =
      ::std::max(::std::abs(U(0, 0)), ::std::abs(U(1, 0) + U(0, 0)));

  if (max_value > 12.0) {
    LOG(DEBUG, "Capping U because max is %f\n", max_value);
    U = U * 12.0 / max_value;
    LOG(DEBUG, "Capping U is now %f %f\n", U(0, 0), U(1, 0));
  }
}

ClawMotor::ClawMotor(control_loops::ClawGroup *my_claw)
    : aos::control_loops::ControlLoop<control_loops::ClawGroup>(my_claw),
      has_top_claw_goal_(false),
      top_claw_goal_(0.0),
      top_claw_(this),
      has_bottom_claw_goal_(false),
      bottom_claw_goal_(0.0),
      bottom_claw_(this),
      claw_(MakeClawLoop()),
      was_enabled_(false),
      doing_calibration_fine_tune_(false),
      capped_goal_(false),
      mode_(UNKNOWN_LOCATION) {}

const int ZeroedStateFeedbackLoop::kZeroingMaxVoltage;

bool ZeroedStateFeedbackLoop::DoGetPositionOfEdge(
    const constants::Values::Claws::AnglePair &angles, double *edge_encoder,
    double *edge_angle, const HallEffectTracker &sensor,
    const char *hall_effect_name) {
  if (sensor.posedge_count_changed()) {
    if (posedge_value_ < last_encoder()) {
      *edge_angle = angles.upper_angle;
      LOG(INFO, "%s Posedge upper of %s -> %f\n", name_,
          hall_effect_name, *edge_angle);
    } else {
      *edge_angle = angles.lower_angle;
      LOG(INFO, "%s Posedge lower of %s -> %f\n", name_,
          hall_effect_name, *edge_angle);
    }
    *edge_encoder = posedge_value_;
    return true;
  }
  if (sensor.negedge_count_changed()) {
    if (negedge_value_ > last_encoder()) {
      *edge_angle = angles.upper_angle;
      LOG(INFO, "%s Negedge upper of %s -> %f\n", name_,
          hall_effect_name, *edge_angle);
    } else {
      *edge_angle = angles.lower_angle;
      LOG(INFO, "%s Negedge lower of %s -> %f\n", name_,
          hall_effect_name, *edge_angle);
    }
    *edge_encoder = negedge_value_;
    return true;
  }

  return false;
}

bool ZeroedStateFeedbackLoop::GetPositionOfEdge(
    const constants::Values::Claws::Claw &claw_values, double *edge_encoder,
    double *edge_angle) {
  // TODO(austin): Validate that the hall effect edge makes sense.
  // We must now be on the side of the edge that we expect to be, and the
  // encoder must have been on either side of the edge before and after.

  // TODO(austin): Compute the last off range min and max and compare the edge
  // value to the middle of the range.  This will be quite a bit more reliable.

  if (DoGetPositionOfEdge(claw_values.front, edge_encoder, edge_angle,
                          front_, "front")) {
    return true;
  }
  if (DoGetPositionOfEdge(claw_values.calibration, edge_encoder, edge_angle,
                          calibration_, "calibration")) {
    return true;
  }
  if (DoGetPositionOfEdge(claw_values.back, edge_encoder, edge_angle,
                          back_, "back")) {
    return true;
  }
  return false;
}

void TopZeroedStateFeedbackLoop::SetCalibration(double edge_encoder,
                                                double edge_angle) {
  double old_offset = offset_;
  offset_ = edge_angle - edge_encoder;
  const double doffset = offset_ - old_offset;
  motor_->ChangeTopOffset(doffset);
}

void BottomZeroedStateFeedbackLoop::SetCalibration(double edge_encoder,
                                                   double edge_angle) {
  double old_offset = offset_;
  offset_ = edge_angle - edge_encoder;
  const double doffset = offset_ - old_offset;
  motor_->ChangeBottomOffset(doffset);
}

void ClawMotor::ChangeTopOffset(double doffset) {
  claw_.ChangeTopOffset(doffset);
  if (has_top_claw_goal_) {
    top_claw_goal_ += doffset;
  }
}

void ClawMotor::ChangeBottomOffset(double doffset) {
  claw_.ChangeBottomOffset(doffset);
  if (has_bottom_claw_goal_) {
    bottom_claw_goal_ += doffset;
  }
}

void ClawLimitedLoop::ChangeTopOffset(double doffset) {
  Y_(1, 0) += doffset;
  X_hat(1, 0) += doffset;
  LOG(INFO, "Changing top offset by %f\n", doffset);
}
void ClawLimitedLoop::ChangeBottomOffset(double doffset) {
  Y_(0, 0) += doffset;
  X_hat(0, 0) += doffset;
  X_hat(1, 0) -= doffset;
  LOG(INFO, "Changing bottom offset by %f\n", doffset);
}

bool ClawMotor::is_ready() const {
  return (
      (top_claw_.zeroing_state() == ZeroedStateFeedbackLoop::CALIBRATED &&
       bottom_claw_.zeroing_state() == ZeroedStateFeedbackLoop::CALIBRATED) ||
      (::aos::robot_state->autonomous &&
       ((top_claw_.zeroing_state() == ZeroedStateFeedbackLoop::CALIBRATED ||
         top_claw_.zeroing_state() ==
             ZeroedStateFeedbackLoop::DISABLED_CALIBRATION) &&
        (bottom_claw_.zeroing_state() == ZeroedStateFeedbackLoop::CALIBRATED ||
         bottom_claw_.zeroing_state() ==
             ZeroedStateFeedbackLoop::DISABLED_CALIBRATION))));
}

bool ClawMotor::is_zeroing() const { return !is_ready(); }

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
  //     transition in all of these states.
  // TODO(austin): Handle zeroing while disabled correctly (only use a single
  //     edge and direction when zeroing.)

  if (::aos::robot_state.get() == nullptr) {
    return;
  }

  const frc971::constants::Values &values = constants::GetValues();

  if (position) {
    Eigen::Matrix<double, 2, 1> Y;
    Y << position->bottom.position + bottom_claw_.offset(),
        position->top.position + top_claw_.offset();
    claw_.Correct(Y);

    top_claw_.SetPositionValues(position->top);
    bottom_claw_.SetPositionValues(position->bottom);

    if (!has_top_claw_goal_) {
      has_top_claw_goal_ = true;
      top_claw_goal_ = top_claw_.absolute_position();
      initial_separation_ =
          top_claw_.absolute_position() - bottom_claw_.absolute_position();
    }
    if (!has_bottom_claw_goal_) {
      has_bottom_claw_goal_ = true;
      bottom_claw_goal_ = bottom_claw_.absolute_position();
      initial_separation_ =
          top_claw_.absolute_position() - bottom_claw_.absolute_position();
    }
    LOG(DEBUG, "Claw position is (top: %f bottom: %f\n",
        top_claw_.absolute_position(), bottom_claw_.absolute_position());
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
    has_bottom_claw_goal_ = true;
    has_top_claw_goal_ = true;
    doing_calibration_fine_tune_ = false;

    mode_ = READY;
  } else if (top_claw_.zeroing_state() !=
                 ZeroedStateFeedbackLoop::UNKNOWN_POSITION &&
             bottom_claw_.zeroing_state() !=
                 ZeroedStateFeedbackLoop::UNKNOWN_POSITION) {
    // Time to fine tune the zero.
    // Limit the goals here.
    // TODO(austin): Handle disabled state here.
    if (bottom_claw_.zeroing_state() != ZeroedStateFeedbackLoop::CALIBRATED) {
      // always get the bottom claw to calibrated first
      LOG(DEBUG, "Calibrating the bottom of the claw\n");
      if (!doing_calibration_fine_tune_) {
        if (::std::abs(bottom_absolute_position() -
                       values.claw.start_fine_tune_pos) <
            values.claw.claw_unimportant_epsilon) {
          doing_calibration_fine_tune_ = true;
          bottom_claw_goal_ += values.claw.claw_zeroing_speed * dt;
          LOG(DEBUG, "Ready to fine tune the bottom\n");
          mode_ = FINE_TUNE_BOTTOM;
        } else {
          // send bottom to zeroing start
          bottom_claw_goal_ = values.claw.start_fine_tune_pos;
          LOG(DEBUG, "Going to the start position for the bottom\n");
          mode_ = PREP_FINE_TUNE_BOTTOM;
        }
      } else {
        mode_ = FINE_TUNE_BOTTOM;
        bottom_claw_goal_ += values.claw.claw_zeroing_speed * dt;
        if (top_claw_.front_or_back_triggered() ||
            bottom_claw_.front_or_back_triggered()) {
          // We shouldn't hit a limit, but if we do, go back to the zeroing
          // point and try again.
          doing_calibration_fine_tune_ = false;
          bottom_claw_goal_ = values.claw.start_fine_tune_pos;
          LOG(DEBUG, "Found a limit, starting over.\n");
          mode_ = PREP_FINE_TUNE_BOTTOM;
        }

        if (bottom_claw_.calibration().value()) {
          if (bottom_claw_.calibration().posedge_count_changed() &&
              position) {
            // do calibration
            bottom_claw_.SetCalibration(
                position->bottom.posedge_value,
                values.claw.lower_claw.calibration.lower_angle);
            bottom_claw_.set_zeroing_state(ZeroedStateFeedbackLoop::CALIBRATED);
            // calibrated so we are done fine tuning bottom
            doing_calibration_fine_tune_ = false;
            LOG(DEBUG, "Calibrated the bottom correctly!\n");
          } else {
            doing_calibration_fine_tune_ = false;
            bottom_claw_goal_ = values.claw.start_fine_tune_pos;
            mode_ = PREP_FINE_TUNE_BOTTOM;
          }
        } else {
          LOG(DEBUG, "Fine tuning\n");
        }
      }
      // now set the top claw to track

      top_claw_goal_ = bottom_claw_goal_ + values.claw.claw_zeroing_separation;
    } else {
      // bottom claw must be calibrated, start on the top
      if (!doing_calibration_fine_tune_) {
        if (::std::abs(top_absolute_position() -
                       values.claw.start_fine_tune_pos) <
            values.claw.claw_unimportant_epsilon) {
          doing_calibration_fine_tune_ = true;
          top_claw_goal_ += values.claw.claw_zeroing_speed * dt;
          LOG(DEBUG, "Ready to fine tune the top\n");
          mode_ = FINE_TUNE_TOP;
        } else {
          // send top to zeroing start
          top_claw_goal_ = values.claw.start_fine_tune_pos;
          LOG(DEBUG, "Going to the start position for the top\n");
          mode_ = PREP_FINE_TUNE_TOP;
        }
      } else {
        mode_ = FINE_TUNE_TOP;
        top_claw_goal_ += values.claw.claw_zeroing_speed * dt;
        if (top_claw_.front_or_back_triggered() ||
            bottom_claw_.front_or_back_triggered()) {
          // this should not happen, but now we know it won't
          doing_calibration_fine_tune_ = false;
          top_claw_goal_ = values.claw.start_fine_tune_pos;
          LOG(DEBUG, "Found a limit, starting over.\n");
          mode_ = PREP_FINE_TUNE_TOP;
        }
        if (top_claw_.calibration().value()) {
          if (top_claw_.calibration().posedge_count_changed() &&
              position) {
            // do calibration
            top_claw_.SetCalibration(
                position->top.posedge_value,
                values.claw.upper_claw.calibration.lower_angle);
            top_claw_.set_zeroing_state(ZeroedStateFeedbackLoop::CALIBRATED);
            // calibrated so we are done fine tuning top
            doing_calibration_fine_tune_ = false;
            LOG(DEBUG, "Calibrated the top correctly!\n");
          } else {
            doing_calibration_fine_tune_ = false;
            top_claw_goal_ = values.claw.start_fine_tune_pos;
            mode_ = PREP_FINE_TUNE_TOP;
          }
        }
      }
      // now set the bottom claw to track
      bottom_claw_goal_ = top_claw_goal_ - values.claw.claw_zeroing_separation;
    }
  } else {
    doing_calibration_fine_tune_ = false;
    if (!was_enabled_ && enabled) {
      if (position) {
        top_claw_goal_ = position->top.position;
        bottom_claw_goal_ = position->bottom.position;
        initial_separation_ =
            position->top.position - position->bottom.position;
      } else {
        has_top_claw_goal_ = false;
        has_bottom_claw_goal_ = false;
      }
    }

    // TODO(austin): Limit the goals here.

    if ((bottom_claw_.zeroing_state() !=
             ZeroedStateFeedbackLoop::UNKNOWN_POSITION ||
         bottom_claw_.front().value() || top_claw_.front().value()) &&
        !top_claw_.back().value() && !bottom_claw_.back().value()) {
      if (enabled) {
        // Time to slowly move back up to find any position to narrow down the
        // zero.
        top_claw_goal_ += values.claw.claw_zeroing_off_speed * dt;
        bottom_claw_goal_ += values.claw.claw_zeroing_off_speed * dt;
        // TODO(austin): Goal velocity too!
        LOG(DEBUG, "Bottom is known.\n");
      }
    } else {
      // We don't know where either claw is.  Slowly start moving down to find
      // any hall effect.
      if (enabled) {
        top_claw_goal_ -= values.claw.claw_zeroing_off_speed * dt;
        bottom_claw_goal_ -= values.claw.claw_zeroing_off_speed * dt;
        // TODO(austin): Goal velocity too!
        LOG(DEBUG, "Both are unknown.\n");
      }
    }

    if (enabled) {
      top_claw_.SetCalibrationOnEdge(
          values.claw.upper_claw, ZeroedStateFeedbackLoop::APPROXIMATE_CALIBRATION);
      bottom_claw_.SetCalibrationOnEdge(
          values.claw.lower_claw, ZeroedStateFeedbackLoop::APPROXIMATE_CALIBRATION);
    } else {
      top_claw_.SetCalibrationOnEdge(
          values.claw.upper_claw, ZeroedStateFeedbackLoop::DISABLED_CALIBRATION);
      bottom_claw_.SetCalibrationOnEdge(
          values.claw.lower_claw, ZeroedStateFeedbackLoop::DISABLED_CALIBRATION);
    }
    mode_ = UNKNOWN_LOCATION;
  }

  if (has_top_claw_goal_ && has_bottom_claw_goal_) {
    claw_.R << bottom_claw_goal_, top_claw_goal_ - bottom_claw_goal_, 0, 0;
    double separation = -971;
    if (position != nullptr) {
      separation = position->top.position - position->bottom.position;
    }
    LOG(DEBUG, "Goal is %f (bottom) %f, separation is %f\n", claw_.R(0, 0),
        claw_.R(1, 0), separation);

    // Only cap power when one of the halves of the claw is unknown.
    claw_.set_is_zeroing(mode_ == UNKNOWN_LOCATION || mode_ == FINE_TUNE_TOP ||
                         mode_ == FINE_TUNE_BOTTOM);
    claw_.Update(output == nullptr);
  } else {
    claw_.Update(true);
  }

  capped_goal_ = false;
  switch (mode_) {
    case READY:
    case PREP_FINE_TUNE_TOP:
    case PREP_FINE_TUNE_BOTTOM:
      break;
    case FINE_TUNE_BOTTOM:
    case FINE_TUNE_TOP:
    case UNKNOWN_LOCATION: {
      if (claw_.uncapped_average_voltage() > values.claw.max_zeroing_voltage) {
        double dx = (claw_.uncapped_average_voltage() -
                     values.claw.max_zeroing_voltage) /
                    claw_.K(0, 0);
        bottom_claw_goal_ -= dx;
        top_claw_goal_ -= dx;
        capped_goal_ = true;
        LOG(DEBUG, "Moving the goal by %f to prevent windup\n", dx);
        LOG(DEBUG, "Uncapped is %f, max is %f, difference is %f\n",
            claw_.uncapped_average_voltage(), values.claw.max_zeroing_voltage,
            (claw_.uncapped_average_voltage() -
             values.claw.max_zeroing_voltage));
      } else if (claw_.uncapped_average_voltage() <
                 -values.claw.max_zeroing_voltage) {
        double dx = (claw_.uncapped_average_voltage() +
                     values.claw.max_zeroing_voltage) /
                    claw_.K(0, 0);
        bottom_claw_goal_ -= dx;
        top_claw_goal_ -= dx;
        capped_goal_ = true;
        LOG(DEBUG, "Moving the goal by %f to prevent windup\n", dx);
      }
    } break;
  }

  if (output) {
    output->top_claw_voltage = claw_.U(1, 0) + claw_.U(0, 0);
    output->bottom_claw_voltage =  claw_.U(0, 0);
  }
  status->done = false;

  was_enabled_ = ::aos::robot_state->enabled;
}

}  // namespace control_loops
}  // namespace frc971
