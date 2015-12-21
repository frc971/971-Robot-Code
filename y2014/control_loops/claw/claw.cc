#include "y2014/control_loops/claw/claw.h"

#include <algorithm>

#include "aos/common/controls/control_loops.q.h"
#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/logging/matrix_logging.h"
#include "aos/common/commonmath.h"

#include "y2014/constants.h"
#include "y2014/control_loops/claw/claw_motor_plant.h"

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
// Difference between the arms has a range, and the values of each arm has a
// range.
// If a claw runs up against a static limit, don't let the goal change outside
// the limit.
// If a claw runs up against a movable limit, move both claws outwards to get
// out of the condition.

namespace y2014 {
namespace control_loops {

using ::frc971::HallEffectTracker;
using ::y2014::control_loops::claw::kDt;
using ::frc971::control_loops::DoCoerceGoal;
using ::y2014::control_loops::ClawPositionToLog;

static const double kZeroingVoltage = 4.0;
static const double kMaxVoltage = 12.0;
const double kRezeroThreshold = 0.07;

ClawLimitedLoop::ClawLimitedLoop(StateFeedbackLoop<4, 2, 2> &&loop)
    : StateFeedbackLoop<4, 2, 2>(::std::move(loop)),
      uncapped_average_voltage_(0.0),
      is_zeroing_(true),
      U_Poly_((Eigen::Matrix<double, 4, 2>() << 1, 0,
               -1, 0,
               0, 1,
               0, -1).finished(),
              (Eigen::Matrix<double, 4, 1>() << kMaxVoltage, kMaxVoltage,
               kMaxVoltage, kMaxVoltage).finished()),
      U_Poly_zeroing_((Eigen::Matrix<double, 4, 2>() << 1, 0,
               -1, 0,
               0, 1,
               0, -1).finished(),
              (Eigen::Matrix<double, 4, 1>() <<
               kZeroingVoltage, kZeroingVoltage,
               kZeroingVoltage, kZeroingVoltage).finished()) {
  ::aos::controls::HPolytope<0>::Init();
}

// Caps the voltage prioritizing reducing velocity error over reducing
// positional error.
// Uses the polytope libararies which we used to just use for the drivetrain.
// Uses a region representing the maximum voltage and then transforms it such
// that the points represent different amounts of positional error and
// constrains the region such that, if at all possible, it will maintain its
// current efforts to reduce velocity error.
void ClawLimitedLoop::CapU() {
  const Eigen::Matrix<double, 4, 1> error = R() - X_hat();

  double u_top = U(1, 0);
  double u_bottom = U(0, 0);

  uncapped_average_voltage_ = (u_top + u_bottom) / 2;

  double max_voltage = is_zeroing_ ? kZeroingVoltage : kMaxVoltage;

  if (::std::abs(u_bottom) > max_voltage || ::std::abs(u_top) > max_voltage) {
    LOG_MATRIX(DEBUG, "U at start", U());
    // H * U <= k
    // U = UPos + UVel
    // H * (UPos + UVel) <= k
    // H * UPos <= k - H * UVel

    // Now, we can do a coordinate transformation and say the following.

    // UPos = position_K * position_error
    // (H * position_K) * position_error <= k - H * UVel

    Eigen::Matrix<double, 2, 2> position_K;
    position_K << K(0, 0), K(0, 1),
                  K(1, 0), K(1, 1);
    Eigen::Matrix<double, 2, 2> velocity_K;
    velocity_K << K(0, 2), K(0, 3),
                  K(1, 2), K(1, 3);

    Eigen::Matrix<double, 2, 1> position_error;
    position_error << error(0, 0), error(1, 0);
    Eigen::Matrix<double, 2, 1> velocity_error;
    velocity_error << error(2, 0), error(3, 0);
    LOG_MATRIX(DEBUG, "error", error);

    const auto &poly = is_zeroing_ ? U_Poly_zeroing_ : U_Poly_;
    const Eigen::Matrix<double, 4, 2> pos_poly_H = poly.H() * position_K;
    const Eigen::Matrix<double, 4, 1> pos_poly_k =
        poly.k() - poly.H() * velocity_K * velocity_error;
    const ::aos::controls::HPolytope<2> pos_poly(pos_poly_H, pos_poly_k);

    Eigen::Matrix<double, 2, 1> adjusted_pos_error;
    {
      const auto &P = position_error;

      // This line was at 45 degrees but is now at some angle steeper than the
      // straight one between the points.
      Eigen::Matrix<double, 1, 2> angle_45;
      // If the top claw is above its soft upper limit, make the line actually
      // 45 degrees to avoid smashing it into the limit in an attempt to fix the
      // separation error faster than the bottom position one.
      if (X_hat(0, 0) + X_hat(1, 0) >
          constants::GetValues().claw.upper_claw.upper_limit) {
        angle_45 << 1, 1;
      } else {
        // Fixing separation error half as fast as positional error works well
        // because it means they both close evenly.
        angle_45 << ::std::sqrt(3), 1;
      }
      Eigen::Matrix<double, 1, 2> L45_quadrant;
      L45_quadrant << ::aos::sign(P(1, 0)), -::aos::sign(P(0, 0));
      const auto L45 = L45_quadrant.cwiseProduct(angle_45);
      const double w45 = 0;

      Eigen::Matrix<double, 1, 2> LH;
      if (::std::abs(P(0, 0)) > ::std::abs(P(1, 0))) {
        LH << 0, 1;
      } else {
        LH << 1, 0;
      }
      const double wh = LH.dot(P);

      Eigen::Matrix<double, 2, 2> standard;
      standard << L45, LH;
      Eigen::Matrix<double, 2, 1> W;
      W << w45, wh;
      const Eigen::Matrix<double, 2, 1> intersection = standard.inverse() * W;

      bool is_inside_h;
      const auto adjusted_pos_error_h =
          DoCoerceGoal(pos_poly, LH, wh, position_error, &is_inside_h);
      const auto adjusted_pos_error_45 =
          DoCoerceGoal(pos_poly, L45, w45, intersection, nullptr);
      if (pos_poly.IsInside(intersection)) {
        adjusted_pos_error = adjusted_pos_error_h;
      } else {
        if (is_inside_h) {
          if (adjusted_pos_error_h.norm() > adjusted_pos_error_45.norm()) {
            adjusted_pos_error = adjusted_pos_error_h;
          } else {
            adjusted_pos_error = adjusted_pos_error_45;
          }
        } else {
          adjusted_pos_error = adjusted_pos_error_45;
        }
      }
    }

    LOG_MATRIX(DEBUG, "adjusted_pos_error", adjusted_pos_error);
    mutable_U() = velocity_K * velocity_error + position_K * adjusted_pos_error;
    LOG_MATRIX(DEBUG, "U is now", U());

    {
      const auto values = constants::GetValues().claw;
      if (top_known_) {
        if (X_hat(0, 0) + X_hat(1, 0) > values.upper_claw.upper_limit && U(1, 0) > 0) {
          LOG(WARNING, "upper claw too high and moving up\n");
          mutable_U(1, 0) = 0;
        } else if (X_hat(0, 0) + X_hat(1, 0) < values.upper_claw.lower_limit &&
                   U(1, 0) < 0) {
          LOG(WARNING, "upper claw too low and moving down\n");
          mutable_U(1, 0) = 0;
        }
      }
      if (bottom_known_) {
        if (X_hat(0, 0) > values.lower_claw.upper_limit && U(0, 0) > 0) {
          LOG(WARNING, "lower claw too high and moving up\n");
          mutable_U(0, 0) = 0;
        } else if (X_hat(0, 0) < values.lower_claw.lower_limit && U(0, 0) < 0) {
          LOG(WARNING, "lower claw too low and moving down\n");
          mutable_U(0, 0) = 0;
        }
      }
    }
  }
}

ZeroedStateFeedbackLoop::ZeroedStateFeedbackLoop(const char *name,
                                                 ClawMotor *motor)
    : offset_(0.0),
      name_(name),
      motor_(motor),
      zeroing_state_(UNKNOWN_POSITION),
      encoder_(0.0),
      last_encoder_(0.0) {}

void ZeroedStateFeedbackLoop::SetPositionValues(
    const ::y2014::control_loops::HalfClawPosition &claw) {
  front_.Update(claw.front);
  calibration_.Update(claw.calibration);
  back_.Update(claw.back);

  bool any_sensor_triggered = any_triggered();
  if (any_sensor_triggered && any_triggered_last_) {
    // We are still on the hall effect and nothing has changed.
    min_hall_effect_on_angle_ =
        ::std::min(min_hall_effect_on_angle_, claw.position);
    max_hall_effect_on_angle_ =
        ::std::max(max_hall_effect_on_angle_, claw.position);
  } else if (!any_sensor_triggered && !any_triggered_last_) {
    // We are still off the hall effect and nothing has changed.
    min_hall_effect_off_angle_ =
        ::std::min(min_hall_effect_off_angle_, claw.position);
    max_hall_effect_off_angle_ =
        ::std::max(max_hall_effect_off_angle_, claw.position);
  }

  if (front_.is_posedge()) {
    // Saw a posedge on the hall effect.  Reset the limits.
    min_hall_effect_on_angle_ =
        ::std::min(claw.front.posedge_value, claw.position);
    max_hall_effect_on_angle_ =
        ::std::max(claw.front.posedge_value, claw.position);
  }
  if (calibration_.is_posedge()) {
    // Saw a posedge on the hall effect.  Reset the limits.
    min_hall_effect_on_angle_ =
        ::std::min(claw.calibration.posedge_value, claw.position);
    max_hall_effect_on_angle_ =
        ::std::max(claw.calibration.posedge_value, claw.position);
  }
  if (back_.is_posedge()) {
    // Saw a posedge on the hall effect.  Reset the limits.
    min_hall_effect_on_angle_ =
        ::std::min(claw.back.posedge_value, claw.position);
    max_hall_effect_on_angle_ =
        ::std::max(claw.back.posedge_value, claw.position);
  }

  if (front_.is_negedge()) {
    // Saw a negedge on the hall effect.  Reset the limits.
    min_hall_effect_off_angle_ =
        ::std::min(claw.front.negedge_value, claw.position);
    max_hall_effect_off_angle_ =
        ::std::max(claw.front.negedge_value, claw.position);
  }
  if (calibration_.is_negedge()) {
    // Saw a negedge on the hall effect.  Reset the limits.
    min_hall_effect_off_angle_ =
        ::std::min(claw.calibration.negedge_value, claw.position);
    max_hall_effect_off_angle_ =
        ::std::max(claw.calibration.negedge_value, claw.position);
  }
  if (back_.is_negedge()) {
    // Saw a negedge on the hall effect.  Reset the limits.
    min_hall_effect_off_angle_ =
        ::std::min(claw.back.negedge_value, claw.position);
    max_hall_effect_off_angle_ =
        ::std::max(claw.back.negedge_value, claw.position);
  }

  last_encoder_ = encoder_;
  if (front().value() || calibration().value() || back().value()) {
    last_on_encoder_ = encoder_;
  } else {
    last_off_encoder_ = encoder_;
  }
  encoder_ = claw.position;
  any_triggered_last_ = any_sensor_triggered;
}

void ZeroedStateFeedbackLoop::Reset(
    const ::y2014::control_loops::HalfClawPosition &claw) {
  set_zeroing_state(ZeroedStateFeedbackLoop::UNKNOWN_POSITION);

  front_.Reset(claw.front);
  calibration_.Reset(claw.calibration);
  back_.Reset(claw.back);
  // close up the min and max edge positions as they are no longer valid and
  // will be expanded in future iterations
  min_hall_effect_on_angle_ = claw.position;
  max_hall_effect_on_angle_ = claw.position;
  min_hall_effect_off_angle_ = claw.position;
  max_hall_effect_off_angle_ = claw.position;
  any_triggered_last_ = any_triggered();
}

bool TopZeroedStateFeedbackLoop::SetCalibrationOnEdge(
    const constants::Values::Claws::Claw &claw_values,
    JointZeroingState zeroing_state) {
  double edge_encoder;
  double edge_angle;
  if (GetPositionOfEdge(claw_values, &edge_encoder, &edge_angle)) {
    LOG(INFO, "Calibration edge edge should be %f.\n", edge_angle);
    SetCalibration(edge_encoder, edge_angle);
    set_zeroing_state(zeroing_state);
    return true;
  }
  return false;
}

void TopZeroedStateFeedbackLoop::HandleCalibrationError(
    const constants::Values::Claws::Claw &claw_values) {
  double edge_encoder;
  double edge_angle;
  if (GetPositionOfEdge(claw_values, &edge_encoder, &edge_angle)) {
    const double calibration_error =
        ComputeCalibrationChange(edge_encoder, edge_angle);
    LOG(INFO, "Top calibration error is %f\n", calibration_error);
    if (::std::abs(calibration_error) > kRezeroThreshold) {
      LOG(WARNING, "rezeroing top\n");
      SetCalibration(edge_encoder, edge_angle);
      set_zeroing_state(ZeroedStateFeedbackLoop::UNKNOWN_POSITION);
    }
  }
}


void BottomZeroedStateFeedbackLoop::HandleCalibrationError(
    const constants::Values::Claws::Claw &claw_values) {
  double edge_encoder;
  double edge_angle;
  if (GetPositionOfEdge(claw_values, &edge_encoder, &edge_angle)) {
    const double calibration_error =
        ComputeCalibrationChange(edge_encoder, edge_angle);
    LOG(INFO, "Bottom calibration error is %f\n", calibration_error);
    if (::std::abs(calibration_error) > kRezeroThreshold) {
      LOG(WARNING, "rezeroing bottom\n");
      SetCalibration(edge_encoder, edge_angle);
      set_zeroing_state(ZeroedStateFeedbackLoop::UNKNOWN_POSITION);
    }
  }
}

bool BottomZeroedStateFeedbackLoop::SetCalibrationOnEdge(
    const constants::Values::Claws::Claw &claw_values,
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

ClawMotor::ClawMotor(::y2014::control_loops::ClawQueue *my_claw)
    : aos::controls::ControlLoop<::y2014::control_loops::ClawQueue>(my_claw),
      has_top_claw_goal_(false),
      top_claw_goal_(0.0),
      top_claw_(this),
      has_bottom_claw_goal_(false),
      bottom_claw_goal_(0.0),
      bottom_claw_(this),
      claw_(::y2014::control_loops::claw::MakeClawLoop()),
      was_enabled_(false),
      doing_calibration_fine_tune_(false),
      capped_goal_(false),
      mode_(UNKNOWN_LOCATION) {}

const int ZeroedStateFeedbackLoop::kZeroingMaxVoltage;

bool ZeroedStateFeedbackLoop::SawFilteredPosedge(
    const HallEffectTracker &this_sensor, const HallEffectTracker &sensorA,
    const HallEffectTracker &sensorB) {
  if (posedge_filter_ == nullptr && this_sensor.posedge_count_changed() &&
      !sensorA.posedge_count_changed() && !sensorB.posedge_count_changed() &&
      this_sensor.value() && !this_sensor.last_value()) {
    posedge_filter_ = &this_sensor;
  } else if (posedge_filter_ == &this_sensor &&
             !this_sensor.posedge_count_changed() &&
             !sensorA.posedge_count_changed() &&
             !sensorB.posedge_count_changed() && this_sensor.value()) {
    posedge_filter_ = nullptr;
    return true;
  } else if (posedge_filter_ == &this_sensor) {
    posedge_filter_ = nullptr;
  }
  return false;
}

bool ZeroedStateFeedbackLoop::SawFilteredNegedge(
    const HallEffectTracker &this_sensor, const HallEffectTracker &sensorA,
    const HallEffectTracker &sensorB) {
  if (negedge_filter_ == nullptr && this_sensor.negedge_count_changed() &&
      !sensorA.negedge_count_changed() && !sensorB.negedge_count_changed() &&
      !this_sensor.value() && this_sensor.last_value()) {
    negedge_filter_ = &this_sensor;
  } else if (negedge_filter_ == &this_sensor &&
             !this_sensor.negedge_count_changed() &&
             !sensorA.negedge_count_changed() &&
             !sensorB.negedge_count_changed() && !this_sensor.value()) {
    negedge_filter_ = nullptr;
    return true;
  } else if (negedge_filter_ == &this_sensor) {
    negedge_filter_ = nullptr;
  }
  return false;
}

bool ZeroedStateFeedbackLoop::DoGetPositionOfEdge(
    const constants::Values::Claws::AnglePair &angles, double *edge_encoder,
    double *edge_angle, const HallEffectTracker &this_sensor,
    const HallEffectTracker &sensorA, const HallEffectTracker &sensorB,
    const char *hall_effect_name) {
  bool found_edge = false;

  if (SawFilteredPosedge(this_sensor, sensorA, sensorB)) {
    if (min_hall_effect_off_angle_ == max_hall_effect_off_angle_) {
      LOG(WARNING, "%s: Uncertain which side, rejecting posedge\n", name_);
    } else {
      const double average_last_encoder =
          (min_hall_effect_off_angle_ + max_hall_effect_off_angle_) / 2.0;
      if (this_sensor.posedge_value() < average_last_encoder) {
        *edge_angle = angles.upper_decreasing_angle;
        LOG(INFO, "%s Posedge upper of %s -> %f posedge: %f avg_encoder: %f\n",
            name_, hall_effect_name, *edge_angle, this_sensor.posedge_value(),
            average_last_encoder);
      } else {
        *edge_angle = angles.lower_angle;
        LOG(INFO, "%s Posedge lower of %s -> %f posedge: %f avg_encoder: %f\n",
            name_, hall_effect_name, *edge_angle, this_sensor.posedge_value(),
            average_last_encoder);
      }
      *edge_encoder = this_sensor.posedge_value();
      found_edge = true;
    }
  }

  if (SawFilteredNegedge(this_sensor, sensorA, sensorB)) {
    if (min_hall_effect_on_angle_ == max_hall_effect_on_angle_) {
      LOG(WARNING, "%s: Uncertain which side, rejecting negedge\n", name_);
    } else {
      const double average_last_encoder =
          (min_hall_effect_on_angle_ + max_hall_effect_on_angle_) / 2.0;
      if (this_sensor.negedge_value() > average_last_encoder) {
        *edge_angle = angles.upper_angle;
        LOG(INFO, "%s Negedge upper of %s -> %f negedge: %f avg_encoder: %f\n",
            name_, hall_effect_name, *edge_angle, this_sensor.negedge_value(),
            average_last_encoder);
      } else {
        *edge_angle = angles.lower_decreasing_angle;
        LOG(INFO, "%s Negedge lower of %s -> %f negedge: %f avg_encoder: %f\n",
            name_, hall_effect_name, *edge_angle, this_sensor.negedge_value(),
            average_last_encoder);
      }
      *edge_encoder = this_sensor.negedge_value();
      found_edge = true;
    }
  }

  return found_edge;
}

bool ZeroedStateFeedbackLoop::GetPositionOfEdge(
    const constants::Values::Claws::Claw &claw_values, double *edge_encoder,
    double *edge_angle) {
  if (DoGetPositionOfEdge(claw_values.front, edge_encoder, edge_angle, front_,
                          calibration_, back_, "front")) {
    return true;
  }
  if (DoGetPositionOfEdge(claw_values.calibration, edge_encoder, edge_angle,
                          calibration_, front_, back_, "calibration")) {
    return true;
  }
  if (DoGetPositionOfEdge(claw_values.back, edge_encoder, edge_angle, back_,
                          calibration_, front_, "back")) {
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

double TopZeroedStateFeedbackLoop::ComputeCalibrationChange(double edge_encoder,
                                                            double edge_angle) {
  const double offset = edge_angle - edge_encoder;
  const double doffset = offset - offset_;
  return doffset;
}

void BottomZeroedStateFeedbackLoop::SetCalibration(double edge_encoder,
                                                   double edge_angle) {
  double old_offset = offset_;
  offset_ = edge_angle - edge_encoder;
  const double doffset = offset_ - old_offset;
  motor_->ChangeBottomOffset(doffset);
}

double BottomZeroedStateFeedbackLoop::ComputeCalibrationChange(
    double edge_encoder, double edge_angle) {
  const double offset = edge_angle - edge_encoder;
  const double doffset = offset - offset_;
  return doffset;
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
  mutable_X_hat()(1, 0) += doffset;
  LOG(INFO, "Changing top offset by %f\n", doffset);
}
void ClawLimitedLoop::ChangeBottomOffset(double doffset) {
  mutable_X_hat()(0, 0) += doffset;
  mutable_X_hat()(1, 0) -= doffset;
  LOG(INFO, "Changing bottom offset by %f\n", doffset);
}

void LimitClawGoal(double *bottom_goal, double *top_goal,
                   const constants::Values &values) {
  // first update position based on angle limit
  const double separation = *top_goal - *bottom_goal;
  if (separation > values.claw.soft_max_separation) {
    LOG_STRUCT(DEBUG, "before", ClawPositionToLog(*top_goal, *bottom_goal));
    const double dsep = (separation - values.claw.soft_max_separation) / 2.0;
    *bottom_goal += dsep;
    *top_goal -= dsep;
    LOG_STRUCT(DEBUG, "after", ClawPositionToLog(*top_goal, *bottom_goal));
  }
  if (separation < values.claw.soft_min_separation) {
    LOG_STRUCT(DEBUG, "before", ClawPositionToLog(*top_goal, *bottom_goal));
    const double dsep = (separation - values.claw.soft_min_separation) / 2.0;
    *bottom_goal += dsep;
    *top_goal -= dsep;
    LOG_STRUCT(DEBUG, "after", ClawPositionToLog(*top_goal, *bottom_goal));
  }

  // now move both goals in unison
  if (*bottom_goal < values.claw.lower_claw.lower_limit) {
    LOG_STRUCT(DEBUG, "before", ClawPositionToLog(*top_goal, *bottom_goal));
    *top_goal += values.claw.lower_claw.lower_limit - *bottom_goal;
    *bottom_goal = values.claw.lower_claw.lower_limit;
    LOG_STRUCT(DEBUG, "after", ClawPositionToLog(*top_goal, *bottom_goal));
  }
  if (*bottom_goal > values.claw.lower_claw.upper_limit) {
    LOG_STRUCT(DEBUG, "before", ClawPositionToLog(*top_goal, *bottom_goal));
    *top_goal -= *bottom_goal - values.claw.lower_claw.upper_limit;
    *bottom_goal = values.claw.lower_claw.upper_limit;
    LOG_STRUCT(DEBUG, "after", ClawPositionToLog(*top_goal, *bottom_goal));
  }

  if (*top_goal < values.claw.upper_claw.lower_limit) {
    LOG_STRUCT(DEBUG, "before", ClawPositionToLog(*top_goal, *bottom_goal));
    *bottom_goal += values.claw.upper_claw.lower_limit - *top_goal;
    *top_goal = values.claw.upper_claw.lower_limit;
    LOG_STRUCT(DEBUG, "after", ClawPositionToLog(*top_goal, *bottom_goal));
  }
  if (*top_goal > values.claw.upper_claw.upper_limit) {
    LOG_STRUCT(DEBUG, "before", ClawPositionToLog(*top_goal, *bottom_goal));
    *bottom_goal -= *top_goal - values.claw.upper_claw.upper_limit;
    *top_goal = values.claw.upper_claw.upper_limit;
    LOG_STRUCT(DEBUG, "after", ClawPositionToLog(*top_goal, *bottom_goal));
  }
}

bool ClawMotor::is_ready() const {
  return (
      (top_claw_.zeroing_state() == ZeroedStateFeedbackLoop::CALIBRATED &&
       bottom_claw_.zeroing_state() == ZeroedStateFeedbackLoop::CALIBRATED) ||
      (((::aos::joystick_state.get() == NULL)
            ? true
            : ::aos::joystick_state->autonomous) &&
       ((top_claw_.zeroing_state() == ZeroedStateFeedbackLoop::CALIBRATED ||
         top_claw_.zeroing_state() ==
             ZeroedStateFeedbackLoop::DISABLED_CALIBRATION) &&
        (bottom_claw_.zeroing_state() == ZeroedStateFeedbackLoop::CALIBRATED ||
         bottom_claw_.zeroing_state() ==
             ZeroedStateFeedbackLoop::DISABLED_CALIBRATION))));
}

bool ClawMotor::is_zeroing() const { return !is_ready(); }

// Positive angle is up, and positive power is up.
void ClawMotor::RunIteration(
    const ::y2014::control_loops::ClawQueue::Goal *goal,
    const ::y2014::control_loops::ClawQueue::Position *position,
    ::y2014::control_loops::ClawQueue::Output *output,
    ::y2014::control_loops::ClawQueue::Status *status) {
  // Disable the motors now so that all early returns will return with the
  // motors disabled.
  if (output) {
    output->top_claw_voltage = 0;
    output->bottom_claw_voltage = 0;
    output->intake_voltage = 0;
    output->tusk_voltage = 0;
  }

  if (goal) {
    if (::std::isnan(goal->bottom_angle) ||
        ::std::isnan(goal->separation_angle) || ::std::isnan(goal->intake) ||
        ::std::isnan(goal->centering)) {
      return;
    }
  }

  if (WasReset()) {
    top_claw_.Reset(position->top);
    bottom_claw_.Reset(position->bottom);
  }

  const constants::Values &values = constants::GetValues();

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
    LOG_STRUCT(DEBUG, "absolute position",
               ClawPositionToLog(top_claw_.absolute_position(),
                                 bottom_claw_.absolute_position()));
  }

  bool autonomous, enabled;
  if (::aos::joystick_state.get() == nullptr) {
    autonomous = true;
    enabled = false;
  } else {
    autonomous = ::aos::joystick_state->autonomous;
    enabled = ::aos::joystick_state->enabled;
  }

  double bottom_claw_velocity_ = 0.0;
  double top_claw_velocity_ = 0.0;

  if (goal != NULL &&
      ((top_claw_.zeroing_state() == ZeroedStateFeedbackLoop::CALIBRATED &&
        bottom_claw_.zeroing_state() == ZeroedStateFeedbackLoop::CALIBRATED) ||
       (autonomous &&
        ((top_claw_.zeroing_state() == ZeroedStateFeedbackLoop::CALIBRATED ||
          top_claw_.zeroing_state() ==
              ZeroedStateFeedbackLoop::DISABLED_CALIBRATION) &&
         (bottom_claw_.zeroing_state() == ZeroedStateFeedbackLoop::CALIBRATED ||
          bottom_claw_.zeroing_state() ==
              ZeroedStateFeedbackLoop::DISABLED_CALIBRATION))))) {
    // Ready to use the claw.
    // Limit the goals here.
    bottom_claw_goal_ = goal->bottom_angle;
    top_claw_goal_ = goal->bottom_angle + goal->separation_angle;
    has_bottom_claw_goal_ = true;
    has_top_claw_goal_ = true;
    doing_calibration_fine_tune_ = false;
    mode_ = READY;

    bottom_claw_.HandleCalibrationError(values.claw.lower_claw);
    top_claw_.HandleCalibrationError(values.claw.upper_claw);
  } else if (top_claw_.zeroing_state() !=
             ZeroedStateFeedbackLoop::UNKNOWN_POSITION &&
             bottom_claw_.zeroing_state() !=
             ZeroedStateFeedbackLoop::UNKNOWN_POSITION) {
    // Time to fine tune the zero.
    // Limit the goals here.
    if (!enabled) {
      // If we are disabled, start the fine tune process over again.
      doing_calibration_fine_tune_ = false;
    }
    if (bottom_claw_.zeroing_state() != ZeroedStateFeedbackLoop::CALIBRATED) {
      // always get the bottom claw to calibrated first
      LOG(DEBUG, "Calibrating the bottom of the claw\n");
      if (!doing_calibration_fine_tune_) {
        if (::std::abs(bottom_absolute_position() -
                       values.claw.start_fine_tune_pos) <
            values.claw.claw_unimportant_epsilon) {
          doing_calibration_fine_tune_ = true;
          bottom_claw_goal_ += values.claw.claw_zeroing_speed * kDt;
          top_claw_velocity_ = bottom_claw_velocity_ =
              values.claw.claw_zeroing_speed;
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
        bottom_claw_goal_ += values.claw.claw_zeroing_speed * kDt;
        top_claw_velocity_ = bottom_claw_velocity_ =
            values.claw.claw_zeroing_speed;
        if (top_claw_.front_or_back_triggered() ||
            bottom_claw_.front_or_back_triggered()) {
          // We shouldn't hit a limit, but if we do, go back to the zeroing
          // point and try again.
          doing_calibration_fine_tune_ = false;
          bottom_claw_goal_ = values.claw.start_fine_tune_pos;
          top_claw_velocity_ = bottom_claw_velocity_ = 0.0;
          LOG(DEBUG, "Found a limit, starting over.\n");
          mode_ = PREP_FINE_TUNE_BOTTOM;
        }

        if (position && bottom_claw_.SawFilteredPosedge(
                            bottom_claw_.calibration(), bottom_claw_.front(),
                            bottom_claw_.back())) {
          // do calibration
          bottom_claw_.SetCalibration(
              position->bottom.calibration.posedge_value,
              values.claw.lower_claw.calibration.lower_angle);
          bottom_claw_.set_zeroing_state(ZeroedStateFeedbackLoop::CALIBRATED);
          // calibrated so we are done fine tuning bottom
          doing_calibration_fine_tune_ = false;
          LOG(DEBUG, "Calibrated the bottom correctly!\n");
        } else if (bottom_claw_.calibration().last_value()) {
          LOG(DEBUG, "Aborting bottom fine tune because sensor triggered\n");
          doing_calibration_fine_tune_ = false;
          bottom_claw_.set_zeroing_state(
              ZeroedStateFeedbackLoop::UNKNOWN_POSITION);
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
          top_claw_goal_ += values.claw.claw_zeroing_speed * kDt;
          top_claw_velocity_ = bottom_claw_velocity_ =
              values.claw.claw_zeroing_speed;
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
        top_claw_goal_ += values.claw.claw_zeroing_speed * kDt;
        top_claw_velocity_ = bottom_claw_velocity_ =
            values.claw.claw_zeroing_speed;
        if (top_claw_.front_or_back_triggered() ||
            bottom_claw_.front_or_back_triggered()) {
          // this should not happen, but now we know it won't
          doing_calibration_fine_tune_ = false;
          top_claw_goal_ = values.claw.start_fine_tune_pos;
          top_claw_velocity_ = bottom_claw_velocity_ = 0.0;
          LOG(DEBUG, "Found a limit, starting over.\n");
          mode_ = PREP_FINE_TUNE_TOP;
        }

        if (position &&
            top_claw_.SawFilteredPosedge(top_claw_.calibration(),
                                         top_claw_.front(), top_claw_.back())) {
          // do calibration
          top_claw_.SetCalibration(
              position->top.calibration.posedge_value,
              values.claw.upper_claw.calibration.lower_angle);
          top_claw_.set_zeroing_state(ZeroedStateFeedbackLoop::CALIBRATED);
          // calibrated so we are done fine tuning top
          doing_calibration_fine_tune_ = false;
          LOG(DEBUG, "Calibrated the top correctly!\n");
        } else if (top_claw_.calibration().last_value()) {
          LOG(DEBUG, "Aborting top fine tune because sensor triggered\n");
          doing_calibration_fine_tune_ = false;
          top_claw_.set_zeroing_state(
              ZeroedStateFeedbackLoop::UNKNOWN_POSITION);
        }
      }
      // now set the bottom claw to track
      bottom_claw_goal_ = top_claw_goal_ - values.claw.claw_zeroing_separation;
    }
  } else {
    doing_calibration_fine_tune_ = false;
    if (!was_enabled_ && enabled) {
      if (position) {
        top_claw_goal_ = position->top.position + top_claw_.offset();
        bottom_claw_goal_ = position->bottom.position + bottom_claw_.offset();
        initial_separation_ =
            position->top.position - position->bottom.position;
      } else {
        has_top_claw_goal_ = false;
        has_bottom_claw_goal_ = false;
      }
    }

    if ((bottom_claw_.zeroing_state() !=
             ZeroedStateFeedbackLoop::UNKNOWN_POSITION ||
         bottom_claw_.front().value() || top_claw_.front().value()) &&
        !top_claw_.back().value() && !bottom_claw_.back().value()) {
      if (enabled) {
        // Time to slowly move back up to find any position to narrow down the
        // zero.
        top_claw_goal_ += values.claw.claw_zeroing_off_speed * kDt;
        bottom_claw_goal_ += values.claw.claw_zeroing_off_speed * kDt;
        top_claw_velocity_ = bottom_claw_velocity_ =
            values.claw.claw_zeroing_off_speed;
        LOG(DEBUG, "Bottom is known.\n");
      }
    } else {
      // We don't know where either claw is.  Slowly start moving down to find
      // any hall effect.
      if (enabled) {
        top_claw_goal_ -= values.claw.claw_zeroing_off_speed * kDt;
        bottom_claw_goal_ -= values.claw.claw_zeroing_off_speed * kDt;
        top_claw_velocity_ = bottom_claw_velocity_ =
            -values.claw.claw_zeroing_off_speed;
        LOG(DEBUG, "Both are unknown.\n");
      }
    }

    if (position) {
      if (enabled) {
        top_claw_.SetCalibrationOnEdge(
            values.claw.upper_claw,
            ZeroedStateFeedbackLoop::APPROXIMATE_CALIBRATION);
        bottom_claw_.SetCalibrationOnEdge(
            values.claw.lower_claw,
            ZeroedStateFeedbackLoop::APPROXIMATE_CALIBRATION);
      } else {
        // TODO(austin): Only calibrate on the predetermined edge.
        // We might be able to just ignore this since the backlash is soooo
        // low.
        // :)
        top_claw_.SetCalibrationOnEdge(
            values.claw.upper_claw,
            ZeroedStateFeedbackLoop::DISABLED_CALIBRATION);
        bottom_claw_.SetCalibrationOnEdge(
            values.claw.lower_claw,
            ZeroedStateFeedbackLoop::DISABLED_CALIBRATION);
      }
    }
    mode_ = UNKNOWN_LOCATION;
  }

  // Limit the goals if both claws have been (mostly) found.
  if (mode_ != UNKNOWN_LOCATION) {
    LimitClawGoal(&bottom_claw_goal_, &top_claw_goal_, values);
  }

  claw_.set_positions_known(
      top_claw_.zeroing_state() != ZeroedStateFeedbackLoop::UNKNOWN_POSITION,
      bottom_claw_.zeroing_state() !=
          ZeroedStateFeedbackLoop::UNKNOWN_POSITION);
  if (has_top_claw_goal_ && has_bottom_claw_goal_) {
    claw_.mutable_R() << bottom_claw_goal_, top_claw_goal_ - bottom_claw_goal_,
        bottom_claw_velocity_, top_claw_velocity_ - bottom_claw_velocity_;
    LOG_MATRIX(DEBUG, "actual goal", claw_.R());

    // Only cap power when one of the halves of the claw is moving slowly and
    // could wind up.
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
        double dx_bot = (claw_.U_uncapped(0, 0) -
                     values.claw.max_zeroing_voltage) /
                    claw_.K(0, 0);
        double dx_top = (claw_.U_uncapped(1, 0) -
                     values.claw.max_zeroing_voltage) /
                    claw_.K(0, 0);
        double dx = ::std::max(dx_top, dx_bot);
        bottom_claw_goal_ -= dx;
        top_claw_goal_ -= dx;
        Eigen::Matrix<double, 4, 1> R;
        R << bottom_claw_goal_, top_claw_goal_ - bottom_claw_goal_, claw_.R(2, 0),
            claw_.R(3, 0);
        claw_.mutable_U() = claw_.K() * (R - claw_.X_hat());
        capped_goal_ = true;
        LOG(DEBUG, "Moving the goal by %f to prevent windup."
            " Uncapped is %f, max is %f, difference is %f\n",
            dx,
            claw_.uncapped_average_voltage(), values.claw.max_zeroing_voltage,
            (claw_.uncapped_average_voltage() -
             values.claw.max_zeroing_voltage));
      } else if (claw_.uncapped_average_voltage() <
                 -values.claw.max_zeroing_voltage) {
        double dx_bot = (claw_.U_uncapped(0, 0) +
                     values.claw.max_zeroing_voltage) /
                    claw_.K(0, 0);
        double dx_top = (claw_.U_uncapped(1, 0) +
                     values.claw.max_zeroing_voltage) /
                    claw_.K(0, 0);
        double dx = ::std::min(dx_top, dx_bot);
        bottom_claw_goal_ -= dx;
        top_claw_goal_ -= dx;
        Eigen::Matrix<double, 4, 1> R;
        R << bottom_claw_goal_, top_claw_goal_ - bottom_claw_goal_, claw_.R(2, 0),
            claw_.R(3, 0);
        claw_.mutable_U() = claw_.K() * (R - claw_.X_hat());
        capped_goal_ = true;
        LOG(DEBUG, "Moving the goal by %f to prevent windup\n", dx);
      }
    } break;
  }

  if (output) {
    if (goal) {
      //setup the intake
      output->intake_voltage =
          (goal->intake > 12.0) ? 12 : (goal->intake < -12.0) ? -12.0
                                                              : goal->intake;
      output->tusk_voltage = goal->centering;
      output->tusk_voltage =
          (goal->centering > 12.0) ? 12 : (goal->centering < -12.0)
              ? -12.0
              : goal->centering;
    }
    output->top_claw_voltage = claw_.U(1, 0);
    output->bottom_claw_voltage = claw_.U(0, 0);

    if (output->top_claw_voltage > kMaxVoltage) {
      output->top_claw_voltage = kMaxVoltage;
    } else if (output->top_claw_voltage < -kMaxVoltage) {
      output->top_claw_voltage = -kMaxVoltage;
    }

    if (output->bottom_claw_voltage > kMaxVoltage) {
      output->bottom_claw_voltage = kMaxVoltage;
    } else if (output->bottom_claw_voltage < -kMaxVoltage) {
      output->bottom_claw_voltage = -kMaxVoltage;
    }
  }

  status->bottom = bottom_absolute_position();
  status->separation = top_absolute_position() - bottom_absolute_position();
  status->bottom_velocity = claw_.X_hat(2, 0);
  status->separation_velocity = claw_.X_hat(3, 0);

  if (goal) {
    bool bottom_done =
        ::std::abs(bottom_absolute_position() - goal->bottom_angle) < 0.020;
    bool bottom_velocity_done = ::std::abs(status->bottom_velocity) < 0.2;
    bool separation_done =
        ::std::abs((top_absolute_position() - bottom_absolute_position()) -
                   goal->separation_angle) < 0.020;
    bool separation_done_with_ball =
        ::std::abs((top_absolute_position() - bottom_absolute_position()) -
                   goal->separation_angle) < 0.06;
    status->done = is_ready() && separation_done && bottom_done && bottom_velocity_done;
    status->done_with_ball =
        is_ready() && separation_done_with_ball && bottom_done && bottom_velocity_done;
  } else {
    status->done = status->done_with_ball = false;
  }

  status->zeroed = is_ready();
  status->zeroed_for_auto =
      (top_claw_.zeroing_state() == ZeroedStateFeedbackLoop::CALIBRATED ||
       top_claw_.zeroing_state() ==
           ZeroedStateFeedbackLoop::DISABLED_CALIBRATION) &&
      (bottom_claw_.zeroing_state() == ZeroedStateFeedbackLoop::CALIBRATED ||
       bottom_claw_.zeroing_state() ==
           ZeroedStateFeedbackLoop::DISABLED_CALIBRATION);

  was_enabled_ = enabled;
}

}  // namespace control_loops
}  // namespace y2014
