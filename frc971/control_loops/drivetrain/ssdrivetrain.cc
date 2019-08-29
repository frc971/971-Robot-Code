#include "frc971/control_loops/drivetrain/ssdrivetrain.h"

#include "aos/commonmath.h"
#include "aos/controls/polytope.h"

#include "frc971/control_loops/coerce_goal.h"
#include "frc971/control_loops/drivetrain/drivetrain_config.h"
#include "frc971/control_loops/drivetrain/drivetrain_goal_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_output_generated.h"
#include "frc971/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc971/control_loops/state_feedback_loop.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

DrivetrainMotorsSS::DrivetrainMotorsSS(
    const DrivetrainConfig<double> &dt_config, StateFeedbackLoop<7, 2, 4> *kf,
    LocalizerInterface *localizer)
    : dt_config_(dt_config),
      kf_(kf),
      U_poly_(
          (Eigen::Matrix<double, 4, 2>() << /*[[*/ 1, 0 /*]*/,
           /*[*/ -1, 0 /*]*/,
           /*[*/ 0, 1 /*]*/,
           /*[*/ 0, -1 /*]]*/)
              .finished(),
          (Eigen::Matrix<double, 4, 1>() << /*[[*/ 1.0 /*]*/,
           /*[*/ 1.0 /*]*/,
           /*[*/ 1.0 /*]*/,
           /*[*/ 1.0 /*]]*/)
              .finished(),
          (Eigen::Matrix<double, 2, 4>() << /*[[*/ 1.0, 1.0, -1.0, -1.0 /*]*/,
           /*[*/ -1.0, 1.0, 1.0, -1.0 /*]*/)
              .finished()),
      linear_profile_(::aos::controls::kLoopFrequency),
      angular_profile_(::aos::controls::kLoopFrequency),
      localizer_(localizer) {
  ::aos::controls::HPolytope<0>::Init();
  T_ << 1, 1, 1, -1;
  T_inverse_ = T_.inverse();
  unprofiled_goal_.setZero();
}

void DrivetrainMotorsSS::ScaleCapU(Eigen::Matrix<double, 2, 1> *U) {
  output_was_capped_ = ::std::abs((*U)(0, 0)) > max_voltage_ ||
                       ::std::abs((*U)(1, 0)) > max_voltage_;

  if (output_was_capped_) {
    *U *= max_voltage_ / kf_->U_uncapped().lpNorm<Eigen::Infinity>();
  }
}

// This intentionally runs the U-capping code even when it's unnecessary to help
// make it more deterministic. Only running it when one or both sides want
// out-of-range voltages could lead to things like running out of CPU under
// certain situations, which would be bad.
void DrivetrainMotorsSS::PolyCapU(Eigen::Matrix<double, 2, 1> *U) {
  output_was_capped_ = ::std::abs((*U)(0, 0)) > max_voltage_ ||
                       ::std::abs((*U)(1, 0)) > max_voltage_;

  const Eigen::Matrix<double, 7, 1> error = kf_->R() - kf_->X_hat();

  Eigen::Matrix<double, 2, 2> position_K;
  position_K << kf_->controller().K(0, 0), kf_->controller().K(0, 2),
      kf_->controller().K(1, 0), kf_->controller().K(1, 2);
  Eigen::Matrix<double, 2, 2> velocity_K;
  velocity_K << kf_->controller().K(0, 1), kf_->controller().K(0, 3),
      kf_->controller().K(1, 1), kf_->controller().K(1, 3);

  Eigen::Matrix<double, 2, 1> position_error;
  position_error << error(0, 0), error(2, 0);
  // drive_error = [total_distance_error, left_error - right_error]
  const auto drive_error = T_inverse_ * position_error;
  Eigen::Matrix<double, 2, 1> velocity_error;
  velocity_error << error(1, 0), error(3, 0);

  Eigen::Matrix<double, 2, 1> U_integral;
  U_integral << kf_->X_hat(4, 0), kf_->X_hat(5, 0);

  const ::aos::controls::HVPolytope<2, 4, 4> pos_poly_hv(
      U_poly_.static_H() * position_K * T_,
      U_poly_.static_H() *
              (-velocity_K * velocity_error + U_integral - kf_->ff_U()) +
          (U_poly_.static_k() * max_voltage_),
      (position_K * T_).inverse() *
          ::aos::controls::ShiftPoints<2, 4, double>(
              (U_poly_.StaticVertices() * max_voltage_),
              -velocity_K * velocity_error + U_integral - kf_->ff_U()));

  Eigen::Matrix<double, 2, 1> adjusted_pos_error;
  {
    const auto &P = drive_error;

    Eigen::Matrix<double, 1, 2> L45;
    L45 << ::aos::sign(P(1, 0)), -::aos::sign(P(0, 0));
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

    bool is_inside_h, is_inside_45;
    const auto adjusted_pos_error_h =
        DoCoerceGoal<double>(pos_poly_hv, LH, wh, drive_error, &is_inside_h);
    const auto adjusted_pos_error_45 =
        DoCoerceGoal<double>(pos_poly_hv, L45, w45, intersection, &is_inside_45);
    if (pos_poly_hv.IsInside(intersection)) {
      adjusted_pos_error = adjusted_pos_error_h;
    } else {
      if (is_inside_h) {
        if (adjusted_pos_error_h.norm() > adjusted_pos_error_45.norm() ||
            adjusted_pos_error_45.norm() > intersection.norm()) {
          adjusted_pos_error = adjusted_pos_error_h;
        } else {
          adjusted_pos_error = adjusted_pos_error_45;
        }
      } else {
        adjusted_pos_error = adjusted_pos_error_45;
      }
    }
  }

  *U = -U_integral + velocity_K *velocity_error +
       position_K *T_ *adjusted_pos_error + kf_->ff_U();

  if (!output_was_capped_) {
    if ((*U - kf_->U_uncapped()).norm() > 0.0001) {
      AOS_LOG(FATAL, "U unnecessarily capped\n");
    }
  }
}

void DrivetrainMotorsSS::SetGoal(
    const ::frc971::control_loops::drivetrain::Goal *goal) {
  unprofiled_goal_ << goal->left_goal(), 0.0, goal->right_goal(), 0.0, 0.0, 0.0,
      0.0;
  if (!goal->has_max_ss_voltage()) {
    max_voltage_ = kMaxVoltage;
  } else {
    max_voltage_ = goal->has_max_ss_voltage();
  }

  use_profile_ = !kf_->controller().Kff().isZero(0) &&
                 (goal->has_linear() && goal->has_angular() &&
                  goal->linear()->has_max_velocity() &&
                  goal->linear()->has_max_acceleration() &&
                  goal->angular()->has_max_velocity() &&
                  goal->angular()->has_max_acceleration());
  if (goal->has_linear()) {
    linear_profile_.set_maximum_velocity(goal->linear()->max_velocity());
    linear_profile_.set_maximum_acceleration(
        goal->linear()->max_acceleration());
  }
  if (goal->has_angular()) {
    angular_profile_.set_maximum_velocity(goal->angular()->max_velocity());
    angular_profile_.set_maximum_acceleration(
        goal->angular()->max_acceleration());
  }
}

void DrivetrainMotorsSS::Update(bool enable_control_loop) {
  Eigen::Matrix<double, 2, 1> wheel_heading =
      dt_config_.LeftRightToAngular(kf_->X_hat());

  const double gyro_to_wheel_offset = wheel_heading(0, 0) - localizer_->theta();

  if (enable_control_loop) {
    // Update profiles.
    Eigen::Matrix<double, 2, 1> unprofiled_linear =
        dt_config_.LeftRightToLinear(unprofiled_goal_);
    Eigen::Matrix<double, 2, 1> unprofiled_angular =
        dt_config_.LeftRightToAngular(unprofiled_goal_);

    Eigen::Matrix<double, 2, 1> next_linear;
    Eigen::Matrix<double, 2, 1> next_angular;

    if (use_profile_) {
      next_linear = linear_profile_.Update(unprofiled_linear(0, 0),
                                           unprofiled_linear(1, 0));
      next_angular = angular_profile_.Update(unprofiled_angular(0, 0),
                                             unprofiled_angular(1, 0));
    } else {
      next_angular = unprofiled_angular;
      next_linear = unprofiled_linear;
    }

    const double wheel_compensation_offset =
        gyro_to_wheel_offset * dt_config_.robot_radius;
    const double scaled_angle_delta =
        (gyro_to_wheel_offset - last_gyro_to_wheel_offset_) *
        dt_config_.robot_radius;

    kf_->mutable_next_R().block<4, 1>(0, 0) =
        dt_config_.AngularLinearToLeftRight(next_linear, next_angular);

    kf_->mutable_next_R().block<3, 1>(4, 0) =
        unprofiled_goal_.block<3, 1>(4, 0);

    kf_->mutable_next_R(0, 0) -= wheel_compensation_offset;
    kf_->mutable_next_R(2, 0) += wheel_compensation_offset;

    if (!use_profile_) {
      kf_->mutable_R() = kf_->next_R();
    } else {
      kf_->mutable_R(0, 0) -= scaled_angle_delta;
      kf_->mutable_R(2, 0) += scaled_angle_delta;
    }

    // Run the controller.
    Eigen::Matrix<double, 2, 1> U = kf_->ControllerOutput();

    kf_->mutable_U_uncapped() = kf_->mutable_U() = U;
    ScaleCapU(&kf_->mutable_U());

    // Now update the feed forwards.
    kf_->UpdateFFReference();

    // Now, move the profile if things didn't go perfectly.
    if (use_profile_ &&
        (kf_->U() - kf_->U_uncapped()).lpNorm<Eigen::Infinity>() > 1e-4) {
      // kf_->R() is in wheel coordinates, while the profile is in absolute
      // coordinates.  Convert back...
      linear_profile_.MoveCurrentState(dt_config_.LeftRightToLinear(kf_->R()));

      AOS_LOG(DEBUG, "Saturated while moving\n");

      Eigen::Matrix<double, 2, 1> absolute_angular =
          dt_config_.LeftRightToAngular(kf_->R());
      absolute_angular(0, 0) -= gyro_to_wheel_offset;
      angular_profile_.MoveCurrentState(absolute_angular);
    }
  } else {
    Eigen::Matrix<double, 2, 1> wheel_linear =
        dt_config_.LeftRightToLinear(kf_->X_hat());
    Eigen::Matrix<double, 2, 1> next_angular = wheel_heading;
    next_angular(0, 0) = localizer_->theta();

    unprofiled_goal_.block<4, 1>(0, 0) =
        dt_config_.AngularLinearToLeftRight(wheel_linear, next_angular);

    auto current_linear = dt_config_.LeftRightToLinear(unprofiled_goal_);
    auto current_angular = dt_config_.LeftRightToAngular(unprofiled_goal_);
    linear_profile_.MoveCurrentState(current_linear);
    angular_profile_.MoveCurrentState(current_angular);

    kf_->mutable_next_R().block<4, 1>(0, 0) = kf_->X_hat().block<4, 1>(0, 0);
    kf_->mutable_R().block<4, 1>(0, 0) = kf_->X_hat().block<4, 1>(0, 0);
  }
  last_gyro_to_wheel_offset_ = gyro_to_wheel_offset;
}

void DrivetrainMotorsSS::SetOutput(
    ::frc971::control_loops::drivetrain::OutputT *output) const {
  if (output) {
    output->left_voltage = kf_->U(0, 0);
    output->right_voltage = kf_->U(1, 0);
    output->left_high = true;
    output->right_high = true;
  }
}

void DrivetrainMotorsSS::PopulateStatus(
    ::frc971::control_loops::drivetrain::StatusBuilder *builder) const {
  Eigen::Matrix<double, 2, 1> profiled_linear =
      dt_config_.LeftRightToLinear(kf_->next_R());
  Eigen::Matrix<double, 2, 1> profiled_angular =
      dt_config_.LeftRightToAngular(kf_->next_R());

  profiled_angular(0, 0) -= last_gyro_to_wheel_offset_;

  Eigen::Matrix<double, 4, 1> profiled_gyro_left_right =
      dt_config_.AngularLinearToLeftRight(profiled_linear, profiled_angular);

  builder->add_profiled_left_position_goal(profiled_gyro_left_right(0, 0));
  builder->add_profiled_left_velocity_goal(profiled_gyro_left_right(1, 0));
  builder->add_profiled_right_position_goal(profiled_gyro_left_right(2, 0));
  builder->add_profiled_right_velocity_goal(profiled_gyro_left_right(3, 0));
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
