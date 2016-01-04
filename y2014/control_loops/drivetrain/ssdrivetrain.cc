#include "y2014/control_loops/drivetrain/ssdrivetrain.h"

#include "aos/common/controls/polytope.h"
#include "aos/common/commonmath.h"
#include "aos/common/logging/matrix_logging.h"

#include "frc971/control_loops/state_feedback_loop.h"
#include "frc971/control_loops/coerce_goal.h"
#include "y2014/constants.h"
#include "y2014/control_loops/drivetrain/drivetrain.q.h"

namespace y2014 {
namespace control_loops {
namespace drivetrain {

using ::frc971::control_loops::DoCoerceGoal;

DrivetrainMotorsSS::LimitedDrivetrainLoop::LimitedDrivetrainLoop(
    StateFeedbackLoop<4, 2, 2> &&loop)
    : StateFeedbackLoop<4, 2, 2>(::std::move(loop)),
      U_Poly_((Eigen::Matrix<double, 4, 2>() << 1, 0, -1, 0, 0, 1, 0, -1)
                  .finished(),
              (Eigen::Matrix<double, 4, 1>() << 12.0, 12.0, 12.0, 12.0)
                  .finished()) {
  ::aos::controls::HPolytope<0>::Init();
  T << 1, -1, 1, 1;
  T_inverse = T.inverse();
}

void DrivetrainMotorsSS::LimitedDrivetrainLoop::CapU() {
  const Eigen::Matrix<double, 4, 1> error = R() - X_hat();

  if (::std::abs(U(0, 0)) > 12.0 || ::std::abs(U(1, 0)) > 12.0) {
    mutable_U() =
        U() * 12.0 / ::std::max(::std::abs(U(0, 0)), ::std::abs(U(1, 0)));
    LOG_MATRIX(DEBUG, "U is now", U());
    // TODO(Austin): Figure out why the polytope stuff wasn't working and
    // remove this hack.
    output_was_capped_ = true;
    return;

    LOG_MATRIX(DEBUG, "U at start", U());
    LOG_MATRIX(DEBUG, "R at start", R());
    LOG_MATRIX(DEBUG, "Xhat at start", X_hat());

    Eigen::Matrix<double, 2, 2> position_K;
    position_K << K(0, 0), K(0, 2), K(1, 0), K(1, 2);
    Eigen::Matrix<double, 2, 2> velocity_K;
    velocity_K << K(0, 1), K(0, 3), K(1, 1), K(1, 3);

    Eigen::Matrix<double, 2, 1> position_error;
    position_error << error(0, 0), error(2, 0);
    const auto drive_error = T_inverse * position_error;
    Eigen::Matrix<double, 2, 1> velocity_error;
    velocity_error << error(1, 0), error(3, 0);
    LOG_MATRIX(DEBUG, "error", error);

    const auto &poly = U_Poly_;
    const Eigen::Matrix<double, 4, 2> pos_poly_H = poly.H() * position_K * T;
    const Eigen::Matrix<double, 4, 1> pos_poly_k =
        poly.k() - poly.H() * velocity_K * velocity_error;
    const ::aos::controls::HPolytope<2> pos_poly(pos_poly_H, pos_poly_k);

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

      bool is_inside_h;
      const auto adjusted_pos_error_h =
          DoCoerceGoal(pos_poly, LH, wh, drive_error, &is_inside_h);
      const auto adjusted_pos_error_45 =
          DoCoerceGoal(pos_poly, L45, w45, intersection, nullptr);
      if (pos_poly.IsInside(intersection)) {
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

    LOG_MATRIX(DEBUG, "adjusted_pos_error", adjusted_pos_error);
    mutable_U() =
        velocity_K * velocity_error + position_K * T * adjusted_pos_error;
    LOG_MATRIX(DEBUG, "U is now", U());
  } else {
    output_was_capped_ = false;
  }
}

DrivetrainMotorsSS::DrivetrainMotorsSS()
    : loop_(new LimitedDrivetrainLoop(
          constants::GetValues().make_drivetrain_loop())),
      filtered_offset_(0.0),
      gyro_(0.0),
      left_goal_(0.0),
      right_goal_(0.0),
      raw_left_(0.0),
      raw_right_(0.0) {
  // High gear on both.
  loop_->set_controller_index(3);
}

void DrivetrainMotorsSS::SetGoal(double left, double left_velocity,
                                 double right, double right_velocity) {
  left_goal_ = left;
  right_goal_ = right;
  loop_->mutable_R() << left, left_velocity, right, right_velocity;
}
void DrivetrainMotorsSS::SetRawPosition(double left, double right) {
  raw_right_ = right;
  raw_left_ = left;
  Eigen::Matrix<double, 2, 1> Y;
  Y << left + filtered_offset_, right - filtered_offset_;
  loop_->Correct(Y);
}
void DrivetrainMotorsSS::SetPosition(double left, double right, double gyro) {
  // Decay the offset quickly because this gyro is great.
  const double offset =
      (right - left - gyro * constants::GetValues().turn_width) / 2.0;
  filtered_offset_ = 0.25 * offset + 0.75 * filtered_offset_;
  gyro_ = gyro;
  SetRawPosition(left, right);
}

void DrivetrainMotorsSS::SetExternalMotors(double left_voltage,
                                           double right_voltage) {
  loop_->mutable_U() << left_voltage, right_voltage;
}

void DrivetrainMotorsSS::Update(bool stop_motors, bool enable_control_loop) {
  if (enable_control_loop) {
    loop_->Update(stop_motors);
  } else {
    if (stop_motors) {
      loop_->mutable_U().setZero();
      loop_->mutable_U_uncapped().setZero();
    }
    loop_->UpdateObserver(loop_->U());
  }
  ::Eigen::Matrix<double, 4, 1> E = loop_->R() - loop_->X_hat();
  LOG_MATRIX(DEBUG, "E", E);
}

double DrivetrainMotorsSS::GetEstimatedRobotSpeed() const {
  // lets just call the average of left and right velocities close enough
  return (loop_->X_hat(1, 0) + loop_->X_hat(3, 0)) / 2;
}

void DrivetrainMotorsSS::SendMotors(
    ::y2014::control_loops::DrivetrainQueue::Output *output) const {
  if (output) {
    output->left_voltage = loop_->U(0, 0);
    output->right_voltage = loop_->U(1, 0);
    output->left_high = true;
    output->right_high = true;
  }
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2014
