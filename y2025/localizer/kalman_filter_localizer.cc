#include "y2025/localizer/localizer.h"

ABSL_FLAG(double, q_pos, 1.0,
          "Q for position states in the kalman (X, Y, Theta)");
ABSL_FLAG(double, q_vel, 1.0,
          "Q for velocity states in the kalman (V_x, V_y, Omega)");

namespace y2025::localizer {
namespace {
using State = KalmanFilterLocalizer::KalmanFilter::State;
using StateSquare = KalmanFilterLocalizer::KalmanFilter::StateSquare;

using PoseIdx = Localizer::PoseIdx;
using StateIdx = KalmanFilterLocalizer::StateIdx;

StateSquare Q() {
  StateSquare Q = StateSquare::Identity();
  Q.diagonal()(StateIdx::kX) = absl::GetFlag(FLAGS_q_pos);
  Q.diagonal()(StateIdx::kY) = absl::GetFlag(FLAGS_q_pos);
  Q.diagonal()(StateIdx::kTheta) = absl::GetFlag(FLAGS_q_pos);
  Q.diagonal()(StateIdx::kVx) = absl::GetFlag(FLAGS_q_vel);
  Q.diagonal()(StateIdx::kVy) = absl::GetFlag(FLAGS_q_vel);
  Q.diagonal()(StateIdx::kOmega) = absl::GetFlag(FLAGS_q_vel);

  return Q;
}

StateSquare A() {
  StateSquare A = StateSquare::Zero();
  A(StateIdx::kX, StateIdx::kVx) = 1.0;
  A(StateIdx::kY, StateIdx::kVy) = 1.0;
  A(StateIdx::kTheta, StateIdx::kOmega) = 1.0;

  return A;
}

// TOOD(max): Initialize P to the steady state covariance.
StateSquare P() {
  StateSquare P = StateSquare::Identity();
  P *= 1e-6;

  return P;
}

}  // namespace
KalmanFilterLocalizer::KalmanFilterLocalizer(aos::EventLoop *event_loop)
    : Localizer(event_loop),
      kalman_filter_(Q(), A(), event_loop->context().monotonic_event_time, P(),
                     State::Zero()) {}

void KalmanFilterLocalizer::HandleDetectedRobotPose(
    KalmanFilterLocalizer::XYThetaPose pose, double distance_to_robot,
    uint64_t target_id, double distortion_factor,
    aos::monotonic_clock::time_point capture_time,
    aos::monotonic_clock::time_point now) {
  (void)capture_time;
  (void)target_id;

  Eigen::Matrix<double, 3, kNumStates> H;
  H.setZero();

  H(0, StateIdx::kX) = 1.0;
  H(1, StateIdx::kY) = 1.0;
  H(2, StateIdx::kTheta) = 1.0;

  Eigen::Matrix<double, 3, 1> z;
  State x_hat = kalman_filter_.X_hat();
  double robot_theta = x_hat(StateIdx::kTheta);
  double closest_theta =
      aos::math::NormalizeAngle(pose(PoseIdx::kTheta) - robot_theta) +
      robot_theta;

  z << pose(PoseIdx::kX), pose(PoseIdx::kY), closest_theta;

  Eigen::Matrix<double, 3, 3> noise = Eigen::Matrix<double, 3, 3>::Identity();

  noise *= (1.0 + distortion_factor) * (distance_to_robot * distance_to_robot);

  kalman_filter_.Correct(z, H, noise, now);

  (*kalman_filter_.X_hat_mut())(StateIdx::kTheta) =
      aos::math::NormalizeAngle(kalman_filter_.X_hat()(StateIdx::kTheta));
}

void KalmanFilterLocalizer::HandleEstimatedSwerveState(
    EstimatedSwerveState estimated_state,
    aos::monotonic_clock::time_point now) {
  Eigen::Matrix<double, 3, KalmanFilterLocalizer::kNumStates> H;

  H.setZero();

  H(0, StateIdx::kVx) = 1.0;
  H(1, StateIdx::kVy) = 1.0;
  H(2, StateIdx::kOmega) = 1.0;

  Eigen::Matrix<double, 3, 1> z;
  z << estimated_state.vx, estimated_state.vy, estimated_state.omega;

  // This should be fine because it already goes through basic filtering before
  // we get it.
  Eigen::Matrix<double, 3, 3> noise = Eigen::Matrix<double, 3, 3>::Identity();

  kalman_filter_.Correct(z, H, noise, now);

  (*kalman_filter_.X_hat_mut())(StateIdx::kTheta) =
      aos::math::NormalizeAngle(kalman_filter_.X_hat()(StateIdx::kTheta));
}

void KalmanFilterLocalizer::HandleAutonomousInit(
    const frc971::control_loops::swerve::AutonomousInit &init_message) {
  (void)init_message;
}

void KalmanFilterLocalizer::SendOutput(
    aos::Sender<Localizer::LocalizerStateStatic>::StaticBuilder state_builder) {
  State x_hat = kalman_filter_.X_hat();

  state_builder->set_x(x_hat(StateIdx::kX));
  state_builder->set_y(x_hat(StateIdx::kY));
  state_builder->set_theta(x_hat(StateIdx::kTheta));

  state_builder->set_vx(x_hat(StateIdx::kVx));
  state_builder->set_vy(x_hat(StateIdx::kVy));
  state_builder->set_omega(x_hat(StateIdx::kOmega));

  state_builder.CheckOk(state_builder.Send());
}

}  // namespace y2025::localizer
