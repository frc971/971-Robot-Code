#include "y2025/localizer/localizer.h"

ABSL_FLAG(double, q_pos, 0.1,
          "Q for position states in the kalman (X, Y, Theta)");
ABSL_FLAG(double, q_vel, 0.1,
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
  (void)distance_to_robot;
  (void)target_id;

  Eigen::Matrix<double, 3, kNumStates> H;
  H.setZero();

  H(0, StateIdx::kX) = 1.0;
  H(1, StateIdx::kY) = 1.0;
  H(2, StateIdx::kTheta) = 1.0;

  Eigen::Matrix<double, 3, 1> z;

  z << pose(PoseIdx::kX), pose(PoseIdx::kY), pose(PoseIdx::kTheta);

  Eigen::Matrix<double, 3, 3> noise = Eigen::Matrix<double, 3, 3>::Identity();

  noise *= distortion_factor;

  kalman_filter_.Correct(z, H, noise, now);
}

void KalmanFilterLocalizer::HandleEstimatedSwerveState(
    EstimatedSwerveState estimated_state,
    aos::monotonic_clock::time_point now) {
  Eigen::Matrix<double, 6, KalmanFilterLocalizer::kNumStates> H;
  H.setZero();
  H(0, StateIdx::kX) = 1.0;
  H(0, StateIdx::kY) = 1.0;
  H(0, StateIdx::kTheta) = 1.0;
  H(0, StateIdx::kVx) = 1.0;
  H(0, StateIdx::kVy) = 1.0;
  H(0, StateIdx::kOmega) = 1.0;

  State z;
  z << estimated_state.x, estimated_state.y, estimated_state.theta,
      estimated_state.vx, estimated_state.vy, estimated_state.omega;

  // This should be fine because it already goes through basic filtering before
  // we get it.
  StateSquare R = StateSquare::Identity();

  R *= 1e-6;
  kalman_filter_.Correct(z, H, R, now);
}

void KalmanFilterLocalizer::HandleAutonomousInit(
    const frc971::control_loops::swerve::AutonomousInit &init_message) {
  State initial_x_hat = State::Zero();

  initial_x_hat(StateIdx::kX) = init_message.x();
  initial_x_hat(StateIdx::kY) = init_message.y();
  initial_x_hat(StateIdx::kTheta) = init_message.theta();

  kalman_filter_.Reset(event_loop_->context().monotonic_event_time,
                       StateSquare::Zero(), initial_x_hat);
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
