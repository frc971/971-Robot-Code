#include "auto_align.h"

using frc971::control_loops::swerve::AutoAlign;

// TODO Move these constants to a json or something
ABSL_FLAG(double, kPVx, 1.0, "Gain for x position error");
ABSL_FLAG(double, kPVy, 1.0, "Gain for y position error");
ABSL_FLAG(double, kPVtheta, 1.0, "Gain for theta position error");

ABSL_FLAG(double, kVelLimit, 1.0, "Velocity Limit");
ABSL_FLAG(double, kOmegaLimit, 1.0, "Omega Limit");

ABSL_FLAG(double, kXOffset, 0.0, "X Offset");
ABSL_FLAG(double, kYOffset, 0.0, "Y Offset");
ABSL_FLAG(double, kThetaOffset, 0.0, "Theta Offset");

constexpr double limit_goal_omega(double goal, double limit) {
  goal = std::min(goal, limit);
  goal = std::max(goal, -limit);
  return goal;
}

AutoAlign::AutoAlign(aos::EventLoop *event_loop)
    : swerve_goal_sender_(
          event_loop->MakeSender<frc971::control_loops::swerve::GoalStatic>(
              "/autonomous")),
      swerve_drivetrain_status_fetcher_(
          event_loop->MakeFetcher<frc971::control_loops::swerve::Status>(
              "/swerve")) {}

void AutoAlign::Iterate() {
  auto builder = swerve_goal_sender_.MakeStaticBuilder();
  swerve_drivetrain_status_fetcher_.Fetch();
  // converts between frc971.fbs.Matrix and Eigen::Matrix
  constexpr size_t num_position_states =
      frc971::control_loops::swerve::SimplifiedDynamics<
          double>::States::kNumPositionStates;
  Eigen::Matrix<double, num_position_states, 1> x_hat =
      ToEigenOrDie<num_position_states, 1>(
          *swerve_drivetrain_status_fetcher_.get()
               ->naive_estimator()
               ->position_state());

  double x = x_hat(
      frc971::control_loops::swerve::SimplifiedDynamics<double>::States::kX);
  double y = x_hat(
      frc971::control_loops::swerve::SimplifiedDynamics<double>::States::kY);
  double theta = x_hat(frc971::control_loops::swerve::SimplifiedDynamics<
                       double>::States::kTheta);

  // TODO: Set goal from fbs?
  const double x_goal = goal_x_;
  const double y_goal = goal_y_;
  const double theta_goal = goal_theta_;

  const double kVelocityLimit = absl::GetFlag(FLAGS_kVelLimit);
  const double kOmegaLimit = absl::GetFlag(FLAGS_kOmegaLimit);

  const double kThreshold = 0.01;

  double x_error = x_goal - x + absl::GetFlag(FLAGS_kXOffset);
  double y_error = y_goal - y + absl::GetFlag(FLAGS_kYOffset);
  // Shortest distance (theta_error should be between -pi, pi)
  double theta_error =
      std::fmod(
          theta_goal - theta + absl::GetFlag(FLAGS_kThetaOffset) + 3 * M_PI,
          (2 * M_PI)) -
      M_PI;

  if (std::abs(x_error) < kThreshold) {
    x_error = 0.0;
  }
  if (std::abs(y_error) < kThreshold) {
    y_error = 0.0;
  }
  if (std::abs(theta_error) < kThreshold) {
    theta_error = 0.0;
  }

  double goal_vx = absl::GetFlag(FLAGS_kPVx) * x_error;
  double goal_vy = absl::GetFlag(FLAGS_kPVy) * y_error;
  double goal_omega = absl::GetFlag(FLAGS_kPVtheta) * theta_error;

  // limit the velocity
  // goal_vx^2 + goal_vy^2 <= magnitude^2
  double magnitude = goal_vx * goal_vx + goal_vy * goal_vy;
  if (magnitude > kVelocityLimit * kVelocityLimit) {
    magnitude = std::sqrt(magnitude);
    goal_vx /= magnitude;
    goal_vy /= magnitude;
    goal_vx *= kVelocityLimit;
    goal_vy *= kVelocityLimit;
  }

  // limit the omega (angular velocity)
  goal_omega = limit_goal_omega(goal_omega, kOmegaLimit);

  // send joystick goal
  auto joystick_goal = builder->add_joystick_goal();
  joystick_goal->set_vx(goal_vx);
  joystick_goal->set_vy(goal_vy);
  joystick_goal->set_omega(goal_omega);

  builder.CheckOk(builder.Send());
}

void AutoAlign::setGoal(double x, double y, double theta) {
  goal_x_ = x;
  goal_y_ = y;
  goal_theta_ = theta;
}
