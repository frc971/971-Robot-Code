#include "autonomous_controller.h"

#include "aos/json_to_flatbuffer.h"
#include "frc971/control_loops/swerve/simplified_dynamics.h"
#include "frc971/control_loops/swerve/swerve_trajectory_static.h"
#include "frc971/math/flatbuffers_matrix.h"

// We are using a PID controller with only the P (proportional) part
ABSL_FLAG(double, kVxProportionalGain, 100.0,
          "Proportional gain for x velocity error");
ABSL_FLAG(double, kVyProportionalGain, 100.0,
          "Proportional gain for y velocity error");
ABSL_FLAG(double, kOmegaProportionalGain, 100.0,
          "Proportional gain for omega velocity error");
ABSL_FLAG(bool, kUseEkfState, false,
          "true if using the EKF's predicted state; "
          "false if using the naive estimator's predicted state");

using frc971::control_loops::swerve::AutonomousController;

AutonomousController::AutonomousController(
    aos::EventLoop *event_loop, std::string_view trajectory_path,
    const std::unordered_map<std::string_view, std::function<void()>>
        &callbacks)
    : trajectory_index_(std::nullopt),
      trajectory_(aos::FileToFlatbuffer<
                  frc971::control_loops::swerve::SwerveTrajectory>(
          trajectory_path)),
      swerve_goal_sender_(
          event_loop->MakeSender<frc971::control_loops::swerve::GoalStatic>(
              "/drivetrain")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/aos")),
      swerve_drivetrain_status_fetcher_(
          event_loop->MakeFetcher<frc971::control_loops::swerve::Status>(
              "/swerve")),
      event_loop_(event_loop) {
  // setup callbacks
  for (auto action : *(trajectory_.message().timestamped_actions())) {
    if (callbacks.contains(action->name()->c_str())) {
      // get callback
      auto callback = std::make_shared<std::function<void()>>(
          callbacks.at(action->name()->c_str()));
      actions_.emplace_back(Action{false, action->time(), callback});
    } else {
      LOG(FATAL) << "Trajectory action '" << action->name()->c_str()
                 << "' not found in callbacks.";
    }
  }

  const auto timer = event_loop_->AddTimer([this]() {
    if (trajectory_index_) {
      size_t index = trajectory_index_.value();

      Iterate();

      if (index >=
          (trajectory_.message().discretized_trajectory()->size() - 1)) {
        trajectory_index_ = std::nullopt;
        completed_ = true;
      } else {
        trajectory_index_ = index + 1;
      }

      if (joystick_state_fetcher_.Fetch()) {
        auto joystick_state = joystick_state_fetcher_.get();
        if (!(joystick_state->autonomous() && joystick_state->enabled())) {
          trajectory_index_ = std::nullopt;
          completed_ = true;
        }
      }
    } else if (!completed_) {
      if (joystick_state_fetcher_.Fetch()) {
        auto joystick_state = joystick_state_fetcher_.get();
        if (joystick_state->autonomous() && joystick_state->enabled()) {
          trajectory_index_ = 0;
        }
      }
    } else {
      if (joystick_state_fetcher_.Fetch()) {
        auto joystick_state = joystick_state_fetcher_.get();
        if (!joystick_state->autonomous()) {
          completed_ = false;
        }
      }
    }
  });
  event_loop->OnRun([timer, event_loop]() {
    timer->Schedule(event_loop->monotonic_now(), std::chrono::milliseconds(5));
  });
};

void AutonomousController::Iterate() {
  auto builder = swerve_goal_sender_.MakeStaticBuilder();
  auto trajectory_point = trajectory_.message().discretized_trajectory()->Get(
      trajectory_index_.value());

  constexpr size_t num_velocity_states =
      frc971::control_loops::swerve::SimplifiedDynamics<
          double>::States::kNumVelocityStates;
  Eigen::Matrix<double, num_velocity_states, 1> x_hat;

  swerve_drivetrain_status_fetcher_.Fetch();
  if (absl::GetFlag(FLAGS_kUseEkfState)) {
    x_hat = ToEigenOrDie<num_velocity_states, 1>(
        *swerve_drivetrain_status_fetcher_.get()->velocity_ekf()->x_hat());
  } else {
    x_hat = ToEigenOrDie<num_velocity_states, 1>(
        *swerve_drivetrain_status_fetcher_.get()
             ->naive_estimator()
             ->velocity_state());
  }

  double vx = x_hat(
      frc971::control_loops::swerve::SimplifiedDynamics<double>::States::kVx);
  double vy = x_hat(
      frc971::control_loops::swerve::SimplifiedDynamics<double>::States::kVy);
  double omega = x_hat(frc971::control_loops::swerve::SimplifiedDynamics<
                       double>::States::kOmega);

  double vx_error = vx - trajectory_point->velocity()->x();
  double vy_error = vy - trajectory_point->velocity()->y();
  double omega_error = omega - trajectory_point->velocity()->theta();

  VLOG(1) << "vx error: " << vx_error;
  VLOG(1) << "vy error: " << vy_error;
  VLOG(1) << "omega error: " << omega_error;

  double goal_vx = trajectory_point->velocity()->x() +
                   absl::GetFlag(FLAGS_kVxProportionalGain) * vx_error;
  double goal_vy = trajectory_point->velocity()->y() +
                   absl::GetFlag(FLAGS_kVyProportionalGain) * vy_error;
  double goal_omega = trajectory_point->velocity()->theta() +
                      absl::GetFlag(FLAGS_kOmegaProportionalGain) * omega_error;

  auto joystick_goal = builder->add_joystick_goal();
  joystick_goal->set_vx(goal_vx);
  joystick_goal->set_vy(goal_vy);
  joystick_goal->set_omega(goal_omega);

  auto time = trajectory_point->time();

  for (auto action : actions_) {
    if (!action.completed && time >= action.time) {
      (*action.callback)();
      action.completed = true;
    }
  }

  builder.CheckOk(builder.Send());
};

bool AutonomousController::Completed() { return completed_; }
