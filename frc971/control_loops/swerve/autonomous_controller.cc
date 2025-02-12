#include "autonomous_controller.h"

#include "aos/json_to_flatbuffer.h"
#include "frc971/control_loops/swerve/simplified_dynamics.h"
#include "frc971/control_loops/swerve/swerve_trajectory_static.h"
#include "frc971/input/robot_state_generated.h"
#include "frc971/math/flatbuffers_matrix.h"

// We are using a PID controller with only the P (proportional) part
ABSL_FLAG(double, kPositionGain, 0.5, "Proportional gain for positional error");
ABSL_FLAG(double, kRotationGain, 0.5, "Proportional gain for rotational error");

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
              "/autonomous")),
      autonomous_init_sender_(
          event_loop
              ->MakeSender<frc971::control_loops::swerve::AutonomousInitStatic>(
                  "/drivetrain")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/roborio/aos")),
      swerve_drivetrain_status_fetcher_(
          event_loop->MakeFetcher<frc971::control_loops::swerve::Status>(
              "/drivetrain")),
      event_loop_(event_loop) {
  completed_ = false;
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

  event_loop_->MakeWatcher("/aos", [this](const aos::RobotState &state) {
    if (state.user_button()) {
      auto builder = autonomous_init_sender_.MakeStaticBuilder();
      const double flip_mult = flipped_ ? -1.0 : 1.0;
      builder->set_x(flip_mult * trajectory_.message()
                                     .discretized_trajectory()
                                     ->Get(0)
                                     ->position()
                                     ->x());
      builder->set_y(trajectory_.message()
                         .discretized_trajectory()
                         ->Get(0)
                         ->position()
                         ->y());
      const double theta_trajectory = trajectory_.message()
                                          .discretized_trajectory()
                                          ->Get(0)
                                          ->position()
                                          ->theta();
      builder->set_theta(flipped_ ? std::numbers::pi - theta_trajectory
                                  : theta_trajectory);
      builder.CheckOk(builder.Send());
    }
  });

  const auto timer = event_loop_->AddTimer([this]() {
    joystick_state_fetcher_.Fetch();
    if (joystick_state_fetcher_.get() != nullptr) {
      VLOG(1) << "Autonomous " << joystick_state_fetcher_->autonomous();
      VLOG(1) << "Enabled " << joystick_state_fetcher_->enabled();
    }

    if (joystick_state_fetcher_->alliance() == aos::Alliance::kRed) {
      flipped_ = true;
    } else if (joystick_state_fetcher_->alliance() == aos::Alliance::kBlue) {
      flipped_ = false;
    }

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
      joystick_state_fetcher_.Fetch();
      if (joystick_state_fetcher_.get() != nullptr) {
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

  constexpr size_t num_position_states =
      frc971::control_loops::swerve::SimplifiedDynamics<
          double>::States::kNumPositionStates;
  Eigen::Matrix<double, num_position_states, 1> x_hat;

  swerve_drivetrain_status_fetcher_.Fetch();
  x_hat = ToEigenOrDie<num_position_states, 1>(
      *swerve_drivetrain_status_fetcher_.get()
           ->naive_estimator()
           ->position_state());

  double x = x_hat(
      frc971::control_loops::swerve::SimplifiedDynamics<double>::States::kX);
  double y = x_hat(
      frc971::control_loops::swerve::SimplifiedDynamics<double>::States::kY);
  double theta = x_hat(frc971::control_loops::swerve::SimplifiedDynamics<
                       double>::States::kTheta);

  double kThreshold = 0.005;

  const double flip_mult = flipped_ ? -1.0 : 1.0;
  double x_error = x - flip_mult * trajectory_point->position()->x();
  double y_error = y - trajectory_point->position()->y();
  const double theta_trajectory = trajectory_point->position()->theta();
  double theta_error = theta - (flipped_ ? std::numbers::pi - theta_trajectory
                                         : theta_trajectory);

  if (std::abs(x_error) < kThreshold) {
    x_error = 0.0;
  }
  if (std::abs(y_error) < kThreshold) {
    y_error = 0.0;
  }
  if (std::abs(theta_error) < kThreshold) {
    theta_error = 0.0;
  }

  VLOG(1) << "x error: " << x_error;
  VLOG(1) << "y error: " << y_error;
  VLOG(1) << "theta error: " << theta_error;

  double goal_vx = flip_mult * trajectory_point->velocity()->x() -
                   absl::GetFlag(FLAGS_kPositionGain) * x_error;
  double goal_vy = trajectory_point->velocity()->y() -
                   absl::GetFlag(FLAGS_kPositionGain) * y_error;
  double goal_omega = flip_mult * trajectory_point->velocity()->theta() -
                      absl::GetFlag(FLAGS_kRotationGain) * theta_error;

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
