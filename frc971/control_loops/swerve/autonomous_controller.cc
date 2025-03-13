#include "autonomous_controller.h"

#include "aos/json_to_flatbuffer.h"
#include "aos/util/math.h"
#include "frc971/control_loops/swerve/simplified_dynamics.h"
#include "frc971/control_loops/swerve/swerve_trajectory_static.h"
#include "frc971/input/robot_state_generated.h"
#include "frc971/math/flatbuffers_matrix.h"

ABSL_FLAG(double, kPositionGain, 7.0, "Proportional gain for positional error");
ABSL_FLAG(double, kRotationGain, 6.0, "Proportional gain for rotational error");
ABSL_FLAG(double, kNextPositionGain, 2.0,
          "Proportional gain for positional error");
ABSL_FLAG(double, kNextRotationGain, 0.0,
          "Proportional gain for rotational error");
ABSL_FLAG(double, kVelocityGain, 0.1, "Proportional gain for velocity error");
ABSL_FLAG(double, kNextVelocityGain, 1.0,
          "Proportional gain for velocity error");
ABSL_FLAG(double, kNextOmegaGain, 0.0, "Proportional gain for velocity error");
ABSL_FLAG(double, kOmegaGain, 1.0,
          "Proportional gain for angular velocity error");

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
      autonomous_controller_debug_sender_(
          event_loop->MakeSender<
              frc971::control_loops::swerve::AutonomousControllerDebugStatic>(
              "/autonomous")),
      joystick_state_fetcher_(
          event_loop->MakeFetcher<aos::JoystickState>("/roborio/aos")),
      localizer_state_fetcher_(
          event_loop
              ->MakeFetcher<frc971::control_loops::swerve::LocalizerState>(
                  "/localizer")),
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
      builder->set_y(flip_mult * trajectory_.message()
                                     .discretized_trajectory()
                                     ->Get(0)
                                     ->position()
                                     ->y());
      const double theta_trajectory = trajectory_.message()
                                          .discretized_trajectory()
                                          ->Get(0)
                                          ->position()
                                          ->theta();
      builder->set_theta((flipped_ ? std::numbers::pi : 0) + theta_trajectory);
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
  auto next_trajectory_point =
      (trajectory_index_.value() + 5 ==
       trajectory_.message().discretized_trajectory()->size())
          ? trajectory_point
          : trajectory_.message().discretized_trajectory()->Get(
                trajectory_index_.value() + 5);

  localizer_state_fetcher_.Fetch();

  const double x = localizer_state_fetcher_->x();
  const double y = localizer_state_fetcher_->y();
  const double theta = localizer_state_fetcher_->theta();

  if (trajectory_index_.value() == 0) {
    prev_x_ = x;
    prev_y_ = y;
    prev_theta_ = theta;
  }

  constexpr double dt =
      aos::time::DurationInSeconds(std::chrono::milliseconds(2));
  const double vx = localizer_state_fetcher_->has_vx()
                        ? localizer_state_fetcher_->vx()
                        : (x - prev_x_) / dt;
  const double vy = localizer_state_fetcher_->has_vy()
                        ? localizer_state_fetcher_->vy()
                        : (y - prev_y_) / dt;
  const double omega =
      localizer_state_fetcher_->has_omega()
          ? localizer_state_fetcher_->omega()
          : aos::math::NormalizeAngle(theta - prev_theta_) / dt;

  const double flip_mult = flipped_ ? -1.0 : 1.0;

  const double goal_x = flip_mult * trajectory_point->position()->x();
  const double goal_y = flip_mult * trajectory_point->position()->y();
  const double goal_theta =
      flipped_ ? std::numbers::pi + trajectory_point->position()->theta()
               : trajectory_point->position()->theta();

  const double goal_vx = flip_mult * trajectory_point->velocity()->x();
  const double goal_vy = flip_mult * trajectory_point->velocity()->y();
  const double goal_omega = trajectory_point->velocity()->theta();

  const double next_goal_x = flip_mult * next_trajectory_point->position()->x();
  const double next_goal_y = flip_mult * next_trajectory_point->position()->y();
  const double next_goal_theta =
      flip_mult * next_trajectory_point->position()->theta();

  const double next_goal_vx =
      flip_mult * next_trajectory_point->velocity()->x();
  const double next_goal_vy =
      flip_mult * next_trajectory_point->velocity()->y();
  const double next_goal_omega =
      flip_mult * next_trajectory_point->velocity()->theta();

  double x_error = x - goal_x;
  double y_error = y - goal_y;
  double theta_error = aos::math::NormalizeAngle(theta - goal_theta);

  double vx_error = vx - goal_vx;
  double vy_error = vy - goal_vy;
  double omega_error = omega - goal_omega;

  double next_x_error = x - next_goal_x;
  double next_y_error = y - next_goal_y;
  double next_theta_error = theta - next_goal_theta;

  double next_vx_error = vx - next_goal_vx;
  double next_vy_error = vy - next_goal_vy;
  double next_omega_error = omega - next_goal_omega;

  prev_x_ = x;
  prev_y_ = y;
  prev_theta_ = theta;

  VLOG(1) << "x error: " << x_error;
  VLOG(1) << "y error: " << y_error;
  VLOG(1) << "theta error: " << theta_error;
  VLOG(1) << "vx error: " << vx_error;
  VLOG(1) << "vy error: " << vy_error;
  VLOG(1) << "omega error: " << omega_error;

  double vx_commanded = goal_vx - x_error * absl::GetFlag(FLAGS_kPositionGain) -
                        vx_error * absl::GetFlag(FLAGS_kVelocityGain) -
                        next_vx_error * absl::GetFlag(FLAGS_kNextVelocityGain) -
                        next_x_error * absl::GetFlag(FLAGS_kNextPositionGain);
  double vy_commanded = goal_vy - y_error * absl::GetFlag(FLAGS_kPositionGain) -
                        vy_error * absl::GetFlag(FLAGS_kVelocityGain) -
                        next_vy_error * absl::GetFlag(FLAGS_kNextVelocityGain) -
                        next_y_error * absl::GetFlag(FLAGS_kNextPositionGain);
  double omega_commanded =
      goal_omega - theta_error * absl::GetFlag(FLAGS_kRotationGain) -
      omega_error * absl::GetFlag(FLAGS_kOmegaGain) -
      next_omega_error * absl::GetFlag(FLAGS_kNextOmegaGain) -
      next_theta_error * absl::GetFlag(FLAGS_kNextRotationGain);

  auto joystick_goal = builder->add_joystick_goal();
  joystick_goal->set_vx(vx_commanded);
  joystick_goal->set_vy(vy_commanded);
  joystick_goal->set_omega(omega_commanded);

  auto time = trajectory_point->time();

  for (size_t i = 0; i < actions_.size(); i++) {
    auto &action = actions_.at(i);
    if (!action.completed && time >= action.time) {
      (*action.callback)();
      action.completed = true;
    }
  }

  builder.CheckOk(builder.Send());

  auto debug_builder = autonomous_controller_debug_sender_.MakeStaticBuilder();
  debug_builder->set_x_error(x_error);
  debug_builder->set_y_error(y_error);
  debug_builder->set_theta_error(theta_error);
  debug_builder->set_vx_error(vx_error);
  debug_builder->set_goal_vx(goal_vx);
  debug_builder->set_vy_error(vy_error);
  debug_builder->set_goal_vy(goal_vy);
  debug_builder->set_omega_error(omega_error);
  debug_builder->set_goal_omega(goal_omega);
  debug_builder.CheckOk(debug_builder.Send());
};

bool AutonomousController::Completed() { return completed_; }

void AutonomousController::AddCallback(std::function<void()> callback,
                                       std::chrono::milliseconds delay) {
  double time = trajectory_.message()
                    .discretized_trajectory()
                    ->Get(trajectory_index_.value())
                    ->time();
  actions_.push_back(
      Action{.completed = false,
             .time = time + delay.count() * 0.001,
             .callback = std::make_shared<std::function<void()>>(callback)});
};
