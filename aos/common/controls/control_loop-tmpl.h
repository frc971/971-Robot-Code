#include <stddef.h>
#include <inttypes.h>

#include "aos/common/logging/logging.h"
#include "aos/common/messages/robot_state.q.h"
#include "aos/common/logging/queue_logging.h"

namespace aos {
namespace controls {

// TODO(aschuh): Tests.

template <class T>
constexpr ::std::chrono::milliseconds ControlLoop<T>::kStaleLogInterval;
template <class T>
constexpr ::std::chrono::milliseconds ControlLoop<T>::kPwmDisableTime;

template <class T>
void ControlLoop<T>::ZeroOutputs() {
  aos::ScopedMessagePtr<OutputType> output =
      control_loop_->output.MakeMessage();
  Zero(output.get());
  output.Send();
}

template <class T>
void ControlLoop<T>::Iterate() {
  no_goal_.Print();
  no_sensor_state_.Print();
  motors_off_log_.Print();

  control_loop_->position.FetchAnother();
  const PositionType *const position = control_loop_->position.get();
  LOG_STRUCT(DEBUG, "position", *position);

  // Fetch the latest control loop goal. If there is no new
  // goal, we will just reuse the old one.
  control_loop_->goal.FetchLatest();
  const GoalType *goal = control_loop_->goal.get();
  if (goal) {
    LOG_STRUCT(DEBUG, "goal", *goal);
  } else {
    LOG_INTERVAL(no_goal_);
  }

  const bool new_robot_state = ::aos::robot_state.FetchLatest();
  if (!::aos::robot_state.get()) {
    LOG_INTERVAL(no_sensor_state_);
    return;
  }
  if (sensor_reader_pid_ != ::aos::robot_state->reader_pid) {
    LOG(INFO, "new sensor reader PID %" PRId32 ", old was %" PRId32 "\n",
        ::aos::robot_state->reader_pid, sensor_reader_pid_);
    reset_ = true;
    sensor_reader_pid_ = ::aos::robot_state->reader_pid;
  }

  bool outputs_enabled = ::aos::robot_state->outputs_enabled;

  // Check to see if we got a driver station packet recently.
  if (new_robot_state) {
    if (::aos::robot_state->outputs_enabled) {
      // If the driver's station reports being disabled, we're probably not
      // actually going to send motor values regardless of what the FPGA
      // reports.
      last_pwm_sent_ = ::aos::robot_state->sent_time;
    }
  }

  const ::aos::monotonic_clock::time_point now = ::aos::monotonic_clock::now();
  const bool motors_off = now >= kPwmDisableTime + last_pwm_sent_;
  ::aos::joystick_state.FetchLatest();
  if (motors_off) {
    if (::aos::joystick_state.get() && ::aos::joystick_state->enabled) {
      LOG_INTERVAL(motors_off_log_);
    }
    outputs_enabled = false;
  }

  aos::ScopedMessagePtr<StatusType> status =
      control_loop_->status.MakeMessage();
  if (status.get() == nullptr) {
    return;
  }

  if (outputs_enabled) {
    aos::ScopedMessagePtr<OutputType> output =
        control_loop_->output.MakeMessage();
    RunIteration(goal, position, output.get(), status.get());

    LOG_STRUCT(DEBUG, "output", *output);
    output.Send();
  } else {
    // The outputs are disabled, so pass nullptr in for the output.
    RunIteration(goal, position, nullptr, status.get());
    ZeroOutputs();
  }

  LOG_STRUCT(DEBUG, "status", *status);
  status.Send();
}

template <class T>
void ControlLoop<T>::Run() {
  struct sigaction action;
  action.sa_handler = &ControlLoop<T>::Quit;
  sigemptyset(&action.sa_mask);
  action.sa_flags = SA_RESETHAND;

  PCHECK(sigaction(SIGTERM, &action, nullptr));
  PCHECK(sigaction(SIGQUIT, &action, nullptr));
  PCHECK(sigaction(SIGINT, &action, nullptr));

  while (run_) {
    Iterate();
  }
  LOG(INFO, "Shutting down\n");
}

template <class T>
::std::atomic<bool> ControlLoop<T>::run_{true};

}  // namespace controls
}  // namespace aos
