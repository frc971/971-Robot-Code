#include <stddef.h>
#include <inttypes.h>

#include "aos/logging/logging.h"
#include "aos/logging/queue_logging.h"
#include "aos/robot_state/robot_state.q.h"

namespace aos {
namespace controls {

// TODO(aschuh): Tests.

template <class T>
constexpr ::std::chrono::milliseconds ControlLoop<T>::kStaleLogInterval;
template <class T>
constexpr ::std::chrono::milliseconds ControlLoop<T>::kPwmDisableTime;

template <class T>
void ControlLoop<T>::ZeroOutputs() {
  typename ::aos::Sender<OutputType>::Message output =
      output_sender_.MakeMessage();
  Zero(output.get());
  output.Send();
}

template <class T>
void ControlLoop<T>::Iterate() {
  control_loop_->position.FetchAnother();
  IteratePosition(*control_loop_->position.get());
}

template <class T>
void ControlLoop<T>::IteratePosition(const PositionType &position) {
  // Since Exit() isn't async safe, we want to call Exit from the periodic
  // handler.
  if (!run_) {
    event_loop_->Exit();
  }
  no_goal_.Print();
  no_sensor_state_.Print();
  motors_off_log_.Print();

  LOG_STRUCT(DEBUG, "position", position);

  // Fetch the latest control loop goal. If there is no new
  // goal, we will just reuse the old one.
  goal_fetcher_.Fetch();
  const GoalType *goal = goal_fetcher_.get();
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

  typename ::aos::Sender<StatusType>::Message status =
      status_sender_.MakeMessage();
  if (status.get() == nullptr) {
    return;
  }

  if (outputs_enabled) {
    typename ::aos::Sender<OutputType>::Message output =
        output_sender_.MakeMessage();
    RunIteration(goal, &position, output.get(), status.get());

    output->SetTimeToNow();
    LOG_STRUCT(DEBUG, "output", *output);
    output.Send();
  } else {
    // The outputs are disabled, so pass nullptr in for the output.
    RunIteration(goal, &position, nullptr, status.get());
    ZeroOutputs();
  }

  status->SetTimeToNow();
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

  event_loop_->MakeWatcher(::std::string(control_loop_->name()) + ".position",
                           [this](const PositionType &position) {
                             this->IteratePosition(position);
                           });

  event_loop_->Run();
  LOG(INFO, "Shutting down\n");
}

template <class T>
::std::atomic<bool> ControlLoop<T>::run_{true};

}  // namespace controls
}  // namespace aos
