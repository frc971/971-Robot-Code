#include <stddef.h>
#include <inttypes.h>

#include "aos/logging/logging.h"

namespace aos {
namespace controls {

// TODO(aschuh): Tests.

template <class GoalType, class PositionType, class StatusType,
          class OutputType>
constexpr ::std::chrono::milliseconds ControlLoop<
    GoalType, PositionType, StatusType, OutputType>::kStaleLogInterval;
template <class GoalType, class PositionType, class StatusType,
          class OutputType>
constexpr ::std::chrono::milliseconds ControlLoop<
    GoalType, PositionType, StatusType, OutputType>::kPwmDisableTime;

template <class GoalType, class PositionType, class StatusType,
          class OutputType>
void ControlLoop<GoalType, PositionType, StatusType, OutputType>::ZeroOutputs() {
  typename ::aos::Sender<OutputType>::Builder builder =
      output_sender_.MakeBuilder();
  builder.Send(Zero(&builder));
}

template <class GoalType, class PositionType, class StatusType,
          class OutputType>
void ControlLoop<GoalType, PositionType, StatusType,
                 OutputType>::IteratePosition(const PositionType &position) {
  no_goal_.Print();
  no_sensor_state_.Print();
  motors_off_log_.Print();

  // Fetch the latest control loop goal. If there is no new
  // goal, we will just reuse the old one.
  goal_fetcher_.Fetch();
  const GoalType *goal = goal_fetcher_.get();

  const bool new_robot_state = robot_state_fetcher_.Fetch();
  if (!robot_state_fetcher_.get()) {
    AOS_LOG_INTERVAL(no_sensor_state_);
    return;
  }
  if (sensor_reader_pid_ != robot_state_fetcher_->reader_pid()) {
    AOS_LOG(INFO, "new sensor reader PID %" PRId32 ", old was %" PRId32 "\n",
            robot_state_fetcher_->reader_pid(), sensor_reader_pid_);
    reset_ = true;
    sensor_reader_pid_ = robot_state_fetcher_->reader_pid();
  }

  bool outputs_enabled = robot_state_fetcher_->outputs_enabled();

  // Check to see if we got a driver station packet recently.
  if (new_robot_state) {
    if (robot_state_fetcher_->outputs_enabled()) {
      // If the driver's station reports being disabled, we're probably not
      // actually going to send motor values regardless of what the FPGA
      // reports.
      last_pwm_sent_ = robot_state_fetcher_.context().monotonic_event_time;
    }
  }

  const ::aos::monotonic_clock::time_point monotonic_now =
      event_loop_->monotonic_now();
  const bool motors_off = monotonic_now >= kPwmDisableTime + last_pwm_sent_;
  joystick_state_fetcher_.Fetch();
  if (motors_off) {
    if (joystick_state_fetcher_.get() && joystick_state_fetcher_->enabled()) {
      AOS_LOG_INTERVAL(motors_off_log_);
    }
    outputs_enabled = false;
  }

  typename ::aos::Sender<StatusType>::Builder status =
      status_sender_.MakeBuilder();
  if (outputs_enabled) {
    typename ::aos::Sender<OutputType>::Builder output =
        output_sender_.MakeBuilder();
    RunIteration(goal, &position, &output, &status);

    output.CheckSent();
  } else {
    // The outputs are disabled, so pass nullptr in for the output.
    RunIteration(goal, &position, nullptr, &status);
    ZeroOutputs();
  }

  status.CheckSent();
}

}  // namespace controls
}  // namespace aos
