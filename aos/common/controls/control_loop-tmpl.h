#include <stddef.h>

#include "aos/common/logging/logging.h"
#include "aos/common/messages/robot_state.q.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/util/phased_loop.h"
#include "aos/common/controls/sensor_generation.q.h"
#include "aos/common/controls/output_check.q.h"

namespace aos {
namespace controls {

// TODO(aschuh): Tests.

template <class T, bool has_position, bool fail_no_position, bool fail_no_goal>
constexpr ::aos::time::Time ControlLoop<T, has_position, fail_no_position,
                                        fail_no_goal>::kStaleLogInterval;

template <class T, bool has_position, bool fail_no_position, bool fail_no_goal>
void
ControlLoop<T, has_position, fail_no_position, fail_no_goal>::ZeroOutputs() {
  aos::ScopedMessagePtr<OutputType> output =
      control_loop_->output.MakeMessage();
  Zero(output.get());
  output.Send();
}

template <class T, bool has_position, bool fail_no_position, bool fail_no_goal>
void ControlLoop<T, has_position, fail_no_position, fail_no_goal>::Iterate() {
  no_prior_goal_.Print();
  no_sensor_generation_.Print();
  very_stale_position_.Print();
  no_prior_position_.Print();
  driver_station_old_.Print();
  no_driver_station_.Print();

  // Fetch the latest control loop goal and position.  If there is no new
  // goal, we will just reuse the old one.
  // If there is no goal, we haven't started up fully.  It isn't worth
  // the added complexity for each loop implementation to handle that case.
  control_loop_->goal.FetchLatest();
  // TODO(aschuh): Check the age here if we want the loop to stop on old
  // goals.
  const GoalType *goal = control_loop_->goal.get();
  if (goal == NULL) {
    LOG_INTERVAL(no_prior_goal_);
    if (fail_no_goal) {
      ZeroOutputs();
      return;
    }
  }

  sensor_generation.FetchLatest();
  if (sensor_generation.get() == nullptr) {
    LOG_INTERVAL(no_sensor_generation_);
    ZeroOutputs();
    return;
  }
  if (!has_sensor_reset_counters_ ||
      sensor_generation->reader_pid != reader_pid_ ||
      sensor_generation->cape_resets != cape_resets_) {
    LOG_STRUCT(INFO, "new sensor_generation message",
               *sensor_generation.get());

    reader_pid_ = sensor_generation->reader_pid;
    cape_resets_ = sensor_generation->cape_resets;
    has_sensor_reset_counters_ = true;
    reset_ = true;
  }

  if (goal) {
    LOG_STRUCT(DEBUG, "goal", *goal);
  }

  // Only pass in a position if we got one this cycle.
  const PositionType *position = NULL;

  // Only fetch the latest position if we have one.
  if (has_position) {
    // If the position is stale, this is really bad.  Try fetching a position
    // and check how fresh it is, and then take the appropriate action.
    if (control_loop_->position.FetchLatest()) {
      position = control_loop_->position.get();
    } else {
      if (control_loop_->position.get() && !reset_) {
        int msec_age = control_loop_->position.Age().ToMSec();
        if (!control_loop_->position.IsNewerThanMS(kPositionTimeoutMs)) {
          LOG_INTERVAL(very_stale_position_);
          ZeroOutputs();
          return;
        } else {
          LOG(ERROR, "Stale position. %d ms (< %d ms)\n", msec_age,
              kPositionTimeoutMs);
        }
      } else {
        LOG_INTERVAL(no_prior_position_);
        if (fail_no_position) {
          ZeroOutputs();
          return;
        }
      }
    }
    if (position) {
      LOG_STRUCT(DEBUG, "position", *position);
    }
  }

  bool outputs_enabled = false;

  // Check to see if we got a driver station packet recently.
  if (::aos::robot_state.FetchLatest()) {
    outputs_enabled = true;
  } else if (::aos::robot_state.IsNewerThanMS(kDSPacketTimeoutMs)) {
    outputs_enabled = true;
  } else {
    if (::aos::robot_state.get()) {
      LOG_INTERVAL(driver_station_old_);
    } else {
      LOG_INTERVAL(no_driver_station_);
    }
  }

  ::aos::controls::output_check_received.FetchLatest();
  // True if we're enabled but the motors aren't working.
  // The 100ms is the result of disabling the robot while it's putting out a lot
  // of power and looking at the time delay between the last PWM pulse and the
  // battery voltage coming back up.
  const bool motors_off =
      !::aos::controls::output_check_received.get() ||
      !::aos::controls::output_check_received.IsNewerThanMS(100);
  motors_off_log_.Print();
  if (motors_off) {
    if (!::aos::robot_state.get() || ::aos::robot_state->enabled) {
      LOG_INTERVAL(motors_off_log_);
    }
    outputs_enabled = false;
  }

  // Run the iteration.
  aos::ScopedMessagePtr<StatusType> status =
      control_loop_->status.MakeMessage();
  if (status.get() == NULL) {
    return;
  }

  if (outputs_enabled) {
    aos::ScopedMessagePtr<OutputType> output =
        control_loop_->output.MakeMessage();
    RunIteration(goal, position, output.get(), status.get());

    LOG_STRUCT(DEBUG, "output", *output);
    output.Send();
  } else {
    // The outputs are disabled, so pass NULL in for the output.
    RunIteration(goal, position, nullptr, status.get());
    ZeroOutputs();
  }

  LOG_STRUCT(DEBUG, "status", *status);
  status.Send();
}

template <class T, bool has_position, bool fail_no_position, bool fail_no_goal>
void ControlLoop<T, has_position, fail_no_position, fail_no_goal>::Run() {
  ::aos::time::Time::EnableMockTime();
  while (true) {
    ::aos::time::Time::UpdateMockTime();
    const ::aos::time::Time next_loop = NextLoopTime();
    time::SleepUntil(next_loop);
    ::aos::time::Time::SetMockTime(next_loop);
    Iterate();
  }
}

}  // namespace controls
}  // namespace aos
