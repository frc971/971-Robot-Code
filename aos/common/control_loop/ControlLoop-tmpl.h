#include <stddef.h>

#include "aos/common/logging/logging.h"
#include "aos/common/control_loop/Timing.h"
#include "aos/common/messages/RobotState.q.h"

namespace aos {
namespace control_loops {

// TODO(aschuh): Tests.

template <class T, bool has_position>
void ControlLoop<T, has_position>::ZeroOutputs() {
  aos::ScopedMessagePtr<OutputType> output =
      control_loop_->output.MakeMessage();
  Zero(output.get());
  output.Send();
}

template <class T, bool has_position>
void ControlLoop<T, has_position>::Iterate() {
  // Temporary storage for printing out inputs and outputs.
  char state[1024];

  // Fetch the latest control loop goal and position.  If there is no new
  // goal, we will just reuse the old one.
  // If there is no goal, we haven't started up fully.  It isn't worth
  // the added complexity for each loop implementation to handle that case.
  control_loop_->goal.FetchLatest();
  // TODO(aschuh): Check the age here if we want the loop to stop on old
  // goals.
  const GoalType *goal = control_loop_->goal.get();
  if (goal == NULL) {
    LOG(ERROR, "No prior control loop goal.\n");
    ZeroOutputs();
    return;
  }
  goal->Print(state, sizeof(state));
  LOG(DEBUG, "goal={%s}\n", state);

  // Only pass in a position if we got one this cycle.
  const PositionType *position = NULL;

  // Only fetch the latest position if we have one.
  if (has_position) {
    // If the position is stale, this is really bad.  Try fetching a position
    // and check how fresh it is, and then take the appropriate action.
    if (control_loop_->position.FetchLatest()) {
      position = control_loop_->position.get();
    } else {
      if (control_loop_->position.get()) {
        int msec_age = control_loop_->position.Age().ToMSec();
        if (!control_loop_->position.IsNewerThanMS(kPositionTimeoutMs)) {
          LOG(ERROR, "Stale position. %d ms > %d ms.  Outputs disabled.\n",
              msec_age, kPositionTimeoutMs);
          ZeroOutputs();
          return;
        } else {
          LOG(ERROR, "Stale position. %d ms\n", msec_age);
        }
      } else {
        LOG(ERROR, "Never had a position.\n");
        ZeroOutputs();
        return;
      }
    }
    if (position) {
      position->Print(state, sizeof(state));
      LOG(DEBUG, "position={%s}\n", state);
    }
  }

  bool outputs_enabled = false;

  // Check to see if we got a driver station packet recently.
  if (aos::robot_state.FetchLatest()) {
    outputs_enabled = true;
  } else if (aos::robot_state.IsNewerThanMS(kDSPacketTimeoutMs)) {
    outputs_enabled = true;
  } else {
    if (aos::robot_state.get()) {
      int msec_age = aos::robot_state.Age().ToMSec();
      LOG(ERROR, "Driver Station packet is too old (%d ms).\n", msec_age);
    } else {
      LOG(ERROR, "No Driver Station packet.\n");
    }
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

    output->Print(state, sizeof(state));
    LOG(DEBUG, "output={%s}\n", state);
    output.Send();
  } else {
    // The outputs are disabled, so pass NULL in for the output.
    RunIteration(goal, position, NULL, status.get());
    ZeroOutputs();
  }

  status->Print(state, sizeof(state));
  LOG(DEBUG, "status={%s}\n", state);
  status.Send();
}

template <class T, bool has_position>
void ControlLoop<T, has_position>::Run() {
  while (true) {
    time::SleepUntil(NextLoopTime());
    Iterate();
  }
}

}  // namespace control_loops
}  // namespace aos
