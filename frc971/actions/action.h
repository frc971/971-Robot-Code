#ifndef FRC971_ACTIONS_ACTION_H_
#define FRC971_ACTIONS_ACTION_H_

#include <stdio.h>
#include <inttypes.h>

#include <functional>

#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/time.h"

namespace frc971 {
namespace actions {

template <class T> class ActionBase {
 public:
  ActionBase(T* acq) : action_q_(acq) {}

  virtual void RunAction() = 0;

  // runs action while enabled
  void Run() {
    LOG(DEBUG, "Waiting for input to start\n");

    action_q_->goal.FetchLatest();
    if (action_q_->goal.get()) {
      LOG_STRUCT(DEBUG, "goal queue ", *action_q_->goal);
      const uint32_t initially_running = action_q_->goal->run;
      if (initially_running != 0) {
        while (action_q_->goal->run == initially_running) {
          LOG(INFO, "run is still %" PRIx32 "\n", initially_running);
          action_q_->goal.FetchNextBlocking();
          LOG_STRUCT(DEBUG, "goal queue ", *action_q_->goal);
        }
      }
    } else {
      action_q_->goal.FetchNextBlocking();
      LOG_STRUCT(DEBUG, "goal queue ", *action_q_->goal);
    }

    LOG(DEBUG, "actually starting\n");

    if (!action_q_->status.MakeWithBuilder().running(0).Send()) {
      LOG(ERROR, "Failed to send the status.\n");
    }
    while (true) {
      while (!action_q_->goal->run) {
        LOG(INFO, "Waiting for an action request.\n");
        action_q_->goal.FetchNextBlocking();
        LOG_STRUCT(DEBUG, "goal queue ", *action_q_->goal);
        if (!action_q_->goal->run) {
          if (!action_q_->status.MakeWithBuilder().running(0).Send()) {
            LOG(ERROR, "Failed to send the status.\n");
          }
        }
      }

      const uint32_t running_id = action_q_->goal->run;
      LOG(INFO, "Starting action %" PRIx32 "\n", running_id);
      if (!action_q_->status.MakeWithBuilder().running(running_id).Send()) {
        LOG(ERROR, "Failed to send the status.\n");
      }
      RunAction();
      LOG(INFO, "Done with action %" PRIx32 "\n", running_id);

      // If we have a new one to run, we shouldn't say we're stopped in between.
      if (action_q_->goal->run == 0 || action_q_->goal->run == running_id) {
        if (!action_q_->status.MakeWithBuilder().running(0).Send()) {
          LOG(ERROR, "Failed to send the status.\n");
        }
      } else {
        LOG(INFO, "skipping sending stopped status for %" PRIx32 "\n",
            running_id);
      }

      while (action_q_->goal->run == running_id) {
        LOG(INFO, "Waiting for the action (%" PRIx32 ") to be stopped.\n",
            running_id);
        action_q_->goal.FetchNextBlocking();
        LOG_STRUCT(DEBUG, "goal queue ", *action_q_->goal);
      }
      LOG(DEBUG, "action %" PRIx32 " was stopped\n", running_id);
    }
  }

  // will run until the done condition is met or times out.
  // will return false if successful and true if the action was canceled or
  // failed or end_time was reached before it succeeded.
  // done condition are defined as functions that return true when done and have
  // some sort of blocking statement(FetchNextBlocking) to throttle spin rate.
  // end_time is when to stop and return true. Time(0, 0) (the default) means
  // never time out.
  bool WaitUntil(::std::function<bool(void)> done_condition,
                 const ::aos::time::Time &end_time = ::aos::time::Time(0, 0)) {
    while (!done_condition()) {
      if (ShouldCancel() || abort_) {
        // Clear abort bit as we have just aborted.
        abort_ = false;
        return true;
      }
      if (end_time != ::aos::time::Time(0, 0) &&
          ::aos::time::Time::Now() >= end_time) {
        LOG(INFO, "WaitUntil timed out\n");
        return true;
      }
    }
    if (ShouldCancel() || abort_) {
      // Clear abort bit as we have just aborted.
      abort_ = false;
      return true;
    } else {
      return false;
    }
  }

  // Returns true if the action should be canceled.
  bool ShouldCancel() {
    if (action_q_->goal.FetchNext()) {
      LOG_STRUCT(DEBUG, "goal queue ", *action_q_->goal);
    }
    bool ans = !action_q_->goal->run;
    if (ans) {
      LOG(INFO, "Time to stop action\n");
    }
    return ans;
  }

 protected:

  // boolean to stop on fail
  bool abort_ = false;

  // queue for this action
  T* action_q_;
};

}  // namespace actions
}  // namespace frc971

#endif  // FRC971_ACTIONS_ACTION_H_
