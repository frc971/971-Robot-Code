#ifndef FRC971_ACTIONS_ACTION_H_
#define FRC971_ACTIONS_ACTION_H_

#include <stdio.h>

#include <functional>

#include "aos/common/control_loop/Timing.h"
#include "aos/common/logging/logging.h"
#include "aos/common/network/team_number.h"
#include "aos/common/time.h"

#include "frc971/constants.h"

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
    while (!action_q_->goal.get()) {
      action_q_->goal.FetchNextBlocking();
    }

    if (!action_q_->status.MakeWithBuilder().running(false).Send()) {
      LOG(ERROR, "Failed to send the status.\n");
    }
    while (true) {
      while (!action_q_->goal->run) {
        LOG(INFO, "Waiting for an action request.\n");
        action_q_->goal.FetchNextBlocking();
        if (!action_q_->goal->run) {
          if (!action_q_->status.MakeWithBuilder().running(false).Send()) {
            LOG(ERROR, "Failed to send the status.\n");
          }
        }
      }
      LOG(INFO, "Starting action\n");
      if (!action_q_->status.MakeWithBuilder().running(true).Send()) {
        LOG(ERROR, "Failed to send the status.\n");
      }
      RunAction();
      LOG(INFO, "Done with action\n");
      if (!action_q_->status.MakeWithBuilder().running(false).Send()) {
        LOG(ERROR, "Failed to send the status.\n");
      }
      while (action_q_->goal->run) {
        LOG(INFO, "Waiting for the action to be stopped.\n");
        action_q_->goal.FetchNextBlocking();
      }
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
    action_q_->goal.FetchLatest();
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
