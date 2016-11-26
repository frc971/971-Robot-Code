#ifndef AOS_COMMON_ACTIONS_ACTOR_H_
#define AOS_COMMON_ACTIONS_ACTOR_H_

#include <stdio.h>
#include <inttypes.h>

#include <functional>

#include "aos/common/logging/logging.h"
#include "aos/common/logging/queue_logging.h"
#include "aos/common/time.h"
#include "aos/common/controls/control_loop.h"
#include "aos/common/util/phased_loop.h"

namespace aos {
namespace common {
namespace actions {

template <class T>
class ActorBase {
 public:
  typedef typename std::remove_reference<decltype(
      *(static_cast<T*>(nullptr)->goal.MakeMessage().get()))>::type GoalType;
  typedef typename std::remove_reference<
      decltype(static_cast<GoalType*>(nullptr)->params)>::type ParamType;

  ActorBase(T* acq) : action_q_(acq) {}

  // Will return true if finished or asked to cancel.
  // Will return false if it failed accomplish its goal
  // due to a problem with the system.
  virtual bool RunAction(const ParamType& params) = 0;

  // Runs action while enabled.
  void Run();

  // Gets ready to run actions.
  void Initialize();

  // Checks if an action was initially running when the thread started.
  bool CheckInitialRunning();

  // Wait here until someone asks us to go.
  void WaitForActionRequest();

  // Do work son.
  uint32_t RunIteration();

  // Wait for stop is signalled.
  void WaitForStop(uint32_t running_id);

  // Will run until the done condition is met or times out.
  // Will return false if successful or end_time is reached and true if the
  // action was canceled or failed.
  // Done condition are defined as functions that return true when done and have
  // some sort of blocking statement (such as FetchNextBlocking) to throttle
  // spin rate.
  // end_time is when to stop and return true. Time(0, 0) (the default) means
  // never time out.
  bool WaitUntil(::std::function<bool(void)> done_condition,
                 ::aos::monotonic_clock::time_point end_time =
                     ::aos::monotonic_clock::min_time);

  // Waits for a certain amount of time from when this method is called.
  // Returns false if the action was canceled or failed, and true if the wait
  // succeeded.
  bool WaitOrCancel(monotonic_clock::duration duration) {
    return !WaitUntil([]() {
      ::aos::time::PhasedLoopXMS(
          ::std::chrono::duration_cast<::std::chrono::milliseconds>(
              ::aos::controls::kLoopFrequency).count(),
          2500);
      return false;
    }, ::aos::monotonic_clock::now() + duration);
  }

  // Returns true if the action should be canceled.
  bool ShouldCancel();

 protected:
  // Set to true when we should stop ASAP.
  bool abort_ = false;

  // The queue for this action.
  T* action_q_;
};

template <class T>
bool ActorBase<T>::CheckInitialRunning() {
  LOG(DEBUG, "Waiting for input to start\n");

  if (action_q_->goal.FetchLatest()) {
    LOG_STRUCT(DEBUG, "goal queue", *action_q_->goal);
    const uint32_t initially_running = action_q_->goal->run;
    if (initially_running != 0) {
      while (action_q_->goal->run == initially_running) {
        LOG(INFO, "run is still %" PRIx32 "\n", initially_running);
        action_q_->goal.FetchNextBlocking();
        LOG_STRUCT(DEBUG, "goal queue", *action_q_->goal);
      }
    }
    LOG(DEBUG, "Done waiting, goal\n");
    return true;
  }
  LOG(DEBUG, "Done waiting, no goal\n");
  return false;
}

template <class T>
void ActorBase<T>::WaitForActionRequest() {
  while (action_q_->goal.get() == nullptr || !action_q_->goal->run) {
    LOG(INFO, "Waiting for an action request.\n");
    action_q_->goal.FetchNextBlocking();
    LOG_STRUCT(DEBUG, "goal queue", *action_q_->goal);
    if (!action_q_->goal->run) {
      if (!action_q_->status.MakeWithBuilder()
               .running(0)
               .last_running(0)
               .success(!abort_)
               .Send()) {
        LOG(ERROR, "Failed to send the status.\n");
      }
    }
  }
}

template <class T>
uint32_t ActorBase<T>::RunIteration() {
  CHECK(action_q_->goal.get() != nullptr);
  const uint32_t running_id = action_q_->goal->run;
  LOG(INFO, "Starting action %" PRIx32 "\n", running_id);
  if (!action_q_->status.MakeWithBuilder()
           .running(running_id)
           .last_running(0)
           .success(!abort_)
           .Send()) {
    LOG(ERROR, "Failed to send the status.\n");
  }
  LOG_STRUCT(INFO, "goal", *action_q_->goal);
  abort_ = !RunAction(action_q_->goal->params);
  LOG(INFO, "Done with action %" PRIx32 "\n", running_id);

  // If we have a new one to run, we shouldn't say we're stopped in between.
  if (action_q_->goal->run == 0 || action_q_->goal->run == running_id) {
    if (!action_q_->status.MakeWithBuilder()
             .running(0)
             .last_running(running_id)
             .success(!abort_)
             .Send()) {
      LOG(ERROR, "Failed to send the status.\n");
    } else {
      LOG(INFO, "Sending Done status %" PRIx32 "\n", running_id);
    }
  } else {
    LOG(INFO, "skipping sending stopped status for %" PRIx32 "\n", running_id);
  }

  return running_id;
}

template <class T>
void ActorBase<T>::WaitForStop(uint32_t running_id) {
  assert(action_q_->goal.get() != nullptr);
  while (action_q_->goal->run == running_id) {
    LOG(INFO, "Waiting for the action (%" PRIx32 ") to be stopped.\n",
        running_id);
    action_q_->goal.FetchNextBlocking();
    LOG_STRUCT(DEBUG, "goal queue", *action_q_->goal);
  }
}

template <class T>
void ActorBase<T>::Run() {
  Initialize();

  while (true) {
    // Wait for a request to come in before starting.
    WaitForActionRequest();

    LOG_STRUCT(INFO, "running with goal", *action_q_->goal);

    // Perform the action once.
    uint32_t running_id = RunIteration();

    LOG(INFO, "done running\n");

    // Don't start again until asked.
    WaitForStop(running_id);
    LOG(DEBUG, "action %" PRIx32 " was stopped\n", running_id);
  }
}

template <class T>
void ActorBase<T>::Initialize() {
  // Make sure the last job is done and we have a signal.
  if (CheckInitialRunning()) {
    LOG(DEBUG, "action %" PRIx32 " was stopped\n", action_q_->goal->run);
  }

  if (!action_q_->status.MakeWithBuilder()
           .running(0)
           .last_running(0)
           .success(!abort_)
           .Send()) {
    LOG(ERROR, "Failed to send the status.\n");
  }
}

template <class T>
bool ActorBase<T>::WaitUntil(::std::function<bool(void)> done_condition,
                             ::aos::monotonic_clock::time_point end_time) {
  while (!done_condition()) {
    if (ShouldCancel() || abort_) {
      // Clear abort bit as we have just aborted.
      abort_ = false;
      return true;
    }
    if (end_time != ::aos::monotonic_clock::min_time &&
        ::aos::monotonic_clock::now() >= end_time) {
      LOG(DEBUG, "WaitUntil timed out\n");
      return false;
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

template <class T>
bool ActorBase<T>::ShouldCancel() {
  if (action_q_->goal.FetchNext()) {
    LOG_STRUCT(DEBUG, "goal queue", *action_q_->goal);
  }
  bool ans = !action_q_->goal->run;
  if (ans) {
    LOG(INFO, "Time to stop action\n");
  }
  return ans;
}

}  // namespace actions
}  // namespace common
}  // namespace aos

#endif  // AOS_COMMON_ACTIONS_ACTOR_H_
