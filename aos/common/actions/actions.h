#ifndef AOS_COMMON_ACTIONS_ACTIONS_H_
#define AOS_COMMON_ACTIONS_ACTIONS_H_

#include <inttypes.h>
#include <sys/types.h>
#include <unistd.h>

#include <type_traits>
#include <atomic>
#include <memory>

#include "aos/common/logging/logging.h"
#include "aos/common/queue.h"
#include "aos/common/logging/queue_logging.h"

namespace aos {
namespace common {
namespace actions {

class Action;

// A queue which queues Actions, verifies a single action is running, and
// cancels Actions.
class ActionQueue {
 public:
  // Queues up an action for sending.
  void EnqueueAction(::std::unique_ptr<Action> action);

  // Cancels the current action, and runs the next one after the current one has
  // finished and the action queue ticks again.
  void CancelCurrentAction();

  // Cancels all running actions.
  void CancelAllActions();

  // Runs the next action when the current one is finished running and polls the
  // running action state for use by Running.
  void Tick();

  // Returns true if any action is running or could be running.
  bool Running();

  // Retrieves the internal state of the current action for testing.
  // See comments on the private members of TypedAction<T, S> for details.
  bool GetCurrentActionState(bool* has_started, bool* sent_started,
                             bool* sent_cancel, bool* interrupted,
                             uint32_t* run_value, uint32_t* old_run_value);

  // Retrieves the internal state of the next action for testing.
  // See comments on the private members of TypedAction<T, S> for details.
  bool GetNextActionState(bool* has_started, bool* sent_started,
                          bool* sent_cancel, bool* interrupted,
                          uint32_t* run_value, uint32_t* old_run_value);

 private:
  ::std::unique_ptr<Action> current_action_;
  ::std::unique_ptr<Action> next_action_;
};

// The basic interface an ActionQueue can access.
class Action {
 public:
  virtual ~Action() {}

  // Cancels the action.
  void Cancel() { DoCancel(); }
  // Returns true if the action is currently running.
  bool Running() { return DoRunning(); }
  // Starts the action.
  void Start() { DoStart(); }

  // Waits until the action has finished.
  void WaitUntilDone() { DoWaitUntilDone(); }

  // Run all the checks for one iteration of waiting. Will return true when the
  // action has completed, successfully or not. This is non-blocking.
  bool CheckIteration() { return DoCheckIteration(false); }

  // Retrieves the internal state of the action for testing.
  // See comments on the private members of TypedAction<T, S> for details.
  void GetState(bool* has_started, bool* sent_started, bool* sent_cancel,
                bool* interrupted, uint32_t* run_value,
                uint32_t* old_run_value) {
    DoGetState(has_started, sent_started, sent_cancel, interrupted, run_value,
               old_run_value);
  }

 private:
  // Cancels the action.
  virtual void DoCancel() = 0;
  // Returns true if the action is running or we don't have an initial response
  // back from it to signal whether or not it is running.
  virtual bool DoRunning() = 0;
  // Starts the action if a goal has been created.
  virtual void DoStart() = 0;
  // Blocks until complete.
  virtual void DoWaitUntilDone() = 0;
  // Updates status for one cycle of waiting
  virtual bool DoCheckIteration(bool blocking) = 0;
  // For testing we will need to get the internal state.
  // See comments on the private members of TypedAction<T, S> for details.
  virtual void DoGetState(bool* has_started, bool* sent_started,
                          bool* sent_cancel, bool* interrupted,
                          uint32_t* run_value, uint32_t* old_run_value) = 0;
};

// Templated subclass to hold the type information.
template <typename T>
class TypedAction : public Action {
 public:
  // A convenient way to refer to the type of our goals.
  typedef typename std::remove_reference<decltype(
      *(static_cast<T*>(nullptr)->goal.MakeMessage().get()))>::type GoalType;
  typedef typename std::remove_reference<
      decltype(static_cast<GoalType*>(nullptr)->params)>::type ParamType;

  TypedAction(T* queue_group, const ParamType &params)
      : queue_group_(queue_group),
        goal_(queue_group_->goal.MakeMessage()),
        // This adds 1 to the counter (atomically because it's potentially
        // shared across threads) and then bitwise-ORs the bottom of the PID to
        // differentiate it from other processes's values (ie a unique id).
        run_value_(run_counter_.fetch_add(1, ::std::memory_order_relaxed) |
                   ((getpid() & 0xFFFF) << 16)),
        params_(params) {
    LOG(INFO, "Action %" PRIx32 " created on queue %s\n", run_value_,
        queue_group_->goal.name());
    // Clear out any old status messages from before now.
    queue_group_->status.FetchLatest();
    if (queue_group_->status.get()) {
      LOG_STRUCT(DEBUG, "have status", *queue_group_->status);
    }
  }

  virtual ~TypedAction() {
    LOG(DEBUG, "Calling destructor of %" PRIx32 "\n", run_value_);
    DoCancel();
  }

  // Returns the current goal that will be sent when the action is sent.
  GoalType* GetGoal() { return goal_.get(); }

 private:
  void DoCancel() override;

  bool DoRunning() override;

  void DoWaitUntilDone() override;

  bool DoCheckIteration(bool blocking) override;

  // Sets the started flag (also possibly the interrupted flag).
  void CheckStarted();

  // Checks for interrupt.
  void CheckInterrupted();

  void DoStart() override;

  void DoGetState(bool* has_started, bool* sent_started, bool* sent_cancel,
                  bool* interrupted, uint32_t* run_value,
                  uint32_t* old_run_value) override {
    if (has_started != nullptr) *has_started = has_started_;
    if (sent_started != nullptr) *sent_started = sent_started_;
    if (sent_cancel != nullptr) *sent_cancel = sent_cancel_;
    if (interrupted != nullptr) *interrupted = interrupted_;
    if (run_value != nullptr) *run_value = run_value_;
    if (old_run_value != nullptr) *old_run_value = old_run_value_;
  }

  T* const queue_group_;
  ::aos::ScopedMessagePtr<GoalType> goal_;

  // Track if we have seen a response to the start message.
  bool has_started_ = false;
  // Track if we have sent an initial start message.
  bool sent_started_ = false;

  bool sent_cancel_ = false;

  // Gets set to true if we ever see somebody else's value in running.
  bool interrupted_ = false;

  // The value we're going to use for goal.run etc.
  const uint32_t run_value_;

  // flag passed to action in order to have differing types
  const ParamType params_;

  // The old value for running that we may have seen. If we see any value other
  // than this or run_value_, somebody else got in the way and we're done. 0 if
  // there was nothing there to start with. Only valid after sent_started_
  // changes to true.
  uint32_t old_run_value_;

  static ::std::atomic<uint16_t> run_counter_;
};

template <typename T>
::std::atomic<uint16_t> TypedAction<T>::run_counter_{0};

template <typename T>
void TypedAction<T>::DoCancel() {
  if (!sent_started_) {
    LOG(INFO, "Action %" PRIx32 " on queue %s was never started\n", run_value_,
        queue_group_->goal.name());
  } else {
    if (interrupted_) {
      LOG(INFO,
          "Action %" PRIx32 " on queue %s was interrupted -> not cancelling\n",
          run_value_, queue_group_->goal.name());
    } else {
      if (sent_cancel_) {
        LOG(INFO, "Action %" PRIx32 " on queue %s already cancelled\n",
            run_value_, queue_group_->goal.name());
      } else {
        LOG(INFO, "Canceling action %" PRIx32 " on queue %s\n", run_value_,
            queue_group_->goal.name());
        queue_group_->goal.MakeWithBuilder().run(0).Send();
        sent_cancel_ = true;
      }
    }
  }
}

template <typename T>
bool TypedAction<T>::DoRunning() {
  if (!sent_started_) {
    LOG(DEBUG, "haven't sent start message yet\n");
    return false;
  }
  if (has_started_) {
    queue_group_->status.FetchNext();
    CheckInterrupted();
  } else {
    while (queue_group_->status.FetchNext()) {
      LOG_STRUCT(DEBUG, "got status", *queue_group_->status);
      CheckStarted();
      if (has_started_) CheckInterrupted();
    }
  }
  if (interrupted_) return false;
  // We've asked it to start but haven't gotten confirmation that it's started
  // yet.
  if (!has_started_) return true;
  return queue_group_->status.get() &&
         queue_group_->status->running == run_value_;
}

template <typename T>
void TypedAction<T>::DoWaitUntilDone() {
  CHECK(sent_started_);
  queue_group_->status.FetchNext();
  LOG_STRUCT(DEBUG, "got status", *queue_group_->status);
  CheckInterrupted();
  while (true) {
    if (DoCheckIteration(true)) {
      return;
    }
  }
}

template <typename T>
bool TypedAction<T>::DoCheckIteration(bool blocking) {
  CHECK(sent_started_);
  if (interrupted_) return true;
  CheckStarted();
  if (blocking) {
    queue_group_->status.FetchNextBlocking();
    LOG_STRUCT(DEBUG, "got status", *queue_group_->status);
  } else {
    if (!queue_group_->status.FetchNext()) {
      return false;
    }
    LOG_STRUCT(DEBUG, "got status", *queue_group_->status);
  }
  CheckStarted();
  CheckInterrupted();
  if (has_started_ && (queue_group_->status.get() &&
                       queue_group_->status->running != run_value_)) {
    return true;
  }
  return false;
}

template <typename T>
void TypedAction<T>::CheckStarted() {
  if (has_started_) return;
  if (queue_group_->status.get()) {
    if (queue_group_->status->running == run_value_ ||
        (queue_group_->status->running == 0 &&
         queue_group_->status->last_running == run_value_)) {
      // It's currently running our instance.
      has_started_ = true;
      LOG(INFO, "Action %" PRIx32 " on queue %s has been started\n",
          run_value_, queue_group_->goal.name());
    } else if (old_run_value_ != 0 &&
               queue_group_->status->running == old_run_value_) {
      LOG(DEBUG, "still running old instance %" PRIx32 "\n", old_run_value_);
    } else {
      LOG(WARNING, "Action %" PRIx32 " on queue %s interrupted by %" PRIx32
                   " before starting\n",
          run_value_, queue_group_->goal.name(), queue_group_->status->running);
      has_started_ = true;
      interrupted_ = true;
    }
  } else {
    LOG(WARNING, "No status message recieved.\n");
  }
}

template <typename T>
void TypedAction<T>::CheckInterrupted() {
  if (!interrupted_ && has_started_ && queue_group_->status.get()) {
    if (queue_group_->status->running != 0 &&
        queue_group_->status->running != run_value_) {
      LOG(WARNING, "Action %" PRIx32 " on queue %s interrupted by %" PRIx32
                   " after starting\n",
          run_value_, queue_group_->goal.name(), queue_group_->status->running);
    }
  }
}

template <typename T>
void TypedAction<T>::DoStart() {
  if (goal_) {
    LOG(INFO, "Starting action %" PRIx32 "\n", run_value_);
    goal_->run = run_value_;
    goal_->params = params_;
    sent_started_ = true;
    if (!goal_.Send()) {
      LOG(ERROR, "sending goal for action %" PRIx32 " on queue %s failed\n",
          run_value_, queue_group_->goal.name());
      // Don't wait to see a message with it.
      has_started_ = true;
    }
    queue_group_->status.FetchNext();
    if (queue_group_->status.get()) {
      LOG_STRUCT(DEBUG, "got status", *queue_group_->status);
    }
    if (queue_group_->status.get() && queue_group_->status->running != 0) {
      old_run_value_ = queue_group_->status->running;
      LOG(INFO, "Action %" PRIx32 " on queue %s already running\n",
          old_run_value_, queue_group_->goal.name());
    } else {
      old_run_value_ = 0;
    }
  } else {
    LOG(WARNING, "Action %" PRIx32 " on queue %s already started\n", run_value_,
        queue_group_->goal.name());
  }
}

}  // namespace actions
}  // namespace common
}  // namespace aos

#endif  // AOS_COMMON_ACTIONS_ACTIONS_H_
