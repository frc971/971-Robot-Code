#ifndef FRC971_ACTIONS_ACTION_CLIENT_H_
#define FRC971_ACTIONS_ACTION_CLIENT_H_

#include <inttypes.h>
#include <sys/types.h>
#include <unistd.h>

#include <type_traits>
#include <atomic>

#include "aos/common/logging/logging.h"
#include "aos/common/queue.h"

namespace frc971 {

class Action {
 public:
  // Cancels the action.
  void Cancel() { DoCancel(); }
  // Returns true if the action is currently running.
  bool Running() { return DoRunning(); }
  // Starts the action.
  void Start() { DoStart(); }

  // Waits until the action has finished.
  void WaitUntilDone() { DoWaitUntilDone(); }

  virtual ~Action() {}

 private:
  virtual void DoCancel() = 0;
  virtual bool DoRunning() = 0;
  virtual void DoStart() = 0;
  virtual void DoWaitUntilDone() = 0;
};

// Templated subclass to hold the type information.
template <typename T>
class TypedAction : public Action {
 public:
  typedef typename std::remove_reference<
      decltype(*(static_cast<T *>(NULL)->goal.MakeMessage().get()))>::type
        GoalType;

  TypedAction(T *queue_group)
      : queue_group_(queue_group),
        goal_(queue_group_->goal.MakeMessage()),
        run_value_(run_counter.fetch_add(1, ::std::memory_order_relaxed) |
                   ((getpid() & 0xFFFF) << 16)) {
    LOG(INFO, "Action %" PRIx32 " created on queue %s\n", run_value_,
        queue_group_->goal.name());
  }

  // Returns the current goal that will be sent when the action is sent.
  GoalType *GetGoal() { return goal_.get(); }

  virtual ~TypedAction() {
    LOG(DEBUG, "Calling destructor of %" PRIx32"\n", run_value_);
    DoCancel();
  }

 private:
  // Cancels the action.
  virtual void DoCancel() {
    if (!sent_started_) {
      LOG(INFO, "action %" PRIx32 " was never started\n", run_value_);
    } else {
      if (interrupted_) {
        LOG(INFO, "action %" PRIx32 " was interrupted -> not cancelling\n",
            run_value_);
      } else {
        LOG(INFO, "Canceling action %" PRIx32 " on queue %s\n", run_value_,
            queue_group_->goal.name());
        queue_group_->goal.MakeWithBuilder().run(0).Send();
      }
    }
  }

  // Returns true if the action is running or we don't have an initial response
  // back from it to signal whether or not it is running.
  virtual bool DoRunning() {
    if (!sent_started_) return false;
    if (has_started_) {
      queue_group_->status.FetchLatest();
      CheckInterrupted();
    } else if (queue_group_->status.FetchLatest()) {
      CheckStarted();
    }
    if (interrupted_) return false;
    if (!has_started_) return true;
    return queue_group_->status.get() &&
           queue_group_->status->running == run_value_;
  }

  virtual void DoWaitUntilDone() {
    assert(sent_started_);
    queue_group_->status.FetchLatest();
    CheckInterrupted();
    while (true) {
      if (interrupted_) return;
      CheckStarted();
      queue_group_->status.FetchNextBlocking();
      CheckStarted();
      CheckInterrupted();
      if (has_started_ && (queue_group_->status.get() &&
                           queue_group_->status->running != run_value_)) {
        return;
      }
    }
  }

  void CheckStarted() {
    if (has_started_) return;
    if (queue_group_->status.get()) {
      if (queue_group_->status->running == run_value_) {
        // It's currently running our instance.
        has_started_ = true;
        LOG(DEBUG, "action %" PRIx32 " has been started\n", run_value_);
      } else if (queue_group_->status->running == old_run_value_) {
        // It's still running an old instance (or still doing nothing).
      } else {
        LOG(WARNING,
            "action %" PRIx32 " interrupted by %" PRIx32 " before starting\n",
            run_value_, queue_group_->status->running);
        has_started_ = true;
        interrupted_ = true;
      }
    }
  }

  void CheckInterrupted() {
    if (!interrupted_ && has_started_ && queue_group_->status.get()) {
      if (queue_group_->status->running != 0 &&
          queue_group_->status->running != run_value_) {
        LOG(WARNING,
            "action %" PRIx32 " interrupted by %" PRIx32 " after starting\n",
            run_value_, queue_group_->status->running);
      }
    }
  }

  // Starts the action if a goal has been created.
  virtual void DoStart() {
    if (goal_) {
      LOG(INFO, "Starting action %" PRIx32 "\n", run_value_);
      goal_->run = run_value_;
      sent_started_ = true;
      if (!goal_.Send()) {
        LOG(ERROR, "sending goal for action %" PRIx32 " failed\n", run_value_);
        // Don't wait to see a message with it.
        has_started_ = true;
      }
      queue_group_->status.FetchLatest();
      if (queue_group_->status.get()) {
        old_run_value_ = queue_group_->status->running;
        LOG(INFO, "action %" PRIx32 " already running\n", old_run_value_);
      } else {
        old_run_value_ = 0;
      }
    } else {
      LOG(WARNING, "action %" PRIx32 " already started\n", run_value_);
    }
  }

  T *const queue_group_;
  ::aos::ScopedMessagePtr<GoalType> goal_;

  // Track if we have seen a response to the start message.
  bool has_started_ = false;
  // Track if we have sent an initial start message.
  bool sent_started_ = false;

  // Gets set to true if we ever see somebody else's value in running.
  bool interrupted_ = false;

  // The value we're going to use for goal.run etc.
  const uint32_t run_value_;

  // The old value for running that we may have seen. If we see any value other
  // than this or run_value_, somebody else got in the way and we're done. 0 if
  // there was nothing there to start with. Only valid after sent_started_
  // changes to true.
  uint32_t old_run_value_;

  static ::std::atomic<uint16_t> run_counter;
};

template <typename T>
::std::atomic<uint16_t> TypedAction<T>::run_counter{0};

}  // namespace frc971

#endif  // FRC971_ACTIONS_ACTION_CLIENT_H_
