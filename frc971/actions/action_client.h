#ifndef FRC971_ACTIONS_ACTION_CLIENT_H_
#define FRC971_ACTIONS_ACTION_CLIENT_H_

#include <type_traits>

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
        has_started_(false) {}

  // Returns the current goal that will be sent when the action is sent.
  GoalType *GetGoal() { return goal_.get(); }

  virtual ~TypedAction() {
    LOG(INFO, "Calling destructor\n");
    DoCancel();
  }

 private:
  // Cancels the action.
  virtual void DoCancel() {
    LOG(INFO, "Canceling action on queue %s\n", queue_group_->goal.name());
    queue_group_->goal.MakeWithBuilder().run(false).Send();
  }

  // Returns true if the action is running or we don't have an initial response
  // back from it to signal whether or not it is running.
  virtual bool DoRunning() {
    if (has_started_) {
      queue_group_->status.FetchLatest();
    } else if (queue_group_->status.FetchLatest()) {
      if (queue_group_->status->running) {
        // Wait until it reports that it is running to start.
        has_started_ = true;
      }
    }
    return !has_started_ ||
           (queue_group_->status.get() && queue_group_->status->running);
  }

  // Returns true if the action is running or we don't have an initial response
  // back from it to signal whether or not it is running.
  virtual void DoWaitUntilDone() {
    queue_group_->status.FetchLatest();
    while (true) {
      if (has_started_) {
        queue_group_->status.FetchNextBlocking();
      } else {
        if (queue_group_->status->running) {
          has_started_ = true;
        }
        queue_group_->status.FetchNextBlocking();
        if (queue_group_->status->running) {
          // Wait until it reports that it is running to start.
          has_started_ = true;
        }
      }
      if (has_started_ &&
          (queue_group_->status.get() && !queue_group_->status->running)) {
        return;
      }
    }
  }

  // Starts the action if a goal has been created.
  virtual void DoStart() {
    if (goal_) {
      goal_->run = true;
      goal_.Send();
      has_started_ = false;
      LOG(INFO, "Starting action\n");
    } else {
      has_started_ = true;
    }
  }

  T *queue_group_;
  ::aos::ScopedMessagePtr<GoalType> goal_;
  // Track if we have seen a response to the start message.
  // If we haven't, we are considered running regardless.
  bool has_started_;
};

}  // namespace frc971

#endif  // FRC971_ACTIONS_ACTION_CLIENT_H_
