#ifndef AOS_ACTIONS_ACTIONS_H_
#define AOS_ACTIONS_ACTIONS_H_

#include <inttypes.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <memory>
#include <type_traits>

#include "aos/actions/actions_generated.h"
#include "aos/events/event_loop.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/logging/logging.h"

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
  bool GetCurrentActionState(bool *has_started, bool *sent_started,
                             bool *sent_cancel, bool *interrupted,
                             uint32_t *run_value, uint32_t *old_run_value);

  // Retrieves the internal state of the next action for testing.
  // See comments on the private members of TypedAction<T, S> for details.
  bool GetNextActionState(bool *has_started, bool *sent_started,
                          bool *sent_cancel, bool *interrupted,
                          uint32_t *run_value, uint32_t *old_run_value);

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

  // Run all the checks for one iteration of waiting. Will return true when the
  // action has completed, successfully or not. This is non-blocking.
  bool CheckIteration() { return DoCheckIteration(); }

  // Retrieves the internal state of the action for testing.
  // See comments on the private members of TypedAction<T, S> for details.
  void GetState(bool *has_started, bool *sent_started, bool *sent_cancel,
                bool *interrupted, uint32_t *run_value,
                uint32_t *old_run_value) {
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
  // Updates status for one cycle of waiting
  virtual bool DoCheckIteration() = 0;
  // For testing we will need to get the internal state.
  // See comments on the private members of TypedAction<T, S> for details.
  virtual void DoGetState(bool *has_started, bool *sent_started,
                          bool *sent_cancel, bool *interrupted,
                          uint32_t *run_value, uint32_t *old_run_value) = 0;
};

// Templated subclass to hold the type information.
template <typename T>
class TypedAction : public Action {
 public:
  // A convenient way to refer to the type of our goals.
  typedef T GoalType;
  typedef typename std::remove_reference<decltype(
      *static_cast<GoalType *>(nullptr)->params())>::type ParamType;

  TypedAction(typename ::aos::Fetcher<Status> *status_fetcher,
              typename ::aos::Sender<GoalType> *goal_sender,
              const typename ParamType::NativeTableType &params)
      : status_fetcher_(status_fetcher),
        goal_sender_(goal_sender),
        // This adds 1 to the counter (atomically because it's potentially
        // shared across threads) and then bitwise-ORs the bottom of the PID to
        // differentiate it from other processes's values (ie a unique id).
        run_value_(run_counter_.fetch_add(1, ::std::memory_order_relaxed) |
                   ((getpid() & 0xFFFF) << 16)),
        params_(params) {
    AOS_LOG(
        DEBUG, "Action %" PRIx32 " created on queue %.*s\n", run_value_,
        static_cast<int>(goal_sender_->channel()->name()->string_view().size()),
        goal_sender_->channel()->name()->string_view().data());
    // Clear out any old status messages from before now.
    status_fetcher_->Fetch();
    if (status_fetcher_->get()) {
      VLOG(1) << "have status" << FlatbufferToJson(status_fetcher_->get());
    }
  }

  virtual ~TypedAction() {
    AOS_LOG(DEBUG, "Calling destructor of %" PRIx32 "\n", run_value_);
    DoCancel();
  }

 private:
  void DoCancel() override;

  bool DoRunning() override;

  bool DoCheckIteration() override;

  // Sets the started flag (also possibly the interrupted flag).
  void CheckStarted();

  // Checks for interrupt.
  void CheckInterrupted();

  void DoStart() override;

  void DoGetState(bool *has_started, bool *sent_started, bool *sent_cancel,
                  bool *interrupted, uint32_t *run_value,
                  uint32_t *old_run_value) override {
    if (has_started != nullptr) *has_started = has_started_;
    if (sent_started != nullptr) *sent_started = sent_started_;
    if (sent_cancel != nullptr) *sent_cancel = sent_cancel_;
    if (interrupted != nullptr) *interrupted = interrupted_;
    if (run_value != nullptr) *run_value = run_value_;
    if (old_run_value != nullptr) *old_run_value = old_run_value_;
  }

  typename ::aos::Fetcher<Status> *status_fetcher_;
  typename ::aos::Sender<GoalType> *goal_sender_;

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
  const typename ParamType::NativeTableType params_;

  // The old value for running that we may have seen. If we see any value other
  // than this or run_value_, somebody else got in the way and we're done. 0 if
  // there was nothing there to start with. Only valid after sent_started_
  // changes to true.
  uint32_t old_run_value_;

  static ::std::atomic<uint16_t> run_counter_;
};

template <typename T>
class TypedActionFactory {
 public:
  typedef T GoalType;
  typedef typename std::remove_reference<decltype(
      *static_cast<GoalType *>(nullptr)->params())>::type ParamType;

  explicit TypedActionFactory(::aos::EventLoop *event_loop,
                              const ::std::string &name)
      : name_(name),
        status_fetcher_(event_loop->MakeFetcher<Status>(name)),
        goal_sender_(event_loop->MakeSender<GoalType>(name)) {}

  ::std::unique_ptr<TypedAction<T>> Make(
      const typename ParamType::NativeTableType &param) {
    return ::std::unique_ptr<TypedAction<T>>(
        new TypedAction<T>(&status_fetcher_, &goal_sender_, param));
  }

  TypedActionFactory(TypedActionFactory &&other)
      : name_(::std::move(other.name_)),
        status_fetcher_(::std::move(other.status_fetcher_)),
        goal_sender_(::std::move(other.goal_sender_)) {}

 private:
  const ::std::string name_;
  typename ::aos::Fetcher<Status> status_fetcher_;
  typename ::aos::Sender<GoalType> goal_sender_;
};

template <typename T>
::std::atomic<uint16_t> TypedAction<T>::run_counter_{0};

template <typename T>
void TypedAction<T>::DoCancel() {
  if (!sent_started_) {
    AOS_LOG(
        INFO, "Action %" PRIx32 " on queue %.*s was never started\n",
        run_value_,
        static_cast<int>(goal_sender_->channel()->name()->string_view().size()),
        goal_sender_->channel()->name()->string_view().data());
  } else {
    if (interrupted_) {
      AOS_LOG(INFO,
              "Action %" PRIx32
              " on queue %.*s was interrupted -> not cancelling\n",
              run_value_,
              static_cast<int>(
                  goal_sender_->channel()->name()->string_view().size()),
              goal_sender_->channel()->name()->string_view().data());
    } else {
      if (sent_cancel_) {
        AOS_LOG(DEBUG, "Action %" PRIx32 " on queue %.*s already cancelled\n",
                run_value_,
                static_cast<int>(
                    goal_sender_->channel()->name()->string_view().size()),
                goal_sender_->channel()->name()->string_view().data());
      } else {
        AOS_LOG(DEBUG, "Canceling action %" PRIx32 " on queue %.*s\n",
                run_value_,
                static_cast<int>(
                    goal_sender_->channel()->name()->string_view().size()),
                goal_sender_->channel()->name()->string_view().data());
        {
          auto builder = goal_sender_->MakeBuilder();
          typename GoalType::Builder goal_builder =
              builder.template MakeBuilder<GoalType>();
          goal_builder.add_run(0);
          builder.Send(goal_builder.Finish());
        }
        sent_cancel_ = true;
      }
    }
  }
}

template <typename T>
bool TypedAction<T>::DoRunning() {
  if (!sent_started_) {
    AOS_LOG(DEBUG, "haven't sent start message yet\n");
    return false;
  }
  if (has_started_) {
    status_fetcher_->FetchNext();
    CheckInterrupted();
  } else {
    while (status_fetcher_->FetchNext()) {
      VLOG(1) << "got status" << FlatbufferToJson(status_fetcher_->get());
      CheckStarted();
      if (has_started_) CheckInterrupted();
    }
  }
  if (interrupted_) return false;
  // We've asked it to start but haven't gotten confirmation that it's started
  // yet.
  if (!has_started_) return true;
  return status_fetcher_->get() &&
         status_fetcher_->get()->running() == run_value_;
}

template <typename T>
bool TypedAction<T>::DoCheckIteration() {
  AOS_CHECK(sent_started_);
  if (interrupted_) return true;
  CheckStarted();
  if (!status_fetcher_->FetchNext()) {
    return false;
  }
  VLOG(1) << "got status" << FlatbufferToJson(status_fetcher_->get());
  CheckStarted();
  CheckInterrupted();
  if (has_started_ && (status_fetcher_->get() &&
                       status_fetcher_->get()->running() != run_value_)) {
    return true;
  }
  return false;
}

template <typename T>
void TypedAction<T>::CheckStarted() {
  if (has_started_) return;
  if (status_fetcher_->get()) {
    if (status_fetcher_->get()->running() == run_value_ ||
        (status_fetcher_->get()->running() == 0 &&
         status_fetcher_->get()->last_running() == run_value_)) {
      // It's currently running our instance.
      has_started_ = true;
      AOS_LOG(DEBUG, "Action %" PRIx32 " on queue %.*s has been started\n",
              run_value_,
              static_cast<int>(
                  goal_sender_->channel()->name()->string_view().size()),
              goal_sender_->channel()->name()->string_view().data());
    } else if (old_run_value_ != 0 &&
               status_fetcher_->get()->running() == old_run_value_) {
      AOS_LOG(DEBUG, "still running old instance %" PRIx32 "\n",
              old_run_value_);
    } else {
      AOS_LOG(WARNING,
              "Action %" PRIx32 " on queue %.*s interrupted by %" PRIx32
              " before starting\n",
              run_value_,
              static_cast<int>(
                  goal_sender_->channel()->name()->string_view().size()),
              goal_sender_->channel()->name()->string_view().data(),
              status_fetcher_->get()->running());
      has_started_ = true;
      interrupted_ = true;
    }
  } else {
    AOS_LOG(WARNING, "No status message recieved.\n");
  }
}

template <typename T>
void TypedAction<T>::CheckInterrupted() {
  if (!interrupted_ && has_started_ && status_fetcher_->get()) {
    if (status_fetcher_->get()->running() != 0 &&
        status_fetcher_->get()->running() != run_value_) {
      AOS_LOG(WARNING,
              "Action %" PRIx32 " on queue %.*s interrupted by %" PRIx32
              " after starting\n",
              run_value_,
              static_cast<int>(
                  goal_sender_->channel()->name()->string_view().size()),
              goal_sender_->channel()->name()->string_view().data(),
              status_fetcher_->get()->running());
    }
  }
}

template <typename T>
void TypedAction<T>::DoStart() {
  if (!sent_started_) {
    AOS_LOG(DEBUG, "Starting action %" PRIx32 "\n", run_value_);
    auto builder = goal_sender_->MakeBuilder();
    auto params_offset = ParamType::Pack(*builder.fbb(), &params_);

    auto goal_builder = builder.template MakeBuilder<GoalType>();
    goal_builder.add_params(params_offset);
    goal_builder.add_run(run_value_);

    sent_started_ = true;
    if (!builder.Send(goal_builder.Finish())) {
      AOS_LOG(ERROR,
              "sending goal for action %" PRIx32 " on queue %.*s failed\n",
              run_value_,
              static_cast<int>(
                  goal_sender_->channel()->name()->string_view().size()),
              goal_sender_->channel()->name()->string_view().data());
      // Don't wait to see a message with it.
      has_started_ = true;
    }
    status_fetcher_->FetchNext();
    if (status_fetcher_->get()) {
      VLOG(1) << "got status" << FlatbufferToJson(status_fetcher_->get());
    }
    if (status_fetcher_->get() && status_fetcher_->get()->running() != 0) {
      old_run_value_ = status_fetcher_->get()->running();
      AOS_LOG(INFO, "Action %" PRIx32 " on queue %.*s already running\n",
              old_run_value_,
              static_cast<int>(
                  goal_sender_->channel()->name()->string_view().size()),
              goal_sender_->channel()->name()->string_view().data());
    } else {
      old_run_value_ = 0;
    }
  } else {
    AOS_LOG(
        WARNING, "Action %" PRIx32 " on queue %.*s already started\n",
        run_value_,
        static_cast<int>(goal_sender_->channel()->name()->string_view().size()),
        goal_sender_->channel()->name()->string_view().data());
  }
}

}  // namespace actions
}  // namespace common
}  // namespace aos

#endif  // AOS_ACTIONS_ACTIONS_H_
