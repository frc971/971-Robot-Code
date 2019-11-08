#ifndef AOS_ACTIONS_ACTOR_H_
#define AOS_ACTIONS_ACTOR_H_

#include <stdio.h>
#include <inttypes.h>

#include <chrono>
#include <functional>

#include "aos/actions/actions_generated.h"
#include "aos/controls/control_loop.h"
#include "aos/logging/logging.h"
#include "aos/time/time.h"
#include "aos/util/phased_loop.h"

namespace aos {
namespace common {
namespace actions {

template <class T>
class ActorBase {
 public:
  typedef T GoalType;
  typedef typename std::remove_reference<decltype(
      *static_cast<GoalType *>(nullptr)->params())>::type ParamType;

  ActorBase(::aos::EventLoop *event_loop, const ::std::string &name)
      : event_loop_(event_loop),
        status_sender_(event_loop->MakeSender<Status>(name)),
        goal_fetcher_(event_loop->MakeFetcher<GoalType>(name)) {
    AOS_LOG(INFO, "Constructing action %s\n", name.c_str());
    event_loop->MakeWatcher(name,
                            [this](const GoalType &goal) { HandleGoal(goal); });

    // Send out an inital status saying we aren't running to wake up any users
    // who might be waiting forever for the previous action.
    event_loop->OnRun([this]() {
      auto builder = status_sender_.MakeBuilder();
      Status::Builder status_builder = builder.template MakeBuilder<Status>();
      status_builder.add_running(0);
      status_builder.add_last_running(0);
      status_builder.add_success(!abort_);
      if (!builder.Send(status_builder.Finish())) {
        AOS_LOG(ERROR, "Failed to send the status.\n");
      }
    });
  }

  // Waits for a certain amount of time from when this method is called.
  // Returns false if the action was canceled or failed, and true if the wait
  // succeeded.
  bool WaitOrCancel(monotonic_clock::duration duration) {
    return !WaitUntil([]() { return false; },
                      event_loop_->monotonic_now() + duration);
  }

  // Returns true if the action should be canceled.
  bool ShouldCancel();

  enum class State {
    WAITING_FOR_ACTION,
    RUNNING_ACTION,
    WAITING_FOR_STOPPED,
  };

  // Returns the number of times we have run.
  int running_count() const { return running_count_; }

  // Returns the current action id being run, or 0 if stopped.
  uint32_t current_id() const { return current_id_; }

 protected:
  // Will run until the done condition is met or times out.
  // Will return false if successful or end_time is reached and true if the
  // action was canceled or failed.
  // Done condition are defined as functions that return true when done
  // end_time is when to stop and return true. Time(0, 0) (the default) means
  // never time out.
  // This will be polled at ::aos::controls::kLoopFrequency
  bool WaitUntil(::std::function<bool(void)> done_condition,
                 ::aos::monotonic_clock::time_point end_time =
                     ::aos::monotonic_clock::min_time);

  // Set to true when we should stop ASAP.
  bool abort_ = false;

  ::aos::EventLoop *event_loop() { return event_loop_; }

  ::aos::monotonic_clock::time_point monotonic_now() {
    return event_loop_->monotonic_now();
  }

 private:
  // Checks if an action was initially running when the thread started.
  bool CheckInitialRunning();

  // Will return true if finished or asked to cancel.
  // Will return false if it failed accomplish its goal
  // due to a problem with the system.
  virtual bool RunAction(const ParamType *params) = 0;

  void HandleGoal(const GoalType &goal);

  ::aos::EventLoop *event_loop_;

  // Number of times we've run.
  int running_count_ = 0;

  uint32_t current_id_ = 0;

  ::aos::Sender<Status> status_sender_;
  ::aos::Fetcher<GoalType> goal_fetcher_;

  State state_ = State::WAITING_FOR_ACTION;
};

template <class T>
void ActorBase<T>::HandleGoal(const GoalType &goal) {
  VLOG(1) << "action goal " << FlatbufferToJson(&goal);
  switch (state_) {
    case State::WAITING_FOR_ACTION:
      if (goal.run()) {
        state_ = State::RUNNING_ACTION;
      } else {
        auto builder = status_sender_.MakeBuilder();
        Status::Builder status_builder = builder.template MakeBuilder<Status>();
        status_builder.add_running(0);
        status_builder.add_last_running(0);
        status_builder.add_success(!abort_);
        if (!builder.Send(status_builder.Finish())) {
          AOS_LOG(ERROR, "Failed to send the status.\n");
        }
        break;
      }
      [[fallthrough]];
    case State::RUNNING_ACTION: {
      ++running_count_;
      const uint32_t running_id = goal.run();
      current_id_ = running_id;
      AOS_LOG(INFO, "Starting action %" PRIx32 "\n", running_id);
      {
        auto builder = status_sender_.MakeBuilder();
        Status::Builder status_builder = builder.template MakeBuilder<Status>();
        status_builder.add_running(running_id);
        status_builder.add_last_running(0);
        status_builder.add_success(!abort_);
        if (!builder.Send(status_builder.Finish())) {
          AOS_LOG(ERROR, "Failed to send the status.\n");
        }
      }

      VLOG(1) << "goal " << FlatbufferToJson(&goal);
      abort_ = !RunAction(goal.params());
      AOS_LOG(INFO, "Done with action %" PRIx32 "\n", running_id);
      current_id_ = 0u;

      {
        auto builder = status_sender_.MakeBuilder();
        Status::Builder status_builder = builder.template MakeBuilder<Status>();
        status_builder.add_running(0);
        status_builder.add_last_running(running_id);
        status_builder.add_success(!abort_);

        if (!builder.Send(status_builder.Finish())) {
          AOS_LOG(ERROR, "Failed to send the status.\n");
        } else {
          AOS_LOG(INFO, "Sending Done status %" PRIx32 "\n", running_id);
        }
      }

      state_ = State::WAITING_FOR_STOPPED;
      AOS_LOG(INFO, "Waiting for the action (%" PRIx32 ") to be stopped.\n",
              running_id);
    } break;
    case State::WAITING_FOR_STOPPED:
      if (goal.run() == 0) {
        AOS_LOG(INFO, "Action stopped.\n");
        state_ = State::WAITING_FOR_ACTION;
      }
      break;
  }
}

template <class T>
bool ActorBase<T>::WaitUntil(::std::function<bool(void)> done_condition,
                             ::aos::monotonic_clock::time_point end_time) {
  ::aos::time::PhasedLoop phased_loop(::aos::controls::kLoopFrequency,
                                      event_loop_->monotonic_now(),
                                      ::std::chrono::milliseconds(5) / 2);

  while (!done_condition()) {
    if (ShouldCancel() || abort_) {
      // Clear abort bit as we have just aborted.
      abort_ = false;
      return true;
    }
    if (end_time != ::aos::monotonic_clock::min_time &&
        ::aos::monotonic_clock::now() >= end_time) {
      AOS_LOG(DEBUG, "WaitUntil timed out\n");
      return false;
    }
    phased_loop.SleepUntilNext();
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
  if (goal_fetcher_.Fetch()) {
    VLOG(1) << "goal queue " << FlatbufferToJson(goal_fetcher_.get());
  }
  bool ans = !goal_fetcher_->run() || goal_fetcher_->run() != current_id_;
  if (ans) {
    AOS_LOG(INFO, "Time to stop action\n");
  }
  return ans;
}

}  // namespace actions
}  // namespace common
}  // namespace aos

#endif  // AOS_ACTIONS_ACTOR_H_
