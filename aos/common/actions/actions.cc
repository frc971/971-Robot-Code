#include "aos/common/actions/actions.h"

namespace aos {
namespace common {
namespace actions {

void ActionQueue::EnqueueAction(::std::unique_ptr<Action> action) {
  if (current_action_) {
    LOG(INFO, "Queueing action, canceling prior\n");
    current_action_->Cancel();
    next_action_ = ::std::move(action);
  } else {
    LOG(INFO, "Queueing action\n");
    current_action_ = ::std::move(action);
    current_action_->Start();
  }
}

void ActionQueue::CancelCurrentAction() {
  LOG(INFO, "Canceling current action\n");
  if (current_action_) {
    current_action_->Cancel();
  }
}

void ActionQueue::CancelAllActions() {
  LOG(DEBUG, "Cancelling all actions\n");
  if (current_action_) {
    current_action_->Cancel();
  }
  next_action_.reset();
}

void ActionQueue::Tick() {
  if (current_action_) {
    if (!current_action_->Running()) {
      LOG(INFO, "Action is done.\n");
      current_action_ = ::std::move(next_action_);
      if (current_action_) {
        LOG(INFO, "Running next action\n");
        current_action_->Start();
      }
    }
  }
}

bool ActionQueue::Running() { return static_cast<bool>(current_action_); }

bool ActionQueue::GetCurrentActionState(bool* has_started, bool* sent_started,
                                        bool* sent_cancel, bool* interrupted,
                                        uint32_t* run_value,
                                        uint32_t* old_run_value) {
  if (current_action_) {
    current_action_->GetState(has_started, sent_started, sent_cancel,
                              interrupted, run_value, old_run_value);
    return true;
  }
  return false;
}

bool ActionQueue::GetNextActionState(bool* has_started, bool* sent_started,
                                     bool* sent_cancel, bool* interrupted,
                                     uint32_t* run_value,
                                     uint32_t* old_run_value) {
  if (next_action_) {
    next_action_->GetState(has_started, sent_started, sent_cancel, interrupted,
                           run_value, old_run_value);
    return true;
  }
  return false;
}

}  // namespace actions
}  // namespace common
}  // namespace aos
