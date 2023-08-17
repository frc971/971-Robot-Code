#include "aos/events/event_loop_runtime.h"

namespace aos {

OnRunForRust::OnRunForRust(EventLoopRuntime *runtime) : runtime_(runtime) {
  ++runtime->child_count_;
}
OnRunForRust::~OnRunForRust() { --runtime_->child_count_; }
bool OnRunForRust::is_running() const { return runtime_->is_running(); }

std::unique_ptr<TimerForRust> TimerForRust::Make(EventLoopRuntime *runtime) {
  auto handler = std::unique_ptr<TimerForRust>(new TimerForRust());
  TimerForRust *inner = handler.get();
  handler->timer_ = runtime->event_loop()->AddTimer([inner, runtime] {
    inner->expired_ = true;
    runtime->DoPoll();
  });
  return handler;
}

bool TimerForRust::Poll() {
  if (expired_) {
    // Reset it for next poll.
    expired_ = false;
    return true;
  }
  return false;
}
}  // namespace aos
