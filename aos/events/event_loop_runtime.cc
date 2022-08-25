#include "aos/events/event_loop_runtime.h"

namespace aos {

OnRunForRust::OnRunForRust(EventLoopRuntime *runtime) : runtime_(runtime) {
  ++runtime->child_count_;
}
OnRunForRust::~OnRunForRust() { --runtime_->child_count_; }
bool OnRunForRust::is_running() const { return runtime_->is_running(); }

}  // namespace aos
