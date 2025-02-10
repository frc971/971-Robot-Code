#include "frc971/control_loops/debouncer.h"

namespace frc971::control_loops {

Debouncer::Debouncer(std::chrono::nanoseconds rising_delay,
                     std::chrono::nanoseconds falling_delay)
    : rising_delay_(rising_delay), falling_delay_(falling_delay) {}

void Debouncer::Update(bool state, aos::monotonic_clock::time_point now) {
  if (state_transition_ != state) {
    transition_time_ = now;
    state_transition_ = state;
  }

  if (state != output_state_) {
    if (state) {
      output_state_ = now > transition_time_ + rising_delay_;
    } else {
      output_state_ = !(now > transition_time_ + falling_delay_);
    }
  }
}

}  // namespace frc971::control_loops
