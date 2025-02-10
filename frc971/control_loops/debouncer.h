#ifndef FRC971_CONTROL_LOOPS_DEBOUNCER_H_
#define FRC971_CONTROL_LOOPS_DEBOUNCER_H_

#include "aos/time/time.h"

namespace frc971::control_loops {

// Debouncer class, which is used to debounce a binary signal (e.g. true/false
// signal). Debouncing is a technique used to filter out noise or rapid
// fluctuations in a signal to ensure that only valid state changes are
// recognized.
class Debouncer {
 public:
  Debouncer(std::chrono::nanoseconds rising_delay,
            std::chrono::nanoseconds falling_delay);

  void Update(bool state, aos::monotonic_clock::time_point now);
  bool state() const { return output_state_; }

 private:
  const std::chrono::nanoseconds rising_delay_;
  const std::chrono::nanoseconds falling_delay_;

  bool state_transition_ = false;
  bool output_state_ = false;
  aos::monotonic_clock::time_point transition_time_ =
      aos::monotonic_clock::min_time;
};

}  // namespace frc971::control_loops

#endif
