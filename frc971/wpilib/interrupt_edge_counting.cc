#include "frc971/wpilib/interrupt_edge_counting.h"

#include <chrono>

#include "aos/common/time.h"
#include "aos/linux_code/init.h"

namespace frc971 {
namespace wpilib {

void EdgeCounter::GatherPolledValue() {
  shadow_values_.polled_value = input_->Get();
  bool miss_match = (shadow_values_.polled_value != current_value_);
  if (miss_match && last_miss_match_) {
    current_value_ = shadow_values_.polled_value;
    last_miss_match_ = false;
  } else {
    last_miss_match_ = miss_match;
  }
}

void EdgeCounter::operator()() {
  ::aos::SetCurrentThreadName("EdgeCounter_" +
#ifdef WPILIB2017
                              ::std::to_string(input_->GetChannel()));
#else
                              ::std::to_string(input_->GetChannelForRouting()));
#endif

  input_->RequestInterrupts();
  input_->SetUpSourceEdge(true, true);

  {
    ::std::unique_lock<::aos::stl_mutex> mutex_guard(*mutex());
    current_value_ = input_->Get();
  }

  ::aos::SetCurrentThreadRealtimePriority(priority());
  InterruptableSensorBase::WaitResult result = InterruptableSensorBase::kBoth;
  while (should_run()) {
    result = input_->WaitForInterrupt(
        0.1, result != InterruptableSensorBase::kTimeout);
    if (result == InterruptableSensorBase::kTimeout) {
      continue;
    }
    interrupt_received();

    ::std::unique_lock<::aos::stl_mutex> mutex_guard(*mutex());
    int32_t encoder_value = encoder_->GetRaw();
    bool hall_value = input_->Get();
    if (current_value_ != hall_value) {
      if (hall_value) {
        ++shadow_values_.positive_interrupt_count;
        shadow_values_.last_positive_encoder_value = encoder_value;
      } else {
        ++shadow_values_.negative_interrupt_count;
        shadow_values_.last_negative_encoder_value = encoder_value;
      }
      current_value_ = hall_value;
    } else {
      LOG(WARNING, "Detected spurious edge on %d.  Dropping it.\n",
#ifdef WPILIB2017
          input_->GetChannel());
#else
          input_->GetChannelForRouting());
#endif
    }
  }
}

void InterruptSynchronizer::RunIteration() {
  while (true) {
    if (!TryStartIteration()) continue;

    // Wait more than the amount of time it takes for a digital input change
    // to go from visible to software to having triggered an interrupt.
    ::std::this_thread::sleep_for(::std::chrono::microseconds(120));

    if (TryFinishingIteration()) return;
  }
}

bool InterruptSynchronizer::TryStartIteration() {
  for (auto &c : handlers_) {
    c->save_interrupt_count();
  }

  {
    ::std::unique_lock<::aos::stl_mutex> mutex_guard(mutex_);
    for (auto &c : handlers_) {
      c->GatherPolledValue();
    }
  }
  return true;
}

bool InterruptSynchronizer::TryFinishingIteration() {
  // Make sure no interrupts have occurred while we were waiting.  If they
  // have, we are in an inconsistent state and need to try again.
  ::std::unique_lock<::aos::stl_mutex> mutex_guard(mutex_);
  for (auto &c : handlers_) {
    if (c->interrupt_count_changed()) {
      LOG(WARNING, "got an interrupt while sampling. retrying\n");
      return false;
    }
  }
  for (auto &c : handlers_) {
    c->CommitValue();
  }
  return true;
}

}  // namespace wpilib
}  // namespace frc971
