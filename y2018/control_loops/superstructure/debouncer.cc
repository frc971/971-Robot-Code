#include "y2018/control_loops/superstructure/debouncer.h"

namespace y2018 {
namespace control_loops {
namespace superstructure {

void Debouncer::Update(bool new_state) {
  // If the incoming state is different from the one we have stored, increment
  // the counter.
  if (new_state != current_state_) {
    consistent_count_++;
  } else {
    consistent_count_ = 0;
  }

  // If we have reached the number required to change the state, change it.
  if (consistent_count_ >= inputs_before_change_) {
    current_state_ = new_state;
    consistent_count_ = 0;
  }
}

}  // namespace superstructure
}  // namespace control_loops
}  // namespace y2018
