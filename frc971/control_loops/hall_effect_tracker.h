#ifndef FRC971_CONTROL_LOOPS_HALL_EFFECT_H_
#define FRC971_CONTROL_LOOPS_HALL_EFFECT_H_

#include <stdint.h>

#include "frc971/control_loops/control_loops.q.h"

namespace frc971 {

// TODO(brians): Have a Reset() for when the cape resets.
class HallEffectTracker {
 public:
  int32_t get_posedges() const { return posedges_.count(); }
  int32_t get_negedges() const { return negedges_.count(); }

  bool either_count_changed() const {
    return posedges_.count_changed() || negedges_.count_changed();
  }
  bool posedge_count_changed() const { return posedges_.count_changed(); }
  bool negedge_count_changed() const { return negedges_.count_changed(); }

  bool value() const { return value_; }

  void Update(const HallEffectStruct &position) {
    value_ = position.current;
    posedges_.update(position.posedge_count);
    negedges_.update(position.negedge_count);
  }

  void Reset() {
    posedges_.Reset();
    negedges_.Reset();
  }

  bool ready() { return posedges_.ready() && negedges_.ready(); }

 private:
  class {
   public:
    void update(int32_t count) {
      if (first_) {
        count_ = count;
        LOG(DEBUG, "First time through the hall effect, resetting\n");
      }
      previous_count_ = count_;
      count_ = count;
      first_ = false;
    }

    void Reset() { first_ = true; }

    bool count_changed() const { return !first_ && previous_count_ != count_; }

    int32_t count() const { return count_; }

    bool ready() { return !first_; }

   private:
    int32_t count_ = 0;
    int32_t previous_count_ = 0;
    bool first_ = true;
  } posedges_, negedges_;
  bool value_ = false;
};

}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_HALL_EFFECT_H_
