#ifndef FRC971_CONTROL_LOOPS_HALL_EFFECT_H_
#define FRC971_CONTROL_LOOPS_HALL_EFFECT_H_

#include <stdint.h>

#include "frc971/control_loops/control_loops.q.h"

namespace frc971 {

// TODO(aschuh): Can we filter for 2 cycles instead of just 1?
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
  bool last_value() const { return last_value_; }
  bool is_posedge() const { return value() && !last_value(); }
  bool is_negedge() const { return !value() && last_value(); }

  double posedge_value() const { return posedge_value_; }
  double negedge_value() const { return negedge_value_; }

  void Update(const HallEffectStruct &position) {
    last_value_ = value_;
    value_ = position.current;
    posedge_value_ = position.posedge_value;
    negedge_value_ = position.negedge_value;
    posedges_.update(position.posedge_count);
    negedges_.update(position.negedge_count);
  }

  void Reset(const HallEffectStruct &position) {
    posedges_.Reset(position.posedge_count);
    negedges_.Reset(position.negedge_count);
    value_ = position.current;
    last_value_ = position.current;
    posedge_value_ = position.posedge_value;
    negedge_value_ = position.negedge_value;
  }

 private:
  class {
   public:
    void update(int32_t count) {
      previous_count_ = count_;
      count_ = count;
    }

    void Reset(int32_t count) { count_ = count; }

    bool count_changed() const { return previous_count_ != count_; }

    int32_t count() const { return count_; }

   private:
    int32_t count_ = 0;
    int32_t previous_count_ = 0;
  } posedges_, negedges_;

  bool value_ = false;
  bool last_value_ = false;

  double posedge_value_ = 0, negedge_value_ = 0;
};

}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_HALL_EFFECT_H_
