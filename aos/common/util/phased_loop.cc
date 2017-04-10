#include "aos/common/util/phased_loop.h"

namespace aos {
namespace time {

int PhasedLoop::Iterate(const monotonic_clock::time_point now) {
  const monotonic_clock::time_point next_time =
      monotonic_clock::time_point(
          (((now - offset_).time_since_epoch() + monotonic_clock::duration(1)) /
           interval_) *
          interval_) +
      ((now.time_since_epoch() < offset_) ? monotonic_clock::zero()
                                          : interval_) +
      offset_;

  const monotonic_clock::duration difference = next_time - last_time_;
  const int result = difference / interval_;
  CHECK_EQ(
      0, (next_time - offset_).time_since_epoch().count() % interval_.count());
  CHECK_GE(next_time, now);
  CHECK_LE(next_time - now, interval_);
  last_time_ = next_time;
  return result;
}

}  // namespace timing
}  // namespace aos
