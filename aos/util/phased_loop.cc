#include "aos/util/phased_loop.h"

#include "glog/logging.h"

namespace aos {
namespace time {

PhasedLoop::PhasedLoop(const monotonic_clock::duration interval,
                       const monotonic_clock::time_point monotonic_now,
                       const monotonic_clock::duration offset)
    : interval_(interval), offset_(offset), last_time_(offset) {
  CHECK(offset >= monotonic_clock::duration(0));
  CHECK(interval > monotonic_clock::duration(0));
  CHECK(offset < interval);
  Reset(monotonic_now);
}

void PhasedLoop::set_interval_and_offset(
    const monotonic_clock::duration interval,
    const monotonic_clock::duration offset) {
  interval_ = interval;
  offset_ = offset;
  CHECK(offset_ >= monotonic_clock::duration(0));
  CHECK(interval_ > monotonic_clock::duration(0));
  CHECK(offset_ < interval_);
}

monotonic_clock::duration PhasedLoop::OffsetFromIntervalAndTime(
    const monotonic_clock::duration interval,
    const monotonic_clock::time_point monotonic_trigger) {
  CHECK(interval > monotonic_clock::duration(0));
  return monotonic_trigger.time_since_epoch() -
         (monotonic_trigger.time_since_epoch() / interval) * interval +
         ((monotonic_trigger.time_since_epoch() >= monotonic_clock::zero())
              ? monotonic_clock::zero()
              : interval);
}

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
  CHECK(next_time >= now);
  CHECK(next_time - now <= interval_);
  last_time_ = next_time;
  return result;
}

}  // namespace timing
}  // namespace aos
