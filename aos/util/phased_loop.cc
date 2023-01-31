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
    const monotonic_clock::duration offset,
    std::optional<monotonic_clock::time_point> monotonic_now) {
  // Update last_time_ to the new offset so that we have an even interval
  // In doing so, set things so that last_time_ will only ever decrease on calls
  // to set_interval_and_offset.
  last_time_ += offset - offset_ -
                (offset > offset_ ? interval : monotonic_clock::duration(0));

  interval_ = interval;
  offset_ = offset;
  CHECK(offset_ >= monotonic_clock::duration(0));
  CHECK(interval_ > monotonic_clock::duration(0));
  CHECK(offset_ < interval_);
  // Reset effectively clears the skipped iteration count and ensures that the
  // last time is in the interval (monotonic_now - interval, monotonic_now],
  // which means that a call to Iterate(monotonic_now) will return 1 and set a
  // wakeup time after monotonic_now.
  if (monotonic_now.has_value()) {
    Iterate(monotonic_now.value());
  }
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
  auto next_time = monotonic_clock::epoch();
  // Round up to the next whole interval, ignoring offset_.
  {
    const auto offset_now = (now - offset_).time_since_epoch();
    monotonic_clock::duration prerounding;
    if (now.time_since_epoch() >= offset_) {
      // We're above 0, so rounding up means away from 0.
      prerounding = offset_now + interval_;
    } else {
      // We're below 0, so rounding up means towards 0.
      prerounding = offset_now + monotonic_clock::duration(1);
    }
    next_time += (prerounding / interval_) * interval_;
  }
  // Add offset_ back in.
  next_time += offset_;

  const monotonic_clock::duration difference = next_time - last_time_;

  const int result = difference / interval_;
  CHECK_EQ(
      0, (next_time - offset_).time_since_epoch().count() % interval_.count());
  CHECK(next_time > now);
  CHECK(next_time - now <= interval_);
  last_time_ = next_time;
  return result;
}

}  // namespace time
}  // namespace aos
