#ifndef AOS_COMMON_UTIL_PHASED_LOOP_H_
#define AOS_COMMON_UTIL_PHASED_LOOP_H_

#include "aos/common/time.h"

#include "aos/common/logging/logging.h"

namespace aos {
namespace time {

// Will not be accurate if ms isn't a factor of 1000.
// offset is in us.
// DEPRECATED(Brian): Use PhasedLoop instead.
void PhasedLoopXMS(int ms, int offset);

// Handles sleeping until a fixed offset from some time interval.
class PhasedLoop {
 public:
  // For example, with interval = 1s and offset = 0.1s this will fire at:
  //   0.1s
  //   1.1s
  //   ...
  //   10000.1s
  // offset must be >= Time::kZero and < interval.
  PhasedLoop(const Time &interval, const Time &offset = Time::kZero)
      : interval_(interval), offset_(offset), last_time_(offset) {
    CHECK_GE(offset, Time::kZero);
    CHECK_GT(interval, Time::kZero);
    CHECK_LT(offset, interval);
    Reset();
  }

  // Resets the count of skipped iterations.
  // Iterate(now) will return 1 and set sleep_time() to something within
  // interval of now.
  void Reset(const Time &now = Time::Now()) { Iterate(now - interval_); }

  // Calculates the next time to run after now.
  // The result can be retrieved with sleep_time().
  // Returns the number of iterations which have passed (1 if this is called
  // often enough). This can be < 1 iff now goes backwards between calls.
  int Iterate(const Time &now = Time::Now());

  // Sleeps until the next time and returns the number of iterations which have
  // passed.
  int SleepUntilNext() {
    const int r = Iterate(Time::Now());
    SleepUntil(sleep_time());
    return r;
  }

  const Time &sleep_time() const { return last_time_; }

 private:
  const Time interval_, offset_;

  // The time we most recently slept until.
  Time last_time_ = Time::kZero;
};

}  // namespace time
}  // namespace aos

#endif  // AOS_COMMON_UTIL_PHASED_LOOP_H_
