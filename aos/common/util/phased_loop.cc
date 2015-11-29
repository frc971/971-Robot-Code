#include "aos/common/util/phased_loop.h"

namespace aos {
namespace time {

void PhasedLoopXMS(int ms, int offset) {
  const Time frequency = Time::InMS(ms);
  SleepUntil((Time::Now() / static_cast<int32_t>(frequency.ToNSec())) *
             static_cast<int32_t>(frequency.ToNSec()) +
             frequency + Time::InUS(offset));
}

int PhasedLoop::Iterate(const Time &now) {
  const Time next_time = Time::InNS(((now - offset_).ToNSec() + 1) /
                                    interval_.ToNSec() * interval_.ToNSec()) +
                         ((now < offset_) ? Time::kZero : interval_) + offset_;

  const Time difference = next_time - last_time_;
  const int result = difference.ToNSec() / interval_.ToNSec();
  CHECK_EQ(difference, interval_ * result);
  CHECK_EQ(0, (next_time - offset_).ToNSec() % interval_.ToNSec());
  CHECK_GE(next_time, now);
  CHECK_LE(next_time - now, interval_);
  last_time_ = next_time;
  return result;
}

}  // namespace timing
}  // namespace aos
