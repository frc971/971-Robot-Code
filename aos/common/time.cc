#include "aos/common/time.h"

#ifdef __VXWORKS__
#include <taskLib.h>
#endif

#include "aos/common/logging/logging.h"
#include "aos/common/inttypes.h"

namespace aos {
namespace time {

Time Time::Now(clockid_t clock) {
  timespec temp;
  if (clock_gettime(clock, &temp) != 0) {
    // TODO(aschuh): There needs to be a pluggable low level logging interface
    // so we can break this dependency loop.  This also would help during
    // startup.
    LOG(FATAL, "clock_gettime(%jd, %p) failed with %d: %s\n",
        static_cast<uintmax_t>(clock), &temp, errno, strerror(errno));
  }
  return Time(temp);
}
void Time::Check() {
  if (nsec_ >= kNSecInSec || nsec_ < 0) {
    LOG(FATAL, "0 <= nsec_(%"PRId32") < %"PRId32" isn't true.\n",
        nsec_, kNSecInSec);
  }
  static_assert(aos::shm_ok<Time>::value,
                "it should be able to go through shared memory");
}

Time &Time::operator+=(const Time &rhs) {
  sec_ += rhs.sec_;
  nsec_ += rhs.nsec_;
  if (nsec_ >= kNSecInSec) {
    nsec_ -= kNSecInSec;
    sec_ += 1;
  }
  return *this;
}
const Time Time::operator+(const Time &rhs) const {
  return Time(*this) += rhs;
}
Time &Time::operator-=(const Time &rhs) {
  sec_ -= rhs.sec_;
  nsec_ -= rhs.nsec_;
  if (nsec_ < 0) {
    nsec_ += kNSecInSec;
    sec_ -= 1;
  }
  return *this;
}
const Time Time::operator-(const Time &rhs) const {
  return Time(*this) -= rhs;
}
Time &Time::operator*=(int32_t rhs) {
  const int64_t temp = static_cast<int64_t>(nsec_) *
      static_cast<int64_t>(rhs);
  sec_ *= rhs;  // better not overflow, or the result is just too big
  nsec_ = temp % kNSecInSec;
  sec_ += (temp - nsec_) / kNSecInSec;
  return *this;
}
const Time Time::operator*(int32_t rhs) const {
  return Time(*this) *= rhs;
}
Time &Time::operator/=(int32_t rhs) {
  nsec_ = (sec_ % rhs) * (kNSecInSec / rhs) + nsec_ / rhs;
  sec_ /= rhs;
  return *this;
}
const Time Time::operator/(int32_t rhs) const {
  return Time(*this) /= rhs;
}
Time &Time::operator%=(int32_t rhs) {
  nsec_ = ToNSec() % rhs;
  const int wraps = nsec_ / kNSecInSec;
  sec_ = wraps;
  nsec_ -= kNSecInSec * wraps;
  return *this;
}
const Time Time::operator%(int32_t rhs) const {
  return Time(*this) %= rhs;
}

bool Time::operator==(const Time &rhs) const {
  return sec_ == rhs.sec_ && nsec_ == rhs.nsec_;
}
bool Time::operator!=(const Time &rhs) const {
  return !(*this == rhs);
}
bool Time::operator<(const Time &rhs) const {
  return sec_ < rhs.sec_ || (sec_ == rhs.sec_ && nsec_ < rhs.nsec_);
}
bool Time::operator>(const Time &rhs) const {
  return sec_ > rhs.sec_ || (sec_ == rhs.sec_ && nsec_ > rhs.nsec_);
}
bool Time::operator<=(const Time &rhs) const {
  return sec_ < rhs.sec_ || (sec_ == rhs.sec_ && nsec_ <= rhs.nsec_);
}
bool Time::operator>=(const Time &rhs) const {
  return sec_ > rhs.sec_ || (sec_ == rhs.sec_ && nsec_ >= rhs.nsec_);
}

bool Time::IsWithin(const Time &other, int64_t amount) const {
  const int64_t temp = ToNSec() - other.ToNSec();
  return temp <= amount && temp >= -amount;
}

std::ostream &operator<<(std::ostream &os, const Time& time) {
  return os << "Time{" << time.sec_ << "s, " << time.nsec_ << "ns}";
}

void SleepFor(const Time &time, clockid_t clock) {
#ifdef __VXWORKS__
  SleepUntil(Time::Now(clock) + time, clock);
#else
  timespec converted(time.ToTimespec()), remaining;
  int failure = EINTR;
  do {
    // This checks whether the last time through the loop actually failed or got
    // interrupted.
    if (failure != EINTR) {
      LOG(FATAL, "clock_nanosleep(%jd, 0, %p, %p) returned %d: %s\n",
          static_cast<intmax_t>(clock), &converted, &remaining,
          failure, strerror(failure));
    }
    failure = clock_nanosleep(clock, 0, &converted, &remaining);
    memcpy(&converted, &remaining, sizeof(converted));
  } while (failure != 0);
#endif
}

void SleepUntil(const Time &time, clockid_t clock) {
#ifdef __VXWORKS__
  if (clock != CLOCK_REALTIME) {
    LOG(FATAL, "vxworks only supports CLOCK_REALTIME\n");
  }
  // Vxworks nanosleep is definitely broken (fails horribly at doing remaining
  // right), and I don't really want to know how else it's broken, so I'm using
  // taskDelay instead because that's simpler.
  // The +1 is because sleep functions are supposed to sleep for at least the
  // requested amount, so we have to round up to the next clock tick.
  while (taskDelay((time - Time::Now(clock)).ToTicks() + 1) != 0) {
    if (errno != EINTR) {
      LOG(FATAL, "taskDelay(some ticks) failed with %d: %s\n",
          errno, strerror(errno));
    }
  }
#else
  timespec converted(time.ToTimespec());
  int failure;
  while ((failure = clock_nanosleep(clock, TIMER_ABSTIME,
                                    &converted, NULL)) != 0) {
    if (failure != EINTR) {
      LOG(FATAL, "clock_nanosleep(%jd, TIMER_ABSTIME, %p, NULL)"
          " returned %d: %s\n", static_cast<intmax_t>(clock), &converted,
          failure, strerror(failure));
    }
  }
#endif
}

}  // namespace time
}  // namespace aos
