#ifndef AOS_COMMON_TIME_H_
#define AOS_COMMON_TIME_H_

#include <stdint.h>
#include <time.h>

#ifndef __VXWORKS__
#include <type_traits>
#include <sys/time.h>
#else
#include <sysLib.h>
#include <sys/times.h>
#endif
#include <ostream>

#include "aos/aos_stdint.h"
#include "aos/common/type_traits.h"

namespace aos {
namespace time {

// A nice structure for representing times.
// 0 <= nsec_ < kNSecInSec should always be true. All functions here will make
// sure that that is true if it was on all inputs (including *this).
//
// The arithmetic and comparison operators are overloaded because they make
// complete sense and are very useful. The default copy and assignment stuff is
// left because it works fine. Multiplication and division of Times by Times are
// not implemented because I can't think of any uses for them and there are
// multiple ways to do it.
struct Time {
 public:
  static const int32_t kNSecInSec = 1000000000;
  static const int32_t kNSecInMSec = 1000000;
  static const int32_t kNSecInUSec = 1000;
  static const int32_t kMSecInSec = 1000;
  static const int32_t kUSecInSec = 1000000;
  Time(int32_t sec, int32_t nsec) : sec_(sec), nsec_(nsec) {
    Check();
  }
  #ifndef SWIG
  explicit Time(const struct timespec &value)
      : sec_(value.tv_sec), nsec_(value.tv_nsec) {
    Check();
  }
  struct timespec ToTimespec() const {
    struct timespec ans;
    ans.tv_sec = sec_;
    ans.tv_nsec = nsec_;
    return ans;
  }
  explicit Time(const struct timeval &value)
      : sec_(value.tv_sec), nsec_(value.tv_usec * kNSecInUSec) {
    Check();
  }
  struct timeval ToTimeval() const {
    struct timeval ans;
    ans.tv_sec = sec_;
    ans.tv_usec = nsec_ / kNSecInUSec;
    return ans;
  }
  #endif  // SWIG

  // CLOCK_MONOTONIC on the fitpc and CLOCK_REALTIME on the cRIO because the
  // cRIO doesn't have any others.
  // CLOCK_REALTIME is the default realtime clock and CLOCK_MONOTONIC doesn't
  // change when somebody changes the wall clock (like the ntp deamon or
  // whatever). See clock_gettime(2) for details.
  //
  // This is the clock that code that just wants to sleep for a certain amount
  // of time or measure how long something takes should use.
  #ifndef __VXWORKS__
  static const clockid_t kDefaultClock = CLOCK_MONOTONIC;
  #else
  static const clockid_t kDefaultClock = CLOCK_REALTIME;
  #endif
  // Creates a Time representing the current value of the specified clock or
  // dies.
  static Time Now(clockid_t clock = kDefaultClock);

  // Constructs a Time representing seconds.
  static Time InSeconds(double seconds) {
    return Time(static_cast<int32_t>(seconds),
                (seconds - static_cast<int32_t>(seconds)) * kNSecInSec);
  }

  // Constructs a time representing microseconds.
  static Time InNS(int64_t nseconds) {
    return Time(nseconds / static_cast<int64_t>(kNSecInSec),
                nseconds % kNSecInSec);
  }

  // Constructs a time representing microseconds.
  static Time InUS(int useconds) {
    return Time(useconds / kUSecInSec, (useconds % kUSecInSec) * kNSecInUSec);
  }

  // Constructs a time representing mseconds.
  static Time InMS(int mseconds) {
    return Time(mseconds / kMSecInSec, (mseconds % kMSecInSec) * kNSecInMSec);
  }

  // Checks whether or not this time is within amount nanoseconds of other.
  bool IsWithin(const Time &other, int64_t amount) const;

  // Returns the time represented all in nanoseconds.
  int64_t ToNSec() const {
    return static_cast<int64_t>(sec_) * static_cast<int64_t>(kNSecInSec) +
        static_cast<int64_t>(nsec_);
  }
#ifdef __VXWORKS__
  // Returns the time represented all in system clock ticks. The system clock
  // rate is retrieved using sysClkRateGet().
  int ToTicks() const {
    return ToNSec() / static_cast<int64_t>(kNSecInSec / sysClkRateGet());
  }
  // Constructs a Time representing ticks.
  // TODO(brians): test this one too
  static Time InTicks(int ticks) {
    return Time::InSeconds(static_cast<double>(ticks) / sysClkRateGet());
  }
#endif

  // Returns the time represented in milliseconds.
  int64_t ToMSec() const {
    return static_cast<int64_t>(sec_) * static_cast<int64_t>(kMSecInSec) +
        (static_cast<int64_t>(nsec_) / static_cast<int64_t>(kNSecInMSec));
  }

  // Returns the time represented in fractional seconds.
  double ToSeconds() const {
    return static_cast<double>(sec_) + static_cast<double>(nsec_) / kNSecInSec;
  }

  #ifndef SWIG
  Time &operator+=(const Time &rhs);
  Time &operator-=(const Time &rhs);
  Time &operator*=(int32_t rhs);
  Time &operator/=(int32_t rhs);
  Time &operator%=(int32_t rhs);
  #endif
  const Time operator+(const Time &rhs) const;
  const Time operator-(const Time &rhs) const;
  const Time operator*(int32_t rhs) const;
  const Time operator/(int32_t rhs) const;
  const Time operator%(int32_t rhs) const;

  bool operator==(const Time &rhs) const;
  bool operator!=(const Time &rhs) const;
  bool operator<(const Time &rhs) const;
  bool operator>(const Time &rhs) const;
  bool operator<=(const Time &rhs) const;
  bool operator>=(const Time &rhs) const;

  #ifndef SWIG
  // For gtest etc.
  friend std::ostream &operator<<(std::ostream &os, const Time &time);
  #endif  // SWIG

  int32_t sec() const { return sec_; }
  void set_sec(int32_t sec) { sec_ = sec; }
  int32_t nsec() const { return nsec_; }
  void set_nsec(int32_t nsec) {
    nsec_ = nsec;
    Check();
  }

  // Absolute value.
  Time abs() const {
    if (*this > Time(0, 0)) return *this;
    return Time(-sec_ - 1, kNSecInSec - nsec_);
  }

  // Enables returning the mock time value for Now instead of checking the
  // system clock.  This should only be used when testing things depending on
  // time, or many things may/will break.
  static void EnableMockTime(const Time now);
  // Sets now when time is being mocked.
  static void SetMockTime(const Time now);
  // Convenience function to just increment the mock time by a certain amount.
  static void IncrementMockTime(const Time amount) {
    SetMockTime(Now() + amount);
  }
  // Disables mocking time.
  static void DisableMockTime();

 private:
  int32_t sec_, nsec_;
  // LOG(FATAL)s if nsec_ is >= kNSecInSec.
  void Check();
};

// Sleeps for the amount of time represented by time counted by clock.
void SleepFor(const Time &time, clockid_t clock = Time::kDefaultClock);
// Sleeps until clock is at the time represented by time.
void SleepUntil(const Time &time, clockid_t clock = Time::kDefaultClock);

}  // namespace time
}  // namespace aos

#endif  // AOS_COMMON_TIME_H_
