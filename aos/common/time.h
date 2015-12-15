#ifndef AOS_COMMON_TIME_H_
#define AOS_COMMON_TIME_H_

#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#include <stdint.h>

#include <type_traits>
#include <ostream>

#include "aos/common/type_traits.h"
#include "aos/common/macros.h"

namespace aos {
namespace time {

// A nice structure for representing times.
// 0 <= nsec_ < kNSecInSec should always be true. All functions here will make
// sure that that is true if it was on all inputs (including *this).
//
// Negative times are supported so that all of the normal arithmetic identities
// work. nsec_ is still always positive.
//
// The arithmetic and comparison operators are overloaded because they make
// complete sense and are very useful. The default copy and assignment stuff is
// left because it works fine. Multiplication of Times by Times is
// not implemented because I can't think of any uses for them and there are
// multiple ways to do it. Division of Times by Times is implemented as the
// ratio of them. Multiplication, division, and modulus of Times by integers are
// implemented as interpreting the argument as nanoseconds. Modulus takes the
// sign from the first operand.
struct Time {
#ifdef SWIG
// All of the uses of constexpr here can safely be simply removed.
// NOTE: This means that relying on the fact that constexpr implicitly makes
// member functions const is not valid, so they all have to be explicitly marked
// const.
#define constexpr
#endif  // SWIG
 public:
  static const int32_t kNSecInSec = 1000000000;
  static const int32_t kNSecInMSec = 1000000;
  static const int32_t kNSecInUSec = 1000;
  static const int32_t kMSecInSec = 1000;
  static const int32_t kUSecInSec = 1000000;

  static const Time kZero;

  explicit constexpr Time(int32_t sec = 0, int32_t nsec = 0)
      : sec_(sec), nsec_(CheckConstexpr(nsec)) {
  }
  #ifndef SWIG
  explicit constexpr Time(const struct timespec &value)
      : sec_(value.tv_sec), nsec_(CheckConstexpr(value.tv_nsec)) {
  }
  struct timespec ToTimespec() const {
    struct timespec ans;
    ans.tv_sec = sec_;
    ans.tv_nsec = nsec_;
    return ans;
  }
  explicit constexpr Time(const struct timeval &value)
      : sec_(value.tv_sec), nsec_(CheckConstexpr(value.tv_usec * kNSecInUSec)) {
  }
  struct timeval ToTimeval() const {
    struct timeval ans;
    ans.tv_sec = sec_;
    ans.tv_usec = nsec_ / kNSecInUSec;
    return ans;
  }
  #endif  // SWIG

  // CLOCK_MONOTONIC on linux and CLOCK_REALTIME on the cRIO because the
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
  static constexpr Time InSeconds(double seconds) {
    return (seconds < 0.0) ?
        Time(static_cast<int32_t>(seconds) - 1,
             (seconds - static_cast<int32_t>(seconds) + 1.0) * kNSecInSec) :
        Time(static_cast<int32_t>(seconds),
             (seconds - static_cast<int32_t>(seconds)) * kNSecInSec);
  }

  // Constructs a time representing microseconds.
  static constexpr Time InNS(int64_t nseconds) {
    return (nseconds < 0)
               ? Time((nseconds - 1) / static_cast<int64_t>(kNSecInSec) - 1,
                      (((nseconds - 1) % kNSecInSec) + 1) + kNSecInSec)
               : Time(nseconds / static_cast<int64_t>(kNSecInSec),
                      nseconds % kNSecInSec);
  }

  // Constructs a time representing microseconds.
  static constexpr Time InUS(int useconds) {
    return (useconds < 0)
               ? Time((useconds + 1) / kUSecInSec - 1,
                      (((useconds + 1) % kUSecInSec) - 1) * kNSecInUSec +
                          kNSecInSec)
               : Time(useconds / kUSecInSec,
                      (useconds % kUSecInSec) * kNSecInUSec);
  }

  // Constructs a time representing mseconds.
  static constexpr Time InMS(int mseconds) {
    return (mseconds < 0)
               ? Time((mseconds + 1) / kMSecInSec - 1,
                      (((mseconds + 1) % kMSecInSec) - 1) * kNSecInMSec +
                          kNSecInSec)
               : Time(mseconds / kMSecInSec,
                      (mseconds % kMSecInSec) * kNSecInMSec);
  }

  // Construct a time representing the period of hertz.
  static constexpr Time FromRate(int hertz) {
    return Time(0, kNSecInSec / hertz);
  }

  // Checks whether or not this time is within amount nanoseconds of other.
  bool IsWithin(const Time &other, int64_t amount) const;

  // Returns the time represented all in nanoseconds.
  int64_t constexpr ToNSec() const {
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
  static Time InTicks(int ticks) {
    return Time::InSeconds(static_cast<double>(ticks) / sysClkRateGet());
  }
#endif

  // Returns the time represented in milliseconds.
  int64_t constexpr ToMSec() const {
    return static_cast<int64_t>(sec_) * static_cast<int64_t>(kMSecInSec) +
        (static_cast<int64_t>(nsec_) / static_cast<int64_t>(kNSecInMSec));
  }

  // Returns the time represent in microseconds.
  int64_t constexpr ToUSec() const {
    return static_cast<int64_t>(sec_) * static_cast<int64_t>(kUSecInSec) +
        (static_cast<int64_t>(nsec_) / static_cast<int64_t>(kNSecInUSec));
  }

  // Returns the time represented in fractional seconds.
  double constexpr ToSeconds() const {
    return static_cast<double>(sec_) + static_cast<double>(nsec_) / kNSecInSec;
  }

  #ifndef SWIG
  Time &operator+=(const Time &rhs);
  Time &operator-=(const Time &rhs);
  Time &operator*=(int32_t rhs);
  Time &operator/=(int32_t rhs);
  Time &operator%=(int32_t rhs);
  Time &operator%=(double rhs) = delete;
  Time &operator*=(double rhs) = delete;
  Time &operator/=(double rhs) = delete;
  const Time operator*(double rhs) const = delete;
  const Time operator/(double rhs) const = delete;
  const Time operator%(double rhs) const = delete;
  #endif
  const Time operator+(const Time &rhs) const;
  const Time operator-(const Time &rhs) const;
  const Time operator*(int32_t rhs) const;
  const Time operator/(int32_t rhs) const;
  double operator/(const Time &rhs) const;
  const Time operator%(int32_t rhs) const;

  const Time operator-() const;

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

  int32_t constexpr sec() const { return sec_; }
  void set_sec(int32_t sec) { sec_ = sec; }
  int32_t constexpr nsec() const { return nsec_; }
  void set_nsec(int32_t nsec) {
    nsec_ = nsec;
    Check();
  }

  // Absolute value.
  Time abs() const {
    if (*this > Time(0, 0)) return *this;
    if (nsec_ == 0) return Time(-sec_, 0);
    return Time(-sec_ - 1, kNSecInSec - nsec_);
  }

  // Enables returning the mock time value for Now instead of checking the
  // system clock.
  static void EnableMockTime(const Time &now = Now());
  // Calls SetMockTime with the current actual time.
  static void UpdateMockTime();
  // Sets now when time is being mocked.
  static void SetMockTime(const Time &now);
  // Convenience function to just increment the mock time by a certain amount in
  // a thread safe way.
  static void IncrementMockTime(const Time &amount);
  // Disables mocking time.
  static void DisableMockTime();

 private:
  int32_t sec_, nsec_;

  // LOG(FATAL)s if nsec is >= kNSecInSec or negative.
  static void CheckImpl(int32_t nsec);
  void Check() { CheckImpl(nsec_); }
  // A constexpr version of CheckImpl that returns the given value when it
  // succeeds or evaluates to non-constexpr and returns 0 when it fails.
  // This will result in the usual LOG(FATAL) if this is used where it isn't
  // required to be constexpr or a compile error if it is.
  static constexpr int32_t CheckConstexpr(int32_t nsec) {
    return (nsec >= kNSecInSec || nsec < 0) ? CheckImpl(nsec), 0 : nsec;
  }

#ifdef SWIG
#undef constexpr
#endif  // SWIG
};

// Sleeps for the amount of time represented by time counted by clock.
void SleepFor(const Time &time, clockid_t clock = Time::kDefaultClock);
// Sleeps until clock is at the time represented by time.
void SleepUntil(const Time &time, clockid_t clock = Time::kDefaultClock);

// Sets the global offset for all times so ::aos::time::Time::Now() will return
// now.
// There is no synchronization here, so this is only safe when only a single
// task is running.
// This is only allowed when the shared memory core infrastructure has been
// initialized in this process.
void OffsetToNow(const Time &now);

// RAII class that freezes Time::Now() (to avoid making large numbers of
// syscalls to find the real time).
class TimeFreezer {
 public:
  TimeFreezer() {
    Time::EnableMockTime();
  }
  ~TimeFreezer() {
    Time::DisableMockTime();
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(TimeFreezer);
};

}  // namespace time
}  // namespace aos

#endif  // AOS_COMMON_TIME_H_
