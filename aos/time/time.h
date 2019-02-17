#ifndef AOS_TIME_H_
#define AOS_TIME_H_

#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#include <stdint.h>

#include <type_traits>
#include <chrono>
#include <thread>
#include <ostream>

#include "aos/type_traits/type_traits.h"
#include "aos/macros.h"

namespace aos {

class monotonic_clock {
 public:
  typedef ::std::chrono::nanoseconds::rep rep;
  typedef ::std::chrono::nanoseconds::period period;
  typedef ::std::chrono::nanoseconds duration;
  typedef ::std::chrono::time_point<monotonic_clock> time_point;

  static monotonic_clock::time_point now() noexcept;
  // This clock is still subject to rate adjustments based on adjtime, so it is
  // not steady.
  static constexpr bool is_steady = false;

  // Returns the epoch (0).
  static constexpr monotonic_clock::time_point epoch() {
    return time_point(zero());
  }

  static constexpr monotonic_clock::duration zero() { return duration(0); }

  static constexpr time_point min_time{
      time_point(duration(::std::numeric_limits<duration::rep>::min()))};
};

class realtime_clock {
 public:
  typedef ::std::chrono::nanoseconds::rep rep;
  typedef ::std::chrono::nanoseconds::period period;
  typedef ::std::chrono::nanoseconds duration;
  typedef ::std::chrono::time_point<monotonic_clock> time_point;

#ifdef __linux__
  static monotonic_clock::time_point now() noexcept;
#endif  // __linux__
  static constexpr bool is_steady = false;

  // Returns the epoch (0).
  static constexpr monotonic_clock::time_point epoch() {
    return time_point(zero());
  }

  static constexpr monotonic_clock::duration zero() { return duration(0); }

  static constexpr time_point min_time{
      time_point(duration(::std::numeric_limits<duration::rep>::min()))};
};

namespace time {

#ifdef __linux__

// Enables returning the mock time value for Now instead of checking the system
// clock.
void EnableMockTime(monotonic_clock::time_point now);
// Calls SetMockTime with the current actual time.
void UpdateMockTime();
// Sets now when time is being mocked.
void SetMockTime(monotonic_clock::time_point now);
// Convenience function to just increment the mock time by a certain amount in
// a thread safe way.
void IncrementMockTime(monotonic_clock::duration amount);
// Disables mocking time.
void DisableMockTime();

// Sets the global offset for all times so monotonic_clock::now() will return
// now.
// There is no synchronization here, so this is only safe when only a single
// task is running.
// This is only allowed when the shared memory core infrastructure has been
// initialized in this process.
void OffsetToNow(const monotonic_clock::time_point now);

// Construct a time representing the period of hertz.
constexpr ::std::chrono::nanoseconds FromRate(int hertz) {
  return ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
             ::std::chrono::seconds(1)) /
         hertz;
}

// RAII class that freezes monotonic_clock::now() (to avoid making large numbers
// of syscalls to find the real time).
class TimeFreezer {
 public:
  TimeFreezer() { EnableMockTime(monotonic_clock::now()); }
  ~TimeFreezer() { DisableMockTime(); }

 private:
  DISALLOW_COPY_AND_ASSIGN(TimeFreezer);
};

#endif  // __linux__

// Converts a monotonic_clock::duration into a timespec object.
struct timespec to_timespec(::aos::monotonic_clock::duration duration);

// Converts a monotonic_clock::time_point into a timespec object as time since
// epoch.
struct timespec to_timespec(::aos::monotonic_clock::time_point time);

namespace time_internal {

template <class T>
struct is_duration : std::false_type {};
template <class Rep, class Period>
struct is_duration<std::chrono::duration<Rep, Period>> : std::true_type {};

}  // namespace time_internal

// Returns the greatest duration t representable in ToDuration that is less or
// equal to d.
// Implementation copied from
// https://en.cppreference.com/w/cpp/chrono/duration/floor.
// TODO(Brian): Remove once we have C++17 support.
template <class To, class Rep, class Period,
          class = std::enable_if_t<time_internal::is_duration<To>{}>>
constexpr To floor(const std::chrono::duration<Rep, Period> &d) {
  To t = std::chrono::duration_cast<To>(d);
  if (t > d) return t - To{1};
  return t;
}

// Returns the value t representable in ToDuration that is the closest to d. If
// there are two such values, returns the even value (that is, the value t such
// that t % 2 == 0).
// Implementation copied from
// https://en.cppreference.com/w/cpp/chrono/duration/round.
// TODO(Brian): Remove once we have C++17 support.
template <class To, class Rep, class Period,
          class = std::enable_if_t<
              time_internal::is_duration<To>{} &&
              !std::chrono::treat_as_floating_point<typename To::rep>{}>>
constexpr To round(const std::chrono::duration<Rep, Period> &d) {
  To t0 = aos::time::floor<To>(d);
  To t1 = t0 + To{1};
  auto diff0 = d - t0;
  auto diff1 = t1 - d;
  if (diff0 == diff1) {
    if (t0.count() & 1) return t1;
    return t0;
  } else if (diff0 < diff1) {
    return t0;
  }
  return t1;
}

// Returns the nearest time point to tp representable in ToDuration, rounding to
// even in halfway cases, like std::chrono::round in C++17.
// Implementation copied from
// https://en.cppreference.com/w/cpp/chrono/time_point/round.
// TODO(Brian): Remove once we have C++17 support.
template <class To, class Clock, class FromDuration,
          class = std::enable_if_t<
              time_internal::is_duration<To>{} &&
              !std::chrono::treat_as_floating_point<typename To::rep>{}>>
constexpr std::chrono::time_point<Clock, To> round(
    const std::chrono::time_point<Clock, FromDuration> &tp) {
  return std::chrono::time_point<Clock, To>{
      aos::time::round<To>(tp.time_since_epoch())};
}

}  // namespace time
}  // namespace aos

#ifdef __linux__

namespace std {
namespace this_thread {
// Template specialization for monotonic_clock, since we can use clock_nanosleep
// with TIMER_ABSTIME and get very precise absolute time sleeps.
template <>
void sleep_until(const ::aos::monotonic_clock::time_point &end_time);

}  // namespace this_thread
}  // namespace std

#endif  // __linux__

#endif  // AOS_TIME_H_
