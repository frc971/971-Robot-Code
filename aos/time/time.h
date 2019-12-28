#ifndef AOS_TIME_H_
#define AOS_TIME_H_

#include <stdint.h>
#include <sys/time.h>
#include <time.h>

#include <chrono>
#include <ostream>
#include <thread>
#include <type_traits>

#include "aos/macros.h"
#include "aos/type_traits/type_traits.h"

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
  static constexpr time_point max_time{
      time_point(duration(::std::numeric_limits<duration::rep>::max()))};
};

class realtime_clock {
 public:
  typedef ::std::chrono::nanoseconds::rep rep;
  typedef ::std::chrono::nanoseconds::period period;
  typedef ::std::chrono::nanoseconds duration;
  typedef ::std::chrono::time_point<realtime_clock> time_point;

#ifdef __linux__
  static realtime_clock::time_point now() noexcept;
#endif  // __linux__
  static constexpr bool is_steady = false;

  // Returns the epoch (0).
  static constexpr realtime_clock::time_point epoch() {
    return time_point(zero());
  }

  static constexpr realtime_clock::duration zero() { return duration(0); }

  static constexpr time_point min_time{
      time_point(duration(::std::numeric_limits<duration::rep>::min()))};
  static constexpr time_point max_time{
      time_point(duration(::std::numeric_limits<duration::rep>::max()))};
};

std::ostream &operator<<(std::ostream &stream,
                         const aos::monotonic_clock::time_point &now);
std::ostream &operator<<(std::ostream &stream,
                         const aos::realtime_clock::time_point &now);

namespace time {
#ifdef __linux__

// Construct a time representing the period of hertz.
constexpr ::std::chrono::nanoseconds FromRate(int hertz) {
  return ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
             ::std::chrono::seconds(1)) /
         hertz;
}

template <typename Scalar>
constexpr Scalar TypedDurationInSeconds(monotonic_clock::duration dt) {
  return ::std::chrono::duration_cast<::std::chrono::duration<Scalar>>(dt)
      .count();
}

constexpr double DurationInSeconds(monotonic_clock::duration dt) {
  return TypedDurationInSeconds<double>(dt);
}

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
