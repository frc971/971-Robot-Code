#ifndef AOS_TIME_H_
#define AOS_TIME_H_

#include <sys/time.h>

#include <chrono>
#include <cstdint>
#include <ctime>
#include <optional>
#include <ostream>
#include <thread>

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

  // Converts the time string to a time_point if it is well formatted.  This is
  // designed to reverse operator <<.
#ifdef __linux__
  static std::optional<monotonic_clock::time_point> FromString(
      const std::string_view now);
#endif

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

  // Converts the time string to a time_point if it is well formatted.  This is
  // designed to reverse operator <<.
#ifdef __linux__
  static std::optional<realtime_clock::time_point> FromString(
      const std::string_view now);
#endif

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

// Converts a timeval object to a monotonic_clock::time_point.
::aos::monotonic_clock::time_point from_timeval(struct timeval t);

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
