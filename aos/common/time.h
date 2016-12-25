#ifndef AOS_COMMON_TIME_H_
#define AOS_COMMON_TIME_H_

#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#include <stdint.h>

#include <type_traits>
#include <chrono>
#include <thread>
#include <ostream>

#include "aos/common/type_traits.h"
#include "aos/common/macros.h"

namespace aos {

class monotonic_clock {
 public:
  typedef ::std::chrono::nanoseconds::rep rep;
  typedef ::std::chrono::nanoseconds::period period;
  typedef ::std::chrono::nanoseconds duration;
  typedef ::std::chrono::time_point<monotonic_clock> time_point;

  static monotonic_clock::time_point now() noexcept;
  static constexpr bool is_steady = true;

  // Returns the epoch (0).
  static constexpr monotonic_clock::time_point epoch() {
    return time_point(zero());
  }

  static constexpr monotonic_clock::duration zero() { return duration(0); }

  static constexpr time_point min_time{
      time_point(duration(::std::numeric_limits<duration::rep>::min()))};
};

namespace time {

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

}  // namespace time
}  // namespace aos

namespace std {
namespace this_thread {
// Template specialization for monotonic_clock, since we can use clock_nanosleep
// with TIMER_ABSTIME and get very precise absolute time sleeps.
template <>
void sleep_until(const ::aos::monotonic_clock::time_point &end_time);

}  // namespace this_thread
}  // namespace std


#endif  // AOS_COMMON_TIME_H_
