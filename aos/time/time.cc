#include "aos/time/time.h"

#include <ctype.h>
#include <errno.h>
#include <sys/time.h>

#include <algorithm>
#include <chrono>
#include <compare>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <ratio>
#include <sstream>

#ifdef __linux__

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/numbers.h"

#else  // __linux__

#include "motors/core/kinetis.h"

// The systick interrupt increments this every 1ms.
extern "C" volatile uint32_t systick_millis_count;

#endif  // __linux__

namespace chrono = ::std::chrono;

namespace {

void PrintToStream(std::ostream &stream, chrono::nanoseconds duration) {
  chrono::seconds seconds = chrono::duration_cast<chrono::seconds>(duration);
  if (duration < chrono::nanoseconds(0)) {
    stream << "-" << -seconds.count() << "." << std::setfill('0')
           << std::setw(9)
           << chrono::duration_cast<chrono::nanoseconds>(seconds - duration)
                  .count()
           << "sec";
  } else {
    stream << seconds.count() << "." << std::setfill('0') << std::setw(9)
           << chrono::duration_cast<chrono::nanoseconds>(duration - seconds)
                  .count()
           << "sec";
  }
}

}  // namespace

#ifdef __linux__

namespace aos::this_thread {

void sleep_until(const ::aos::monotonic_clock::time_point &end_time) {
  struct timespec end_time_timespec;
  ::std::chrono::seconds sec =
      ::std::chrono::duration_cast<::std::chrono::seconds>(
          end_time.time_since_epoch());
  ::std::chrono::nanoseconds nsec =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
          end_time.time_since_epoch() - sec);
  end_time_timespec.tv_sec = sec.count();
  end_time_timespec.tv_nsec = nsec.count();
  int returnval;
  do {
    returnval = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                                &end_time_timespec, nullptr);
    PCHECK(returnval == 0 || returnval == EINTR)
        << ": clock_nanosleep(" << static_cast<uintmax_t>(CLOCK_MONOTONIC)
        << ", TIMER_ABSTIME, " << &end_time_timespec << ", nullptr) failed";
  } while (returnval != 0);
}

}  // namespace aos::this_thread

#endif  // __linux__

namespace aos {

std::ostream &operator<<(std::ostream &stream,
                         const aos::monotonic_clock::time_point &now) {
  PrintToStream(stream, now.time_since_epoch());
  return stream;
}

std::string ToString(const aos::monotonic_clock::time_point &now) {
  std::ostringstream stream;
  stream << now;
  return stream.str();
}

std::string ToString(const aos::realtime_clock::time_point &now) {
  std::ostringstream stream;
  stream << now;
  return stream.str();
}

#ifdef __linux__
std::optional<monotonic_clock::time_point> monotonic_clock::FromString(
    const std::string_view now) {
  // This should undo the operator << above.
  if (now.size() < 14) {
    return std::nullopt;
  }

  if (now.substr(now.size() - 3, now.size()) != "sec") {
    return std::nullopt;
  }

  if (now[now.size() - 13] != '.') {
    return std::nullopt;
  }

  bool negative = now[0] == '-';

  std::string_view sec(
      now.substr(negative ? 1 : 0, now.size() - (negative ? 14 : 13)));
  std::string_view nsec(now.substr(now.size() - 12, 9));

  if (!std::all_of(sec.begin(), sec.end(), ::isdigit) ||
      !std::all_of(nsec.begin(), nsec.end(), ::isdigit)) {
    return std::nullopt;
  }

  std::chrono::seconds::rep seconds_data;
  if (!absl::SimpleAtoi(sec, &seconds_data)) {
    return std::nullopt;
  }

  std::chrono::nanoseconds::rep nanoseconds_data;
  if (!absl::SimpleAtoi(nsec, &nanoseconds_data)) {
    return std::nullopt;
  }

  return monotonic_clock::time_point(
      std::chrono::seconds((negative ? -1 : 1) * seconds_data) +
      std::chrono::nanoseconds((negative ? -1 : 1) * nanoseconds_data));
}

std::optional<realtime_clock::time_point> realtime_clock::FromString(
    const std::string_view now) {
  // This should undo the operator << above.

  if (now.size() < 25) {
    return std::nullopt;
  }

  if (now[now.size() - 10] != '.') {
    return std::nullopt;
  }

  std::string_view nsec(now.substr(now.size() - 9, 9));

  if (!std::all_of(nsec.begin(), nsec.end(), ::isdigit)) {
    return std::nullopt;
  }

  struct tm tm;
  std::istringstream ss(std::string(now.substr(0, now.size() - 10)));
  ss >> std::get_time(&tm, "%Y-%m-%d_%H-%M-%S");
  if (ss.fail()) {
    return std::nullopt;
  }
  tm.tm_isdst = -1;

  time_t seconds = mktime(&tm);

  std::chrono::nanoseconds::rep nanoseconds_data;
  if (!absl::SimpleAtoi(nsec, &nanoseconds_data)) {
    return std::nullopt;
  }

  return realtime_clock::time_point(std::chrono::seconds(seconds) +
                                    std::chrono::nanoseconds(nanoseconds_data));
}
#endif

std::ostream &operator<<(std::ostream &stream,
                         const aos::realtime_clock::time_point &now) {
  std::tm tm;
  std::chrono::seconds seconds =
      now < realtime_clock::epoch()
          ? (std::chrono::duration_cast<std::chrono::seconds>(
                 now.time_since_epoch() + std::chrono::nanoseconds(1)) -
             std::chrono::seconds(1))
          : std::chrono::duration_cast<std::chrono::seconds>(
                now.time_since_epoch());

  // We can run into some corner cases where the seconds value is large enough
  // to cause the conversion to nanoseconds to overflow. That is undefined
  // behaviour so we prevent it with this check here.
  if (int64_t result;
      __builtin_mul_overflow(seconds.count(), 1'000'000'000, &result)) {
    stream << "(unrepresentable realtime " << now.time_since_epoch().count()
           << ")";
    return stream;
  }

  std::time_t seconds_t = seconds.count();
  stream << std::put_time(localtime_r(&seconds_t, &tm), "%Y-%m-%d_%H-%M-%S.")
         << std::setfill('0') << std::setw(9)
         << std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch() - seconds)
                .count();
  return stream;
}

namespace time {

struct timespec to_timespec(const ::aos::monotonic_clock::duration duration) {
  struct timespec time_timespec;
  ::std::chrono::seconds sec =
      ::std::chrono::duration_cast<::std::chrono::seconds>(duration);
  ::std::chrono::nanoseconds nsec =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(duration - sec);
  time_timespec.tv_sec = sec.count();
  time_timespec.tv_nsec = nsec.count();
  return time_timespec;
}

struct timespec to_timespec(const ::aos::monotonic_clock::time_point time) {
  return to_timespec(time.time_since_epoch());
}

::aos::monotonic_clock::time_point from_timeval(struct timeval t) {
  return monotonic_clock::epoch() + std::chrono::seconds(t.tv_sec) +
         std::chrono::microseconds(t.tv_usec);
}

}  // namespace time

constexpr monotonic_clock::time_point monotonic_clock::min_time;
constexpr monotonic_clock::time_point monotonic_clock::max_time;
constexpr realtime_clock::time_point realtime_clock::min_time;
constexpr realtime_clock::time_point realtime_clock::max_time;

monotonic_clock::time_point monotonic_clock::now() noexcept {
#ifdef __linux__
  struct timespec current_time;
  PCHECK(clock_gettime(CLOCK_MONOTONIC, &current_time) == 0)
      << ": clock_gettime(" << static_cast<uintmax_t>(CLOCK_MONOTONIC) << ", "
      << &current_time << ") failed";

  return time_point(::std::chrono::seconds(current_time.tv_sec) +
                    ::std::chrono::nanoseconds(current_time.tv_nsec));

#else  // __linux__

  __disable_irq();
  const uint32_t current_counter = SYST_CVR;
  uint32_t ms_count = systick_millis_count;
  const uint32_t istatus = SCB_ICSR;
  __enable_irq();
  // If the interrupt is pending and the timer has already wrapped from 0 back
  // up to its max, then add another ms.
  if ((istatus & SCB_ICSR_PENDSTSET) && current_counter > 50) {
    ++ms_count;
  }

  // It counts down, but everything we care about counts up.
  const uint32_t counter_up = ((F_CPU / 1000) - 1) - current_counter;

  // "3.2.1.2 System Tick Timer" in the TRM says "The System Tick Timer's clock
  // source is always the core clock, FCLK".
  using systick_duration =
      std::chrono::duration<uint32_t, std::ratio<1, F_CPU>>;

  return time_point(std::chrono::round<std::chrono::nanoseconds>(
      std::chrono::milliseconds(ms_count) + systick_duration(counter_up)));

#endif  // __linux__
}

#ifdef __linux__
realtime_clock::time_point realtime_clock::now() noexcept {
  struct timespec current_time;
  PCHECK(clock_gettime(CLOCK_REALTIME, &current_time) == 0)
      << "clock_gettime(" << static_cast<uintmax_t>(CLOCK_REALTIME) << ", "
      << &current_time << ") failed";

  return time_point(::std::chrono::seconds(current_time.tv_sec) +
                    ::std::chrono::nanoseconds(current_time.tv_nsec));
}
#endif  // __linux__

}  // namespace aos
