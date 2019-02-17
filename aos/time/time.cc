#include "aos/time/time.h"

#include <inttypes.h>
#include <string.h>

#include <atomic>
#include <chrono>

#ifdef __linux__

// We only use global_core from here, which is weak, so we don't really have a
// dependency on it.
#include "aos/ipc_lib/shared_mem.h"

#include "aos/logging/logging.h"
#include "aos/mutex/mutex.h"

#else  // __linux__

#include "motors/core/kinetis.h"

// The systick interrupt increments this every 1ms.
extern "C" volatile uint32_t systick_millis_count;

#endif  // __linux__

namespace chrono = ::std::chrono;

#ifdef __linux__

namespace std {
namespace this_thread {
template <>
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
    if (returnval != EINTR && returnval != 0) {
      PLOG(FATAL, "clock_nanosleep(%jd, TIMER_ABSTIME, %p, nullptr) failed",
           static_cast<uintmax_t>(CLOCK_MONOTONIC), &end_time_timespec);
    }
  } while (returnval != 0);
}

}  // namespace this_thread
}  // namespace std

#endif  // __linux__

namespace aos {
namespace time {

#ifdef __linux__

// State required to enable and use mock time.
namespace {
// True if mock time is enabled.
// This does not need to be checked with the mutex held because setting time to
// be enabled or disabled is atomic, and all future operations are atomic
// anyways.  If there is a race condition setting or clearing whether time is
// enabled or not, it will still be a race condition if current_mock_time is
// also set atomically with enabled.
::std::atomic<bool> mock_time_enabled{false};
// Mutex to make time reads and writes thread safe.
Mutex time_mutex;
// Current time when time is mocked.
monotonic_clock::time_point current_mock_time = monotonic_clock::epoch();

}  // namespace

void EnableMockTime(monotonic_clock::time_point now) {
  MutexLocker time_mutex_locker(&time_mutex);
  mock_time_enabled = true;
  current_mock_time = now;
}

void UpdateMockTime() { SetMockTime(monotonic_clock::now()); }

void DisableMockTime() {
  MutexLocker time_mutex_locker(&time_mutex);
  mock_time_enabled = false;
}

void SetMockTime(monotonic_clock::time_point now) {
  MutexLocker time_mutex_locker(&time_mutex);
  if (__builtin_expect(!mock_time_enabled, 0)) {
    LOG(FATAL, "Tried to set mock time and mock time is not enabled\n");
  }
  current_mock_time = now;
}

void IncrementMockTime(monotonic_clock::duration amount) {
  static ::aos::Mutex mutex;
  ::aos::MutexLocker sync(&mutex);
  SetMockTime(monotonic_clock::now() + amount);
}

void OffsetToNow(monotonic_clock::time_point now) {
  CHECK_NOTNULL(&global_core);
  CHECK_NOTNULL(global_core);
  CHECK_NOTNULL(global_core->mem_struct);
  const auto offset = now - monotonic_clock::now();
  global_core->mem_struct->time_offset =
      chrono::duration_cast<chrono::nanoseconds>(offset).count();
}

#endif  // __linux__

struct timespec to_timespec(
    const ::aos::monotonic_clock::duration duration) {
  struct timespec time_timespec;
  ::std::chrono::seconds sec =
      ::std::chrono::duration_cast<::std::chrono::seconds>(duration);
  ::std::chrono::nanoseconds nsec =
      ::std::chrono::duration_cast<::std::chrono::nanoseconds>(duration - sec);
  time_timespec.tv_sec = sec.count();
  time_timespec.tv_nsec = nsec.count();
  return time_timespec;
}

struct timespec to_timespec(
    const ::aos::monotonic_clock::time_point time) {
  return to_timespec(time.time_since_epoch());
}
}  // namespace time

constexpr monotonic_clock::time_point monotonic_clock::min_time;
constexpr realtime_clock::time_point realtime_clock::min_time;

monotonic_clock::time_point monotonic_clock::now() noexcept {
#ifdef __linux__

  if (time::mock_time_enabled.load(::std::memory_order_relaxed)) {
    MutexLocker time_mutex_locker(&time::time_mutex);
    return time::current_mock_time;
  }

  struct timespec current_time;
  if (clock_gettime(CLOCK_MONOTONIC, &current_time) != 0) {
    PLOG(FATAL, "clock_gettime(%jd, %p) failed",
         static_cast<uintmax_t>(CLOCK_MONOTONIC), &current_time);
  }
  const chrono::nanoseconds offset =
      (&global_core == nullptr || global_core == nullptr ||
       global_core->mem_struct == nullptr)
          ? chrono::nanoseconds(0)
          : chrono::nanoseconds(global_core->mem_struct->time_offset);

  return time_point(::std::chrono::seconds(current_time.tv_sec) +
                    ::std::chrono::nanoseconds(current_time.tv_nsec)) + offset;

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

  return time_point(aos::time::round<std::chrono::nanoseconds>(
      std::chrono::milliseconds(ms_count) + systick_duration(counter_up)));

#endif  // __linux__
}

#ifdef __linux__
realtime_clock::time_point realtime_clock::now() noexcept {
  struct timespec current_time;
  if (clock_gettime(CLOCK_REALTIME, &current_time) != 0) {
    PLOG(FATAL, "clock_gettime(%jd, %p) failed",
         static_cast<uintmax_t>(CLOCK_REALTIME), &current_time);
  }

  return time_point(::std::chrono::seconds(current_time.tv_sec) +
                    ::std::chrono::nanoseconds(current_time.tv_nsec));
}
#endif  // __linux__

}  // namespace aos
