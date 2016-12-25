#include "aos/common/time.h"

#include <inttypes.h>
#include <string.h>

#include <atomic>
#include <chrono>

// We only use global_core from here, which is weak, so we don't really have a
// dependency on it.
#include "aos/linux_code/ipc_lib/shared_mem.h"

#include "aos/common/logging/logging.h"
#include "aos/common/mutex.h"

namespace chrono = ::std::chrono;

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


namespace aos {
namespace time {

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

}  // namespace time

constexpr monotonic_clock::time_point monotonic_clock::min_time;

monotonic_clock::time_point monotonic_clock::now() noexcept {
  {
    if (time::mock_time_enabled.load(::std::memory_order_relaxed)) {
      MutexLocker time_mutex_locker(&time::time_mutex);
      return time::current_mock_time;
    }
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
}


}  // namespace aos
