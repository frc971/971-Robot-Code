#include "aos/common/time.h"

#include <atomic>

#include <string.h>
#include <inttypes.h>

// We only use global_core from here, which is weak, so we don't really have a
// dependency on it.
#include "aos/linux_code/ipc_lib/shared_mem.h"

#include "aos/common/logging/logging.h"
#include "aos/common/mutex.h"

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

// TODO(aschuh): This doesn't include SleepFor and SleepUntil.
// TODO(aschuh): Create a clock source object and change the default?
//  That would let me create a MockTime clock source.

Time NowImpl(clockid_t clock) {
  timespec temp;
  if (clock_gettime(clock, &temp) != 0) {
    PLOG(FATAL, "clock_gettime(%jd, %p) failed",
         static_cast<uintmax_t>(clock), &temp);
  }

  const timespec offset = (&global_core == nullptr || global_core == nullptr ||
                           global_core->mem_struct == nullptr)
                              ? timespec{0, 0}
                              : global_core->mem_struct->time_offset;
  return Time(temp) + Time(offset);
}

}  // namespace

const int32_t Time::kNSecInSec;
const int32_t Time::kNSecInMSec;
const int32_t Time::kNSecInUSec;
const int32_t Time::kMSecInSec;
const int32_t Time::kUSecInSec;

const Time Time::kZero{0, 0};

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

Time Time::Now(clockid_t clock) {
  {
    if (mock_time_enabled.load(::std::memory_order_relaxed)) {
      MutexLocker time_mutex_locker(&time_mutex);
      return Time::InNS(
          ::std::chrono::duration_cast<::std::chrono::nanoseconds>(
              current_mock_time.time_since_epoch()).count());
    }
  }
  return NowImpl(clock);
}

void Time::CheckImpl(int32_t nsec) {
  static_assert(aos::shm_ok<Time>::value,
                "it should be able to go through shared memory");
  if (nsec >= kNSecInSec || nsec < 0) {
    LOG(FATAL, "0 <= nsec(%" PRId32 ") < %" PRId32 " isn't true.\n",
        nsec, kNSecInSec);
  }
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
  if (nsec_ < 0) {
    nsec_ += kNSecInSec;
    sec_ -= 1;
  }
  return *this;
}
const Time Time::operator*(int32_t rhs) const {
  return Time(*this) *= rhs;
}
Time &Time::operator/=(int32_t rhs) {
  nsec_ = (sec_ % rhs) * (kNSecInSec / rhs) + nsec_ / rhs;
  sec_ /= rhs;
  if (nsec_ < 0) {
    nsec_ += kNSecInSec;
    sec_ -= 1;
  }
  return *this;
}
const Time Time::operator/(int32_t rhs) const {
  return Time(*this) /= rhs;
}
double Time::operator/(const Time &rhs) const {
  return ToSeconds() / rhs.ToSeconds();
}
Time &Time::operator%=(int32_t rhs) {
  nsec_ = ToNSec() % rhs;
  const int wraps = nsec_ / ((rhs / kNSecInSec + 1) * kNSecInSec);
  sec_ = wraps + rhs / kNSecInSec;
  nsec_ -= kNSecInSec * wraps;
  if (nsec_ < 0) {
    nsec_ += kNSecInSec;
    sec_ -= 1;
  }
  return *this;
}
const Time Time::operator%(int32_t rhs) const {
  return Time(*this) %= rhs;
}

const Time Time::operator-() const {
  return Time(-sec_ - 1, kNSecInSec - nsec_);
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
  timespec converted(time.ToTimespec()), remaining;
  int failure = EINTR;
  do {
    // This checks whether the last time through the loop actually failed or got
    // interrupted.
    if (failure != EINTR) {
      PELOG(FATAL, failure, "clock_nanosleep(%jd, 0, %p, %p) failed",
            static_cast<intmax_t>(clock), &converted, &remaining);
    }
    failure = clock_nanosleep(clock, 0, &converted, &remaining);
    memcpy(&converted, &remaining, sizeof(converted));
  } while (failure != 0);
}

void SleepUntil(const Time &time, clockid_t clock) {
  timespec converted(time.ToTimespec());
  int failure;
  while ((failure = clock_nanosleep(clock, TIMER_ABSTIME,
                                    &converted, NULL)) != 0) {
    if (failure != EINTR) {
      PELOG(FATAL, failure, "clock_nanosleep(%jd, TIMER_ABSTIME, %p, NULL)"
            " failed", static_cast<intmax_t>(clock), &converted);
    }
  }
}

void OffsetToNow(const Time &now) {
  CHECK_NOTNULL(&global_core);
  CHECK_NOTNULL(global_core);
  CHECK_NOTNULL(global_core->mem_struct);
  global_core->mem_struct->time_offset.tv_nsec = 0;
  global_core->mem_struct->time_offset.tv_sec = 0;
  global_core->mem_struct->time_offset = (now - Time::Now()).ToTimespec();
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
  return time_point(::std::chrono::seconds(current_time.tv_sec) +
                    ::std::chrono::nanoseconds(current_time.tv_nsec));
}


}  // namespace aos
