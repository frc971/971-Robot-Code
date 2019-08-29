#include "aos/condition.h"

#include <assert.h>
#include <inttypes.h>
#include <time.h>

#include "aos/mutex/mutex.h"
#include "aos/type_traits/type_traits.h"
#include "glog/logging.h"

namespace aos {

namespace chrono = ::std::chrono;

static_assert(shm_ok<Condition>::value,
              "Condition should work in shared memory");

Condition::Condition(Mutex *m) : impl_(), m_(m) {}

bool Condition::Wait() {
  const int ret = condition_wait(&impl_, &m_->impl_, nullptr);
  assert(__builtin_expect(ret == 0 || ret == 1, 1));
  return ret == 1;
}

Condition::WaitResult Condition::WaitTimed(chrono::nanoseconds timeout) {
  struct timespec end_time;
  const bool do_timeout = timeout != chrono::nanoseconds(0);

  if (do_timeout) {
    PCHECK(clock_gettime(CLOCK_MONOTONIC, &end_time) == 0);
    timeout += chrono::nanoseconds(end_time.tv_nsec);
    chrono::seconds timeout_seconds =
        chrono::duration_cast<chrono::seconds>(timeout);
    end_time.tv_sec += timeout_seconds.count();
    end_time.tv_nsec = (timeout - timeout_seconds).count();
  }

  const int ret =
      condition_wait(&impl_, &m_->impl_, do_timeout ? &end_time : nullptr);
  assert(__builtin_expect(ret == 0 || ret == 1 || ret == -1, 1));
  switch (ret) {
    case 0:
      return WaitResult::kOk;
    case 1:
      return WaitResult::kOwnerDied;
    default:
      return WaitResult::kTimeout;
  }
}

void Condition::Signal() { condition_signal(&impl_, &m_->impl_); }

void Condition::Broadcast() {
  condition_broadcast(&impl_, &m_->impl_);
}

}  // namespace aos
