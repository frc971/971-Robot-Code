#include "aos/common/condition.h"

#include <inttypes.h>

#include "aos/common/type_traits.h"

namespace aos {

static_assert(shm_ok<Condition>::value,
              "Condition should work in shared memory");

Condition::Condition(Mutex *m) : impl_(), m_(m) {}

void Condition::Wait() {
  condition_wait(&impl_, &m_->impl_);
}

void Condition::Signal() {
  condition_signal(&impl_);
}

void Condition::Broadcast() {
  condition_broadcast(&impl_, &m_->impl_);
}

}  // namespace aos
