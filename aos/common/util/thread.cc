#include "aos/common/util/thread.h"

#include <pthread.h>
#include <assert.h>

namespace aos {
namespace util {

Thread::Thread() : started_(false), joined_(false), should_terminate_(false) {}

Thread::~Thread() {
  if (started_ && !joined_) {
    assert(false);
  }
}

void Thread::Start() {
  assert(!started_);
  started_ = true;
  assert(pthread_create(&thread_, NULL, &Thread::StaticRun, this) == 0);
}

void Thread::Join() {
  assert(!joined_ && started_);
  joined_ = true;
  {
    MutexLocker locker(&should_terminate_mutex_);
    should_terminate_ = true;
  }
  assert(pthread_join(thread_, NULL) == 0);
}

void *Thread::StaticRun(void *self) {
  static_cast<Thread *>(self)->Run();
  return NULL;
}

}  // namespace util
}  // namespace aos
