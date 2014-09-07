#include "aos/common/util/thread.h"

#include <pthread.h>

#include "aos/common/logging/logging.h"

namespace aos {
namespace util {

Thread::Thread() : started_(false), joined_(false), should_terminate_(false) {}

Thread::~Thread() {
  if (started_ && !joined_) {
    CHECK(false);
  }
}

void Thread::Start() {
  CHECK(!started_);
  started_ = true;
  CHECK(pthread_create(&thread_, NULL, &Thread::StaticRun, this) == 0);
}

void Thread::Join() {
  CHECK(!joined_ && started_);
  joined_ = true;
  should_terminate_.store(true);
  CHECK(pthread_join(thread_, NULL) == 0);
}

void Thread::WaitUntilDone() {
  CHECK(!joined_ && started_);
  joined_ = true;
  CHECK(pthread_join(thread_, NULL) == 0);
}

void *Thread::StaticRun(void *self) {
  static_cast<Thread *>(self)->Run();
  return NULL;
}

}  // namespace util
}  // namespace aos
