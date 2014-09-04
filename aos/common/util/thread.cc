#include "aos/common/util/thread.h"

#include <pthread.h>
#include <signal.h>

#include "aos/common/logging/logging.h"

namespace aos {
namespace util {

Thread::Thread() : started_(false), joined_(false), should_terminate_(false) {}

Thread::~Thread() {
  CHECK(!(started_ && !joined_));
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

bool Thread::TryJoin() {
  CHECK(!joined_ && started_);
#ifdef AOS_SANITIZER_thread
  // ThreadSanitizer misses the tryjoin and then complains about leaking the
  // thread. Instead, we'll just check if the thread is still around and then
  // do a regular Join() iff it isn't.
  // TODO(brians): Remove this once tsan learns about pthread_tryjoin_np.
  const int kill_ret = pthread_kill(thread_, 0);
  // If it's still around.
  if (kill_ret == 0) return false;
  // If it died, we'll get ESRCH. Otherwise, something went wrong.
  if (kill_ret != ESRCH) {
    PELOG(FATAL, kill_ret, "pthread_kill(thread_, 0) failed");
  }
  Join();
  return true;
#else
  const int ret = pthread_tryjoin_np(thread_, nullptr);
  if (ret == 0) {
    joined_ = true;
    return true;
  } else if (ret == EBUSY) {
    return false;
  } else {
    PELOG(FATAL, ret, "pthread_tryjoin_np(thread_, nullptr) failed");
  }
#endif
}

void Thread::RequestStop() {
  CHECK(!joined_ && started_);
  should_terminate_.store(true);
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
