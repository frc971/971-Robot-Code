#ifndef AOS_COMMON_UTIL_THREAD_H_
#define AOS_COMMON_UTIL_THREAD_H_

#include "aos/common/mutex.h"

namespace aos {
namespace util {

// A nice wrapper around a pthreads thread.
//
// TODO(aschuh): Test this.
class Thread {
 public:
  Thread();
  ~Thread();

  // Actually creates the thread.
  void Start();

  // Asks the code to stop and then waits until it has done so.
  void Join();

 protected:
  // Subclasses need to call this periodically if they are going to loop to
  // check whether they have been asked to stop.
  bool should_continue() {
    MutexLocker locker(&should_terminate_mutex_);
    return !should_terminate_;
  }

 private:
  // Where subclasses actually do something.
  //
  // They should not block for long periods of time without checking
  // should_continue().
  virtual void Run() = 0;

  static void *StaticRun(void *self);

  pthread_t thread_;
  bool started_;
  bool joined_;
  bool should_terminate_;
  Mutex should_terminate_mutex_;
};

}  // namespace util
}  // namespace aos

#endif  // AOS_COMMON_UTIL_THREAD_H_
