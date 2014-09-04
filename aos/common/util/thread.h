#ifndef AOS_COMMON_UTIL_THREAD_H_
#define AOS_COMMON_UTIL_THREAD_H_

#include <functional>
#include <atomic>

#include <pthread.h>

#include "aos/common/macros.h"

namespace aos {
namespace util {

// A nice wrapper around a pthreads thread.
//
// TODO(aschuh): Test this.
class Thread {
 public:
  Thread();
  virtual ~Thread();

  // Actually creates the thread.
  void Start();

  // Asks the code to stop and then waits until it has done so.
  // This or TryJoin() (returning true) must be called exactly once for every
  // instance.
  void Join();

  // If the code has already finished, returns true. Does not block waiting if
  // it isn't.
  // Join() must not be called on this instance if this returns true.
  // This must return true or Join() must be called exactly once for every
  // instance.
  bool TryJoin();

  // Asks the code to stop (in preparation for a Join()).
  void RequestStop();

  // Waits until the code has stopped. Does not ask it to do so.
  void WaitUntilDone();

 protected:
  // Subclasses need to call this periodically if they are going to loop to
  // check whether they have been asked to stop.
  bool should_continue() {
    return !should_terminate_.load();
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
  ::std::atomic_bool should_terminate_;

  DISALLOW_COPY_AND_ASSIGN(Thread);
};

class FunctionThread : public Thread {
 public:
  FunctionThread(::std::function<void(FunctionThread *)> function)
      : function_(function) {}

  // Runs function in a new thread and waits for it to return.
  static void RunInOtherThread(::std::function<void()> function) {
    FunctionThread t([&function](FunctionThread *) { function(); });
    t.Start();
    t.Join();
  }

 private:
  virtual void Run() override {
    function_(this);
  }

  const ::std::function<void(FunctionThread *)> function_;
};

}  // namespace util
}  // namespace aos

#endif  // AOS_COMMON_UTIL_THREAD_H_
