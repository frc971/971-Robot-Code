#ifndef AOS_UTIL_THREADED_CONSUMER_H_
#define AOS_UTIL_THREADED_CONSUMER_H_

#include <functional>
#include <optional>
#include <thread>

#include "aos/condition.h"
#include "aos/containers/ring_buffer.h"
#include "aos/mutex/mutex.h"
#include "aos/realtime.h"

namespace aos {
namespace util {

// This class implements a threadpool of a single worker that accepts work
// from the main thread through a queue and executes it at a different realtime
// priority.
//
// There is no mechanism to get data back to the main thread, the worker only
// acts as a consumer. When this class is destroyed, it join()s the worker and
// finishes all outstanding tasks.
template <typename T, int QueueSize>
class ThreadedConsumer {
 public:
  // Constructs a new ThreadedConsumer with the given consumer function to be
  // run at the given realtime priority. If worker_priority is zero, the thread
  // will stay at non realtime priority.
  ThreadedConsumer(std::function<void(T)> consumer_function,
                   int worker_priority)
      : consumer_function_(consumer_function),
        worker_priority_(worker_priority),
        more_tasks_(&mutex_),
        worker_thread_([this]() { WorkerFunction(); }) {}

  ~ThreadedConsumer() {
    {
      aos::MutexLocker locker(&mutex_);
      quit_ = true;
      more_tasks_.Broadcast();
    }
    worker_thread_.join();
  }

  // Submits another task to be processed by the worker.
  // Returns true if successfully pushed onto the queue, and false if the queue
  // is full.
  bool Push(T task) {
    aos::MutexLocker locker(&mutex_);

    if (task_queue_.full()) {
      return false;
    }

    task_queue_.Push(task);
    more_tasks_.Broadcast();

    return true;
  }

 private:
  void WorkerFunction() {
    if (worker_priority_ > 0) {
      aos::SetCurrentThreadRealtimePriority(worker_priority_);
    }

    while (true) {
      std::optional<T> task;

      {
        aos::MutexLocker locker(&mutex_);
        while (task_queue_.empty() && !quit_) {
          CHECK(!more_tasks_.Wait());
        }

        if (task_queue_.empty() && quit_) break;

        // Pop
        task = std::move(task_queue_[0]);
        task_queue_.Shift();
      }

      consumer_function_(*task);
      task.reset();
    }

    aos::UnsetCurrentThreadRealtimePriority();
  }

  std::function<void(T)> consumer_function_;
  aos::RingBuffer<T, QueueSize> task_queue_;
  aos::Mutex mutex_;
  bool quit_ = false;
  int worker_priority_;
  aos::Condition more_tasks_;
  std::thread worker_thread_;
};

}  // namespace util
}  // namespace aos

#endif  // AOS_UTIL_THREADWORKER_H_
