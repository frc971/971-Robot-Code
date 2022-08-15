#ifndef AOS_UTIL_THREADED_QUEUE_H_
#define AOS_UTIL_THREADED_QUEUE_H_
#include <functional>
#include <optional>
#include <queue>
#include <thread>

#include "aos/condition.h"
#include "aos/mutex/mutex.h"

namespace aos::util {
// This class implements a queue of objects of type T in which a worker thread
// pushes and the calling thread pops from the queue.
//
// This is setup such that the user will pass a worker function that will get
// called in a separate thread whenever we think that there might be something
// to trigger more work to be done. All the methods on this calss are intended
// to be called from the main thread (not from within the worker function).
// Communication between the main thread and the worker can be achieved either
// by manually handling your own state, or preferably by using the SharedState
// object, which will get passed into the worker. The worker gets called every
// time that SetState() is called.
template <typename T, typename SharedState>
class ThreadedQueue {
 public:
  // PushResult is returned from the worker to indicate its current state.
  struct PushResult {
    // The new item to push, if any. If set to nullopt, nothing gets pushed.
    std::optional<T> item = std::nullopt;
    // Set to true if the worker knows that there is more work that it has to do
    // and so should be immediately called again. If set to false, then the
    // worker will not get called again until one of the following events
    // occurs:
    // 1) The queue is successfully popped from.
    // 2) SetState() is called.
    bool more_to_push = false;
    // Set to true if there is no more work to be done. The worker should not
    // get called after setting done to true.
    bool done = false;
  };
  ThreadedQueue(
      std::function<PushResult(SharedState)> push_request_handler,
      SharedState initial_state);
  ~ThreadedQueue();
  // Sets state. Triggers a new call to push_request_handler.
  void SetState(const SharedState &state);
  // Returns the current front of the queue, blocking until a new item is
  // available. Will only return nullopt if done is true and there will be no
  // more items in the queue.
  std::optional<T> Peek();
  // Identical to Peek(), except that it also removes the item from the queue.
  std::optional<T> Pop();
  // Waits until the push_request_handler has returned more_to_push = false, and
  // so is just spinning. Useful if you want ensure that the worker has done all
  // the work it can before going to the next step.
  void WaitForNoMoreWork();
  // Stops any further calls to push_request_handler. Used to terminate the
  // queue from the calling thread.
  void StopPushing();

 private:
  // Safely grabs the current state and returns a copy.
  SharedState State();
  // Implements the Peek()/Pop() methods, blocking until we are either done or
  // the next item is available in the queue.
  std::optional<T> PeekOrPop(bool pop);

  // Mutex controlling access to all shared state (in this case, all the members
  // of this class).
  aos::Mutex mutex_;
  // Condition variable used to indicate when anything has happened that should
  // cause us to check for whether the worker can add anything new to the queue.
  // Called "popped_" in reference to that it may be called when an item is
  // popped from the queue.
  aos::Condition popped_;
  // Condition variable to indicate when an item has either been pushed to the
  // queue or there is some other state change that consumers may care about
  // (e.g., being done).
  aos::Condition pushed_;
  // TODO(jkuszmaul): Evaluate impact of dynamic memory allocation in
  // std::queue, consider using aos::RingBuffer or similarly statically
  // allocated buffer.
  std::queue<T> queue_;
  // Whether we are done processing entirely.
  bool done_{false};
  // Set while the pusher thread is waiting on popped_.
  // Used to notice when the push handler is out of work to do.
  bool pusher_waiting_{false};
  // Set when SetState() is called, cleared when State() is read. Used to track
  // whether the push handler has been called with the most recent state.
  bool state_updated_{false};
  SharedState state_;
  std::thread pusher_thread_;
};

}  // namespace aos::util

#include "aos/util/threaded_queue_tmpl.h"
#endif  // AOS_UTIL_THREADED_QUEUE_H_
