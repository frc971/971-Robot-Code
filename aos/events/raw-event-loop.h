#ifndef _AOS_EVENTS_RAW_EVENT_LOOP_H_
#define _AOS_EVENTS_RAW_EVENT_LOOP_H_

#include <atomic>
#include <memory>
#include <string>
#include "aos/queue.h"
#include "aos/time/time.h"

// This file contains raw versions of the classes in event-loop.h.
//
// Users should look exclusively at event-loop.h. Only people who wish to
// implement a new IPC layer (like a fake layer for example) may wish to use
// these classes.
namespace aos {

// Raw version of fetcher. Contains a local variable that the fetcher will
// update.
// It is the job of the typed version to cast this to the appropriate type.
class RawFetcher {
 public:
  class FetchValue;
  RawFetcher() {}
  virtual ~RawFetcher() {}

  // Non-blocking fetch of the next message in the queue. Returns true if there
  // was a new message and we got it.
  virtual bool FetchNext() = 0;
  // Non-blocking fetch of the latest message:
  virtual bool Fetch() = 0;

  const FetchValue *most_recent() { return most_recent_; }

 protected:
  RawFetcher(const RawFetcher &) = delete;
  RawFetcher &operator=(const RawFetcher &) = delete;
  void set_most_recent(const FetchValue *most_recent) {
    most_recent_ = most_recent;
  }

 private:
  const FetchValue *most_recent_ = nullptr;
};

// Raw version of sender. Sending a message is a 3 part process. Fetch an opaque
// token, cast that token to the message type, populate and then calling one of
// Send() or Free().
class RawSender {
 public:
  RawSender() {}
  virtual ~RawSender() {}

  virtual aos::Message *GetMessage() = 0;

  virtual void Free(aos::Message *msg) = 0;

  virtual bool Send(aos::Message *msg) = 0;

  // Call operator that calls Free().
  template <typename T>
  void operator()(T *t) {
    Free(t);
  }

  virtual const char *name() const = 0;

 protected:
  RawSender(const RawSender &) = delete;
  RawSender &operator=(const RawSender &) = delete;
};

// Opaque Information extracted from a particular type passed to the underlying
// system so that it knows how much memory to allocate etc.
struct QueueTypeInfo {
  // Message size:
  size_t size;
  // This should be a globally unique identifier for the type.
  int hash;
  // Config parameter for how long the queue should be.
  int queue_length;

  template <typename T>
  static QueueTypeInfo Get() {
    QueueTypeInfo info;
    info.size = sizeof(T);
    info.hash = T::kHash;
    info.queue_length = T::kQueueLength;
    return info;
  }

  // Necessary for the comparison of QueueTypeInfo objects in the
  // SimulatedEventLoop.
  bool operator<(const QueueTypeInfo &other) const {
    if (size != other.size) return size < other.size;
    if (hash != other.hash) return hash < other.hash;
    return queue_length < other.queue_length;
  }
};

// Interface for timers
class TimerHandler {
 public:
  virtual ~TimerHandler() {}

  // Timer should sleep until base, base + offset, base + offset * 2, ...
  // If repeat_offset isn't set, the timer only expires once.
  virtual void Setup(monotonic_clock::time_point base,
                     monotonic_clock::duration repeat_offset =
                         ::aos::monotonic_clock::zero()) = 0;

  // Stop future calls to callback().
  virtual void Disable() = 0;
};

// Interface for phased loops.  They are built on timers.
class PhasedLoopHandler {
 public:
  virtual ~PhasedLoopHandler() {}

  // Sets the interval and offset.  Any changes to interval and offset only take
  // effect when the handler finishes running.
  virtual void set_interval_and_offset(
      const monotonic_clock::duration interval,
      const monotonic_clock::duration offset) = 0;
};

class EventScheduler;

// Virtual base class for all event queue-types.
class RawEventLoop {
 public:
  virtual ~RawEventLoop() {}

  // Current time.
  virtual monotonic_clock::time_point monotonic_now() = 0;

  // The passed in function will be called when the event loop starts.
  // Use this to run code once the thread goes into "real-time-mode",
  virtual void OnRun(::std::function<void()> on_run) = 0;

  // Sets the name of the event loop.
  virtual void set_name(const char *name) = 0;

  // Threadsafe.
  bool is_running() const { return is_running_.load(); }

  // Creates a timer that executes callback when the timer expires
  // Returns a TimerHandle for configuration of the timer
  virtual TimerHandler *AddTimer(::std::function<void()> callback) = 0;

  // Creates a timer that executes callback periodically at the specified
  // interval and offset.  Returns a PhasedLoopHandler for interacting with the
  // timer.
  virtual PhasedLoopHandler *AddPhasedLoop(
      ::std::function<void(int)> callback,
      const monotonic_clock::duration interval,
      const monotonic_clock::duration offset = ::std::chrono::seconds(0)) = 0;

  // Stops receiving events.
  virtual void Exit() = 0;

  // TODO(austin): This shouldn't belong here.
  virtual void Run() = 0;

 protected:
  friend class EventScheduler;
  void set_is_running(bool value) { is_running_.store(value); }

  // Will send new messages from (path, type).
  virtual std::unique_ptr<RawSender> MakeRawSender(
      const std::string &path, const QueueTypeInfo &type) = 0;

  // Will fetch new messages from (path, type).
  virtual std::unique_ptr<RawFetcher> MakeRawFetcher(
      const std::string &path, const QueueTypeInfo &type) = 0;

  // Will watch (path, type) for new messages
  virtual void MakeRawWatcher(
      const std::string &path, const QueueTypeInfo &type,
      std::function<void(const Message *message)> watcher) = 0;

 private:
  ::std::atomic<bool> is_running_{false};
};

}  // namespace aos

#endif  // _AOS_EVENTS_RAW_EVENT_LOOP_H_
