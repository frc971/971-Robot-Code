#ifndef _AOS_EVENTS_SIMULATED_EVENT_LOOP_H_
#define _AOS_EVENTS_SIMULATED_EVENT_LOOP_H_

#include <algorithm>
#include <map>
#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include "aos/events/event-loop.h"

namespace aos {

// This class manages allocation of queue messages for simulation.
// Unfortunately, because the current interfaces all assume that we pass around
// raw pointers to messages we can't use a std::shared_ptr or the such, and
// because aos::Message's themselves to not have any sort of built-in support
// for this, we need to manage memory for the Messages in some custom fashion.
// In this case, we do so by allocating a ref-counter in the bytes immediately
// preceding the aos::Message. We then provide a constructor that takes just a
// pointer to an existing message and we assume that it was allocated using this
// class, and can decrement the counter if the RefCountedBuffer we constructed
// goes out of scope. There are currently no checks to ensure that pointers
// passed into this class were actually allocated using this class.
class RefCountedBuffer {
 public:
  RefCountedBuffer() {}
  ~RefCountedBuffer() { clear(); }

  // Create a RefCountedBuffer for some Message that was already allocated using
  // a RefCountedBuffer class. This, or some function like it, is required to
  // allow us to let users of the simulated event loops work with raw pointers
  // to messages.
  explicit RefCountedBuffer(aos::Message *data) : data_(data) {}

  // Allocates memory for a new message of a given size. Does not initialize the
  // memory or call any constructors.
  explicit RefCountedBuffer(size_t size) {
    data_ = reinterpret_cast<uint8_t *>(malloc(kRefCountSize + size)) +
            kRefCountSize;
    // Initialize the allocated memory with an integer
    *GetRefCount() = 1;
  }

  RefCountedBuffer(const RefCountedBuffer &other) {
    data_ = other.data_;
    if (data_ != nullptr) {
      ++*GetRefCount();
    }
  }

  RefCountedBuffer(RefCountedBuffer &&other) { std::swap(data_, other.data_); }

  RefCountedBuffer &operator=(const RefCountedBuffer &other) {
    if (this == &other) return *this;
    clear();
    data_ = other.data_;
    ++*GetRefCount();
    return *this;
  }

  RefCountedBuffer &operator=(RefCountedBuffer &&other) {
    if (this == &other) return *this;
    std::swap(data_, other.data_);
    return *this;
  }

  operator bool() const { return data_ != nullptr; }

  aos::Message *get() const { return static_cast<aos::Message *>(data_); }

  aos::Message *release() {
    auto tmp = get();
    data_ = nullptr;
    return tmp;
  }

  void clear() {
    if (data_ != nullptr) {
      if (--*GetRefCount() == 0) {
        // Free memory block from the start of the allocated block
        free(GetRefCount());
      }
      data_ = nullptr;
    }
  }

 private:
  void *data_ = nullptr;
  // Qty. memory to be allocated to the ref counter
  static constexpr size_t kRefCountSize = sizeof(int64_t);

  int64_t *GetRefCount() {
    // Need to cast the void* to an 8 bit long object (size of void* is
    // technically 0)
    return reinterpret_cast<int64_t *>(static_cast<void *>(
        reinterpret_cast<uint8_t *>(data_) - kRefCountSize));
  }
};

class EventScheduler {
 public:
  using QueueType = ::std::multimap<::aos::monotonic_clock::time_point,
                                    ::std::function<void()>>;
  using Token = QueueType::iterator;

  // Schedule an event with a callback function
  // Returns an iterator to the event
  Token Schedule(::aos::monotonic_clock::time_point time,
                 ::std::function<void()> callback);

  // Deschedule an event by its iterator
  void Deschedule(Token token);

  void Run();
  void RunFor(::aos::monotonic_clock::duration duration);

  void Exit() {
    is_running_ = false;
  }

  bool is_running() const { return is_running_; }

  void AddRawEventLoop(RawEventLoop *event_loop) {
    raw_event_loops_.push_back(event_loop);
  }
  void RemoveRawEventLoop(RawEventLoop *event_loop) {
    raw_event_loops_.erase(::std::find(raw_event_loops_.begin(),
                                       raw_event_loops_.end(), event_loop));
  }

  ::aos::monotonic_clock::time_point monotonic_now() const { return now_; }

 private:
  ::aos::monotonic_clock::time_point now_ = ::aos::monotonic_clock::epoch();
  QueueType events_list_;
  bool is_running_ = false;
  ::std::vector<RawEventLoop *> raw_event_loops_;
};

// Class for simulated fetchers.
class SimulatedFetcher;

class SimulatedQueue {
 public:
  explicit SimulatedQueue(const QueueTypeInfo &type, const ::std::string &name,
                          EventScheduler *scheduler)
      : type_(type), name_(name), scheduler_(scheduler){};

  ~SimulatedQueue() { AOS_CHECK_EQ(0u, fetchers_.size()); }

  // Makes a connected raw sender which calls Send below.
  ::std::unique_ptr<RawSender> MakeRawSender(EventLoop *event_loop);

  // Makes a connected raw fetcher.
  ::std::unique_ptr<RawFetcher> MakeRawFetcher();

  // Registers a watcher for the queue.
  void MakeRawWatcher(
      ::std::function<void(const ::aos::Message *message)> watcher);

  // Sends the message to all the connected receivers and fetchers.
  void Send(RefCountedBuffer message);

  // Unregisters a fetcher.
  void UnregisterFetcher(SimulatedFetcher *fetcher);

  const RefCountedBuffer &latest_message() { return latest_message_; }

  size_t size() const { return type_.size; }

  const char *name() const { return name_.c_str(); }

 private:
  const QueueTypeInfo type_;
  const ::std::string name_;

  // List of all watchers.
  ::std::vector<std::function<void(const aos::Message *message)>> watchers_;

  // List of all fetchers.
  ::std::vector<SimulatedFetcher *> fetchers_;
  RefCountedBuffer latest_message_;
  EventScheduler *scheduler_;
};

class SimulatedEventLoopFactory {
 public:
  ::std::unique_ptr<EventLoop> MakeEventLoop();

  // Starts executing the event loops unconditionally.
  void Run() { scheduler_.Run(); }
  // Executes the event loops for a duration.
  void RunFor(monotonic_clock::duration duration) {
    scheduler_.RunFor(duration);
  }

  // Stops executing all event loops.  Meant to be called from within an event
  // loop handler.
  void Exit() { scheduler_.Exit(); }

  monotonic_clock::time_point monotonic_now() const {
    return scheduler_.monotonic_now();
  }

 private:
  EventScheduler scheduler_;
  ::std::map<::std::pair<::std::string, QueueTypeInfo>, SimulatedQueue> queues_;
};

}  // namespace aos

#endif  //_AOS_EVENTS_TEST_EVENT_LOOP_H_
