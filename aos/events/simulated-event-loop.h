#ifndef _AOS_EVENTS_SIMULATED_EVENT_LOOP_H_
#define _AOS_EVENTS_SIMULATED_EVENT_LOOP_H_

#include <map>
#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include "aos/events/event-loop.h"

namespace aos {

class RefCountedBuffer {
 public:
  RefCountedBuffer() {}
  ~RefCountedBuffer() { clear(); }

  explicit RefCountedBuffer(aos::Message *data) : data_(data) {}

  explicit RefCountedBuffer(size_t size) {
    data_ = reinterpret_cast<uint8_t *>(malloc(kRefCountSize + size)) +
            kRefCountSize;
    // Initialize the allocated memory with an integer
    *GetRefCount() = 1;
  }

  RefCountedBuffer(const RefCountedBuffer &other) {
    data_ = other.data_;
    ++*GetRefCount();
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

  void Exit() { is_running_ = false; }

  ::aos::monotonic_clock::time_point monotonic_now() const { return now_; }

 private:
  ::aos::monotonic_clock::time_point now_ = ::aos::monotonic_clock::epoch();
  QueueType events_list_;
  bool is_running_ = false;
};

class SimulatedQueue {
 public:
  explicit SimulatedQueue(const QueueTypeInfo &type, const ::std::string &name,
                          EventScheduler *scheduler)
      : type_(type), name_(name), scheduler_(scheduler){};

  ::std::unique_ptr<RawSender> MakeRawSender(EventLoop *event_loop);

  ::std::unique_ptr<RawFetcher> MakeRawFetcher();

  void MakeRawWatcher(
      ::std::function<void(const ::aos::Message *message)> watcher);

  void Send(RefCountedBuffer message) {
    index_++;
    latest_message_ = message;
    for (auto &watcher : watchers_) {
      scheduler_->Schedule(scheduler_->monotonic_now(),
                           [watcher, message]() { watcher(message.get()); });
    }
  }

  const RefCountedBuffer &latest_message() { return latest_message_; }

  int64_t index() { return index_; }

  size_t size() { return type_.size; }

  const char *name() const { return name_.c_str(); }

 private:
  int64_t index_ = -1;
  QueueTypeInfo type_;
  const ::std::string name_;
  ::std::vector<std::function<void(const aos::Message *message)>> watchers_;
  RefCountedBuffer latest_message_;
  EventScheduler *scheduler_;
};

class SimulatedEventLoopFactory {
 public:
  ::std::unique_ptr<EventLoop> MakeEventLoop();

  void Run() { scheduler_.Run(); }

  monotonic_clock::time_point monotonic_now() const {
    return scheduler_.monotonic_now();
  }

 private:
  EventScheduler scheduler_;
  ::std::map<::std::pair<::std::string, QueueTypeInfo>, SimulatedQueue> queues_;
};

}  // namespace aos

#endif  //_AOS_EVENTS_TEST_EVENT_LOOP_H_
