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

  ::aos::monotonic_clock::time_point now() { return now_; }

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

  std::unique_ptr<RawSender> MakeRawSender();

  std::unique_ptr<RawFetcher> MakeRawFetcher();

  void MakeRawWatcher(std::function<void(const aos::Message *message)> watcher);

  void Send(RefCountedBuffer message) {
    index_++;
    latest_message_ = message;
    for (auto &watcher : watchers_) {
      scheduler_->Schedule(scheduler_->now(),
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

class SimulatedTimerHandler : public TimerHandler {
 public:
  explicit SimulatedTimerHandler(EventScheduler *scheduler,
                                 ::std::function<void()> fn)
      : scheduler_(scheduler), fn_(fn) {}
  ~SimulatedTimerHandler() {}

  void Setup(monotonic_clock::time_point base,
             monotonic_clock::duration repeat_offset) override {
    Disable();
    auto now = scheduler_->now();
    base_ = base;
    repeat_offset_ = repeat_offset;
    if (base < now) {
      token_ = scheduler_->Schedule(now, [this]() { HandleEvent(); });
    } else {
      token_ = scheduler_->Schedule(base, [this]() { HandleEvent(); });
    }
  }

  void HandleEvent() {
    auto now = scheduler_->now();
    if (repeat_offset_ != ::aos::monotonic_clock::zero()) {
      // Reschedule.
      while (base_ <= now) base_ += repeat_offset_;
      token_ = scheduler_->Schedule(base_, [this]() { HandleEvent(); });
    } else {
      token_ = EventScheduler::Token();
    }
    fn_();
  }

  void Disable() override {
    if (token_ != EventScheduler::Token()) {
      scheduler_->Deschedule(token_);
      token_ = EventScheduler::Token();
    }
  }

 private:
  EventScheduler *scheduler_;
  EventScheduler::Token token_;
  // Function to be run on the thread
  ::std::function<void()> fn_;
  monotonic_clock::time_point base_;
  monotonic_clock::duration repeat_offset_;
};

class SimulatedEventLoop : public EventLoop {
 public:
  explicit SimulatedEventLoop(
      EventScheduler *scheduler,
      ::std::map<::std::pair<::std::string, QueueTypeInfo>, SimulatedQueue>
          *queues) : scheduler_(scheduler), queues_(queues){};
  ~SimulatedEventLoop() override{};

  ::aos::monotonic_clock::time_point monotonic_now() override {
    return scheduler_->now();
  }

  std::unique_ptr<RawSender> MakeRawSender(const std::string &path,
                                           const QueueTypeInfo &type) override;

  std::unique_ptr<RawFetcher> MakeRawFetcher(
      const std::string &path, const QueueTypeInfo &type) override;

  void MakeRawWatcher(
      const std::string &path, const QueueTypeInfo &type,
      std::function<void(const aos::Message *message)> watcher) override;

  TimerHandler *AddTimer(::std::function<void()> callback) override {
    timers_.emplace_back(new SimulatedTimerHandler(scheduler_, callback));
    return timers_.back().get();
  }

  void OnRun(std::function<void()> on_run) override {
    scheduler_->Schedule(scheduler_->now(), on_run);
  }
  void Run() override {
    set_is_running(true);
    scheduler_->Run();
  }
  void Exit() override {
    set_is_running(false);
    scheduler_->Exit();
  }

  SimulatedQueue *GetSimulatedQueue(
      const ::std::pair<::std::string, QueueTypeInfo> &);

  void Take(const ::std::string &path);

 private:
  EventScheduler *scheduler_;
  ::std::map<::std::pair<::std::string, QueueTypeInfo>, SimulatedQueue>
      *queues_;
  ::std::vector<std::string> taken_;
  ::std::vector<std::unique_ptr<TimerHandler>> timers_;
};

class SimulatedEventLoopFactory {
 public:
  ::std::unique_ptr<EventLoop> CreateEventLoop() {
    return ::std::unique_ptr<EventLoop>(
        new SimulatedEventLoop(&scheduler_, &queues_));
  }

 private:
  EventScheduler scheduler_;
  ::std::map<::std::pair<::std::string, QueueTypeInfo>, SimulatedQueue> queues_;
};
}  // namespace aos
#endif  //_AOS_EVENTS_TEST_EVENT_LOOP_H_
