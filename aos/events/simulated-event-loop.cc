#include "aos/events/simulated-event-loop.h"

#include <algorithm>
#include <deque>

#include "aos/logging/logging.h"
#include "aos/queue.h"
#include "aos/testing/test_logging.h"

namespace aos {
namespace {

class SimulatedSender : public RawSender {
 public:
  SimulatedSender(SimulatedQueue *queue, EventLoop *event_loop)
      : queue_(queue), event_loop_(event_loop) {
    testing::EnableTestLogging();
  }
  ~SimulatedSender() {}

  aos::Message *GetMessage() override {
    return RefCountedBuffer(queue_->size()).release();
  }

  void Free(aos::Message *msg) override { RefCountedBuffer tmp(msg); }

  bool Send(aos::Message *msg) override {
    {
      if (msg->sent_time == monotonic_clock::min_time) {
        msg->sent_time = event_loop_->monotonic_now();
      }
    }
    queue_->Send(RefCountedBuffer(msg));
    return true;
  }

  const char *name() const override { return queue_->name(); }

 private:
  SimulatedQueue *queue_;
  EventLoop *event_loop_;
};
}  // namespace

class SimulatedFetcher : public RawFetcher {
 public:
  explicit SimulatedFetcher(SimulatedQueue *queue) : queue_(queue) {}
  ~SimulatedFetcher() { queue_->UnregisterFetcher(this); }

  bool FetchNext() override {
    if (msgs_.size() == 0) return false;

    msg_ = msgs_.front();
    msgs_.pop_front();
    set_most_recent(reinterpret_cast<FetchValue *>(msg_.get()));
    return true;
  }

  bool Fetch() override {
    if (msgs_.size() == 0) {
      if (!msg_ && queue_->latest_message()) {
        msg_ = queue_->latest_message();
        set_most_recent(reinterpret_cast<FetchValue *>(msg_.get()));
        return true;
      } else {
        return false;
      }
    }

    // We've had a message enqueued, so we don't need to go looking for the
    // latest message from before we started.
    msg_ = msgs_.back();
    msgs_.clear();
    set_most_recent(reinterpret_cast<FetchValue *>(msg_.get()));
    return true;
  }

 private:
  friend class SimulatedQueue;

  // Internal method for Simulation to add a message to the buffer.
  void Enqueue(RefCountedBuffer buffer) {
    msgs_.emplace_back(buffer);
  }

  SimulatedQueue *queue_;
  RefCountedBuffer msg_;

  // Messages queued up but not in use.
  ::std::deque<RefCountedBuffer> msgs_;
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
    const ::aos::monotonic_clock::time_point monotonic_now =
        scheduler_->monotonic_now();
    base_ = base;
    repeat_offset_ = repeat_offset;
    if (base < monotonic_now) {
      token_ = scheduler_->Schedule(monotonic_now, [this]() { HandleEvent(); });
    } else {
      token_ = scheduler_->Schedule(base, [this]() { HandleEvent(); });
    }
  }

  void HandleEvent() {
    const ::aos::monotonic_clock::time_point monotonic_now =
        scheduler_->monotonic_now();
    if (repeat_offset_ != ::aos::monotonic_clock::zero()) {
      // Reschedule.
      while (base_ <= monotonic_now) base_ += repeat_offset_;
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
          *queues)
      : scheduler_(scheduler), queues_(queues) {
    scheduler_->AddRawEventLoop(this);
  }
  ~SimulatedEventLoop() override { scheduler_->RemoveRawEventLoop(this); };

  ::aos::monotonic_clock::time_point monotonic_now() override {
    return scheduler_->monotonic_now();
  }

  ::std::unique_ptr<RawSender> MakeRawSender(
      const ::std::string &path, const QueueTypeInfo &type) override;

  ::std::unique_ptr<RawFetcher> MakeRawFetcher(
      const ::std::string &path, const QueueTypeInfo &type) override;

  void MakeRawWatcher(
      const ::std::string &path, const QueueTypeInfo &type,
      ::std::function<void(const ::aos::Message *message)> watcher) override;

  TimerHandler *AddTimer(::std::function<void()> callback) override {
    timers_.emplace_back(new SimulatedTimerHandler(scheduler_, callback));
    return timers_.back().get();
  }

  void OnRun(::std::function<void()> on_run) override {
    scheduler_->Schedule(scheduler_->monotonic_now(), on_run);
  }
  void Run() override {
    LOG(FATAL, "Run from the factory instead\n");
    scheduler_->Run();
  }
  void Exit() override {
    scheduler_->Exit();
  }

  SimulatedQueue *GetSimulatedQueue(
      const ::std::pair<::std::string, QueueTypeInfo> &);

  void Take(const ::std::string &path);

  void SetRuntimeRealtimePriority(int /*priority*/) override {
    if (is_running()) {
      ::aos::Die("Cannot set realtime priority while running.");
    }
  }

 private:
  EventScheduler *scheduler_;
  ::std::map<::std::pair<::std::string, QueueTypeInfo>, SimulatedQueue>
      *queues_;
  ::std::vector<std::string> taken_;
  ::std::vector<std::unique_ptr<TimerHandler>> timers_;
};

EventScheduler::Token EventScheduler::Schedule(
    ::aos::monotonic_clock::time_point time, ::std::function<void()> callback) {
  return events_list_.emplace(time, callback);
}

void EventScheduler::Deschedule(EventScheduler::Token token) {
  events_list_.erase(token);
}

void EventScheduler::RunFor(monotonic_clock::duration duration) {
  const ::aos::monotonic_clock::time_point end_time =
      monotonic_now() + duration;
  testing::MockTime(monotonic_now());
  for (RawEventLoop *event_loop : raw_event_loops_) {
    event_loop->set_is_running(true);
  }
  is_running_ = true;
  while (!events_list_.empty() && is_running_) {
    auto iter = events_list_.begin();
    ::aos::monotonic_clock::time_point next_time = iter->first;
    if (next_time > end_time) {
      break;
    }
    now_ = iter->first;
    testing::MockTime(now_);
    ::std::function<void()> callback = ::std::move(iter->second);
    events_list_.erase(iter);
    callback();
  }
  now_ = end_time;
  if (!is_running_) {
    for (RawEventLoop *event_loop : raw_event_loops_) {
      event_loop->set_is_running(false);
    }
  }
  testing::UnMockTime();
}

void EventScheduler::Run() {
  testing::MockTime(monotonic_now());
  for (RawEventLoop *event_loop : raw_event_loops_) {
    event_loop->set_is_running(true);
  }
  is_running_ = true;
  while (!events_list_.empty() && is_running_) {
    auto iter = events_list_.begin();
    now_ = iter->first;
    testing::MockTime(now_);
    ::std::function<void()> callback = ::std::move(iter->second);
    events_list_.erase(iter);
    callback();
  }
  if (!is_running_) {
    for (RawEventLoop *event_loop : raw_event_loops_) {
      event_loop->set_is_running(false);
    }
  }
  testing::UnMockTime();
}

void SimulatedEventLoop::MakeRawWatcher(
    const std::string &path, const QueueTypeInfo &type,
    std::function<void(const aos::Message *message)> watcher) {
  Take(path);
  ::std::pair<::std::string, QueueTypeInfo> key(path, type);
  GetSimulatedQueue(key)->MakeRawWatcher(watcher);
}

std::unique_ptr<RawSender> SimulatedEventLoop::MakeRawSender(
    const std::string &path, const QueueTypeInfo &type) {
  Take(path);
  ::std::pair<::std::string, QueueTypeInfo> key(path, type);
  return GetSimulatedQueue(key)->MakeRawSender(this);
}

std::unique_ptr<RawFetcher> SimulatedEventLoop::MakeRawFetcher(
    const std::string &path, const QueueTypeInfo &type) {
  ::std::pair<::std::string, QueueTypeInfo> key(path, type);
  return GetSimulatedQueue(key)->MakeRawFetcher();
}

SimulatedQueue *SimulatedEventLoop::GetSimulatedQueue(
    const ::std::pair<::std::string, QueueTypeInfo> &type) {
  auto it = queues_->find(type);
  if (it == queues_->end()) {
    it =
        queues_
            ->emplace(type, SimulatedQueue(type.second, type.first, scheduler_))
            .first;
  }
  return &it->second;
}

void SimulatedQueue::MakeRawWatcher(
    ::std::function<void(const aos::Message *message)> watcher) {
  watchers_.push_back(watcher);
}

::std::unique_ptr<RawSender> SimulatedQueue::MakeRawSender(
    EventLoop *event_loop) {
  return ::std::unique_ptr<RawSender>(new SimulatedSender(this, event_loop));
}

::std::unique_ptr<RawFetcher> SimulatedQueue::MakeRawFetcher() {
  ::std::unique_ptr<SimulatedFetcher> fetcher(new SimulatedFetcher(this));
  fetchers_.push_back(fetcher.get());
  return ::std::move(fetcher);
}

void SimulatedQueue::Send(RefCountedBuffer message) {
  latest_message_ = message;
  if (scheduler_->is_running()) {
    for (auto &watcher : watchers_) {
      scheduler_->Schedule(scheduler_->monotonic_now(),
                           [watcher, message]() { watcher(message.get()); });
    }
  }
  for (auto &fetcher : fetchers_) {
    fetcher->Enqueue(message);
  }
}

void SimulatedQueue::UnregisterFetcher(SimulatedFetcher *fetcher) {
  fetchers_.erase(::std::find(fetchers_.begin(), fetchers_.end(), fetcher));
}

void SimulatedEventLoop::Take(const ::std::string &path) {
  if (is_running()) {
    ::aos::Die("Cannot add new objects while running.\n");
  }
  const auto prior = ::std::find(taken_.begin(), taken_.end(), path);
  if (prior != taken_.end()) {
    ::aos::Die("%s is already being used.", path.c_str());
  } else {
    taken_.emplace_back(path);
  }
}

::std::unique_ptr<EventLoop> SimulatedEventLoopFactory::MakeEventLoop() {
  return ::std::unique_ptr<EventLoop>(
      new SimulatedEventLoop(&scheduler_, &queues_));
}

}  // namespace aos
