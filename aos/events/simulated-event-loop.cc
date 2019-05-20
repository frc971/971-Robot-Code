#include "aos/events/simulated-event-loop.h"

#include <algorithm>

#include "aos/logging/logging.h"
#include "aos/queue.h"

namespace aos {
namespace {
class SimulatedFetcher : public RawFetcher {
 public:
  explicit SimulatedFetcher(SimulatedQueue *queue) : queue_(queue) {}
  ~SimulatedFetcher() {}

  bool FetchNext() override {
    LOG(FATAL, "Simulated event loops do not support FetchNext.");
    return false;
  }

  bool Fetch() override {
    if (index_ == queue_->index()) return false;

    // Fetched message is newer
    msg_ = queue_->latest_message();
    index_ = queue_->index();
    set_most_recent(reinterpret_cast<FetchValue *>(msg_.get()));
    return true;
  }

 private:
  int64_t index_ = -1;
  SimulatedQueue *queue_;
  RefCountedBuffer msg_;
};

class SimulatedSender : public RawSender {
 public:
  SimulatedSender(SimulatedQueue *queue, EventLoop *event_loop)
      : queue_(queue), event_loop_(event_loop) {}
  ~SimulatedSender() {}

  SendContext *GetContext() override {
    return reinterpret_cast<SendContext *>(
        RefCountedBuffer(queue_->size()).release());
  }

  void Free(SendContext *context) override {
    RefCountedBuffer(reinterpret_cast<aos::Message *>(context));
  }

  bool Send(SendContext *context) override {
    {
      ::aos::Message *aos_msg = reinterpret_cast<Message *>(context);
      if (aos_msg->sent_time == monotonic_clock::min_time) {
        aos_msg->sent_time = event_loop_->monotonic_now();
      }
    }
    queue_->Send(RefCountedBuffer(reinterpret_cast<aos::Message *>(context)));
    return true;  // Maybe false instead? :)
  }

  const char *name() const override { return queue_->name(); }

 private:
  SimulatedQueue *queue_;
  EventLoop *event_loop_;
};
}  // namespace

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
      : scheduler_(scheduler), queues_(queues) {}
  ~SimulatedEventLoop() override {};

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

EventScheduler::Token EventScheduler::Schedule(
    ::aos::monotonic_clock::time_point time, ::std::function<void()> callback) {
  return events_list_.emplace(time, callback);
}

void EventScheduler::Deschedule(EventScheduler::Token token) {
  events_list_.erase(token);
}

void EventScheduler::Run() {
  is_running_ = true;
  while (!events_list_.empty() && is_running_) {
    auto iter = events_list_.begin();
    now_ = iter->first;
    ::std::function<void()> callback = ::std::move(iter->second);
    events_list_.erase(iter);
    callback();
  }
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
    std::function<void(const aos::Message *message)> watcher) {
  watchers_.push_back(watcher);
}

std::unique_ptr<RawSender> SimulatedQueue::MakeRawSender(
    EventLoop *event_loop) {
  return std::unique_ptr<RawSender>(new SimulatedSender(this, event_loop));
}

std::unique_ptr<RawFetcher> SimulatedQueue::MakeRawFetcher() {
  return std::unique_ptr<RawFetcher>(new SimulatedFetcher(this));
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
