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
  SimulatedSender(SimulatedQueue *queue) : queue_(queue) {}
  ~SimulatedSender() {}

  SendContext *GetContext() override {
    return reinterpret_cast<SendContext *>(
        RefCountedBuffer(queue_->size()).release());
  }

  void Free(SendContext *context) override {
    RefCountedBuffer(reinterpret_cast<aos::Message *>(context));
  }

  bool Send(SendContext *context) override {
    queue_->Send(RefCountedBuffer(reinterpret_cast<aos::Message *>(context)));
    return true;  // Maybe false instead? :)
  }

  const char *name() const override { return queue_->name(); }

 private:
  SimulatedQueue *queue_;
};
}  // namespace

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
  return GetSimulatedQueue(key)->MakeRawSender();
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

std::unique_ptr<RawSender> SimulatedQueue::MakeRawSender() {
  return std::unique_ptr<RawSender>(new SimulatedSender(this));
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
}  // namespace aos
