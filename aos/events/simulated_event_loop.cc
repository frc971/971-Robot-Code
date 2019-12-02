#include "aos/events/simulated_event_loop.h"

#include <algorithm>
#include <deque>
#include <string_view>

#include "absl/container/btree_map.h"
#include "absl/container/btree_set.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/util/phased_loop.h"

namespace aos {

// Container for both a message, and the context for it for simulation.  This
// makes tracking the timestamps associated with the data easy.
struct SimulatedMessage {
  // Struct to let us force data to be well aligned.
  struct OveralignedChar {
    char data alignas(32);
  };

  // Context for the data.
  Context context;

  // The data.
  char *data() { return reinterpret_cast<char *>(&actual_data[0]); }

  // Then the data.
  OveralignedChar actual_data[];
};

class SimulatedFetcher;
class SimulatedChannel;

class ShmWatcher : public WatcherState {
 public:
  ShmWatcher(
      EventLoop *event_loop, const Channel *channel,
      std::function<void(const Context &context, const void *message)> fn)
      : WatcherState(event_loop, channel, std::move(fn)),
        event_loop_(event_loop) {}

  ~ShmWatcher() override;

  void Call(const Context &context) {
    const monotonic_clock::time_point monotonic_now =
        event_loop_->monotonic_now();
    DoCallCallback([monotonic_now]() { return monotonic_now; }, context);
  }

  void Startup(EventLoop * /*event_loop*/) override {}

  void Schedule(EventScheduler *scheduler,
                std::shared_ptr<SimulatedMessage> message) {
    // TODO(austin): Track the token once we schedule in the future.
    // TODO(austin): Schedule wakeup in the future so we don't have 0 latency.
    scheduler->Schedule(scheduler->monotonic_now(),
                        [this, message]() { Call(message->context); });
  }

  void SetSimulatedChannel(SimulatedChannel *channel) {
    simulated_channel_ = channel;
  }

 private:
  EventLoop *event_loop_;
  SimulatedChannel *simulated_channel_ = nullptr;
};

class SimulatedChannel {
 public:
  explicit SimulatedChannel(const Channel *channel, EventScheduler *scheduler)
      : channel_(channel),
        scheduler_(scheduler),
        next_queue_index_(ipc_lib::QueueIndex::Zero(channel->max_size())) {}

  ~SimulatedChannel() { CHECK_EQ(0u, fetchers_.size()); }

  // Makes a connected raw sender which calls Send below.
  ::std::unique_ptr<RawSender> MakeRawSender(EventLoop *event_loop);

  // Makes a connected raw fetcher.
  ::std::unique_ptr<RawFetcher> MakeRawFetcher(EventLoop *event_loop);

  // Registers a watcher for the queue.
  void MakeRawWatcher(ShmWatcher *watcher);

  void RemoveWatcher(ShmWatcher *watcher) {
    watchers_.erase(std::find(watchers_.begin(), watchers_.end(), watcher));
  }

  // Sends the message to all the connected receivers and fetchers.
  void Send(std::shared_ptr<SimulatedMessage> message);

  // Unregisters a fetcher.
  void UnregisterFetcher(SimulatedFetcher *fetcher);

  std::shared_ptr<SimulatedMessage> latest_message() { return latest_message_; }

  size_t max_size() const { return channel()->max_size(); }

  const std::string_view name() const {
    return channel()->name()->string_view();
  }

  const Channel *channel() const { return channel_; }

  ::aos::monotonic_clock::time_point monotonic_now() const {
    return scheduler_->monotonic_now();
  }

 private:
  const Channel *channel_;

  // List of all watchers.
  ::std::vector<ShmWatcher *> watchers_;

  // List of all fetchers.
  ::std::vector<SimulatedFetcher *> fetchers_;
  std::shared_ptr<SimulatedMessage> latest_message_;
  EventScheduler *scheduler_;

  ipc_lib::QueueIndex next_queue_index_;
};

ShmWatcher::~ShmWatcher() { simulated_channel_->RemoveWatcher(this); }

namespace {

// Creates a SimulatedMessage with size bytes of storage.
// This is a shared_ptr so we don't have to implement refcounting or copying.
std::shared_ptr<SimulatedMessage> MakeSimulatedMessage(size_t size) {
  SimulatedMessage *message = reinterpret_cast<SimulatedMessage *>(
      malloc(sizeof(SimulatedMessage) + size));
  message->context.size = size;
  message->context.data = message->data();

  return std::shared_ptr<SimulatedMessage>(message, free);
}

class SimulatedSender : public RawSender {
 public:
  SimulatedSender(SimulatedChannel *simulated_channel, EventLoop *event_loop)
      : RawSender(event_loop, simulated_channel->channel()),
        simulated_channel_(simulated_channel),
        event_loop_(event_loop) {}
  ~SimulatedSender() {}

  void *data() override {
    if (!message_) {
      message_ = MakeSimulatedMessage(simulated_channel_->max_size());
    }
    return message_->data();
  }

  size_t size() override { return simulated_channel_->max_size(); }

  bool DoSend(size_t length) override {
    CHECK_LE(length, size()) << ": Attempting to send too big a message.";
    message_->context.monotonic_sent_time = event_loop_->monotonic_now();
    message_->context.realtime_sent_time = event_loop_->realtime_now();
    CHECK_LE(length, message_->context.size);
    message_->context.size = length;

    // TODO(austin): Track sending too fast.
    simulated_channel_->Send(message_);

    // Drop the reference to the message so that we allocate a new message for
    // next time.  Otherwise we will continue to reuse the same memory for all
    // messages and corrupt it.
    message_.reset();
    return true;
  }

  bool DoSend(const void *msg, size_t size) override {
    CHECK_LE(size, this->size()) << ": Attempting to send too big a message.";

    // This is wasteful, but since flatbuffers fill from the back end of the
    // queue, we need it to be full sized.
    message_ = MakeSimulatedMessage(simulated_channel_->max_size());

    // Now fill in the message.  size is already populated above, and
    // queue_index will be populated in queue_.  Put this at the back of the
    // data segment.
    memcpy(message_->data() + simulated_channel_->max_size() - size, msg, size);

    return Send(size);
  }

 private:
  SimulatedChannel *simulated_channel_;
  EventLoop *event_loop_;

  std::shared_ptr<SimulatedMessage> message_;
};
}  // namespace

class SimulatedEventLoop;

class SimulatedFetcher : public RawFetcher {
 public:
  explicit SimulatedFetcher(EventLoop *event_loop, SimulatedChannel *queue)
      : RawFetcher(event_loop, queue->channel()), queue_(queue) {}
  ~SimulatedFetcher() { queue_->UnregisterFetcher(this); }

  std::pair<bool, monotonic_clock::time_point> DoFetchNext() override {
    if (msgs_.size() == 0) {
      return std::make_pair(false, monotonic_clock::min_time);
    }

    SetMsg(msgs_.front());
    msgs_.pop_front();
    return std::make_pair(true, queue_->monotonic_now());
  }

  std::pair<bool, monotonic_clock::time_point> DoFetch() override {
    if (msgs_.size() == 0) {
      if (!msg_ && queue_->latest_message()) {
        SetMsg(queue_->latest_message());
        return std::make_pair(true, queue_->monotonic_now());
      } else {
        return std::make_pair(false, monotonic_clock::min_time);
      }
    }

    // We've had a message enqueued, so we don't need to go looking for the
    // latest message from before we started.
    SetMsg(msgs_.back());
    msgs_.clear();
    return std::make_pair(true, queue_->monotonic_now());
  }

 private:
  friend class SimulatedChannel;

  // Updates the state inside RawFetcher to point to the data in msg_.
  void SetMsg(std::shared_ptr<SimulatedMessage> msg) {
    msg_ = msg;
    context_ = msg_->context;
  }

  // Internal method for Simulation to add a message to the buffer.
  void Enqueue(std::shared_ptr<SimulatedMessage> buffer) {
    msgs_.emplace_back(buffer);
  }

  SimulatedChannel *queue_;
  std::shared_ptr<SimulatedMessage> msg_;

  // Messages queued up but not in use.
  ::std::deque<std::shared_ptr<SimulatedMessage>> msgs_;
};

class SimulatedTimerHandler : public TimerHandler {
 public:
  explicit SimulatedTimerHandler(EventScheduler *scheduler,
                                 SimulatedEventLoop *simulated_event_loop,
                                 ::std::function<void()> fn);
  ~SimulatedTimerHandler() {}

  void Setup(monotonic_clock::time_point base,
             monotonic_clock::duration repeat_offset) override;

  void HandleEvent();

  void Disable() override {
    if (token_ != scheduler_->InvalidToken()) {
      scheduler_->Deschedule(token_);
      token_ = scheduler_->InvalidToken();
    }
  }

  ::aos::monotonic_clock::time_point monotonic_now() const {
    return scheduler_->monotonic_now();
  }

 private:
  EventScheduler *scheduler_;
  EventScheduler::Token token_;

  monotonic_clock::time_point base_;
  monotonic_clock::duration repeat_offset_;
};

class SimulatedPhasedLoopHandler : public PhasedLoopHandler {
 public:
  SimulatedPhasedLoopHandler(EventScheduler *scheduler,
                             SimulatedEventLoop *simulated_event_loop,
                             ::std::function<void(int)> fn,
                             const monotonic_clock::duration interval,
                             const monotonic_clock::duration offset);
  ~SimulatedPhasedLoopHandler() {
    if (token_ != scheduler_->InvalidToken()) {
      scheduler_->Deschedule(token_);
      token_ = scheduler_->InvalidToken();
    }
  }

  void HandleTimerWakeup();

  void Schedule(monotonic_clock::time_point sleep_time) override {
    token_ = scheduler_->Schedule(sleep_time, [this]() { HandleTimerWakeup(); });
  }

 private:
  SimulatedEventLoop *simulated_event_loop_;

  EventScheduler *scheduler_;
  EventScheduler::Token token_;
};

class SimulatedEventLoop : public EventLoop {
 public:
  explicit SimulatedEventLoop(
      EventScheduler *scheduler,
      absl::btree_map<SimpleChannel, std::unique_ptr<SimulatedChannel>>
          *channels,
      const Configuration *configuration,
      std::vector<std::pair<EventLoop *, std::function<void(bool)>>>
          *raw_event_loops,
      pid_t tid)
      : EventLoop(CHECK_NOTNULL(configuration)),
        scheduler_(scheduler),
        channels_(channels),
        raw_event_loops_(raw_event_loops),
        tid_(tid) {
    raw_event_loops_->push_back(std::make_pair(this, [this](bool value) {
      if (!has_setup_) {
        Setup();
        has_setup_ = true;
      }
      set_is_running(value);
    }));
  }
  ~SimulatedEventLoop() override {
    // Trigger any remaining senders or fetchers to be cleared before destroying
    // the event loop so the book keeping matches.
    timing_report_sender_.reset();

    // Force everything with a registered fd with epoll to be destroyed now.
    timers_.clear();
    phased_loops_.clear();
    watchers_.clear();

    for (auto it = raw_event_loops_->begin(); it != raw_event_loops_->end();
         ++it) {
      if (it->first == this) {
        raw_event_loops_->erase(it);
        break;
      }
    }
  }

  ::aos::monotonic_clock::time_point monotonic_now() override {
    return scheduler_->monotonic_now();
  }

  ::aos::realtime_clock::time_point realtime_now() override {
    return scheduler_->realtime_now();
  }

  ::std::unique_ptr<RawSender> MakeRawSender(const Channel *channel) override;

  ::std::unique_ptr<RawFetcher> MakeRawFetcher(const Channel *channel) override;

  void MakeRawWatcher(
      const Channel *channel,
      ::std::function<void(const Context &context, const void *message)>
          watcher) override;

  TimerHandler *AddTimer(::std::function<void()> callback) override {
    CHECK(!is_running());
    return NewTimer(::std::unique_ptr<TimerHandler>(
        new SimulatedTimerHandler(scheduler_, this, callback)));
  }

  PhasedLoopHandler *AddPhasedLoop(::std::function<void(int)> callback,
                                   const monotonic_clock::duration interval,
                                   const monotonic_clock::duration offset =
                                       ::std::chrono::seconds(0)) override {
    return NewPhasedLoop(
        ::std::unique_ptr<PhasedLoopHandler>(new SimulatedPhasedLoopHandler(
            scheduler_, this, callback, interval, offset)));
  }

  void OnRun(::std::function<void()> on_run) override {
    scheduler_->ScheduleOnRun(on_run);
  }

  void set_name(const std::string_view name) override {
    name_ = std::string(name);
  }
  const std::string_view name() const override { return name_; }

  SimulatedChannel *GetSimulatedChannel(const Channel *channel);

  void Take(const Channel *channel);

  void SetRuntimeRealtimePriority(int priority) override {
    CHECK(!is_running()) << ": Cannot set realtime priority while running.";
    priority_ = priority;
  }

  int priority() const override { return priority_; }

  void Setup() { MaybeScheduleTimingReports(); }

 private:
  friend class SimulatedTimerHandler;

  pid_t GetTid() override { return tid_; }

  EventScheduler *scheduler_;
  absl::btree_map<SimpleChannel, std::unique_ptr<SimulatedChannel>> *channels_;
  std::vector<std::pair<EventLoop *, std::function<void(bool)>>>
      *raw_event_loops_;
  absl::btree_set<SimpleChannel> taken_;

  ::std::string name_;

  int priority_ = 0;

  bool has_setup_ = false;

  const pid_t tid_;
};

void SimulatedEventLoop::MakeRawWatcher(
    const Channel *channel,
    std::function<void(const Context &channel, const void *message)> watcher) {
  ChannelIndex(channel);
  Take(channel);
  std::unique_ptr<ShmWatcher> shm_watcher(
      new ShmWatcher(this, channel, std::move(watcher)));

  GetSimulatedChannel(channel)->MakeRawWatcher(shm_watcher.get());
  NewWatcher(std::move(shm_watcher));
}

std::unique_ptr<RawSender> SimulatedEventLoop::MakeRawSender(
    const Channel *channel) {
  ChannelIndex(channel);
  Take(channel);
  return GetSimulatedChannel(channel)->MakeRawSender(this);
}

std::unique_ptr<RawFetcher> SimulatedEventLoop::MakeRawFetcher(
    const Channel *channel) {
  ChannelIndex(channel);
  return GetSimulatedChannel(channel)->MakeRawFetcher(this);
}

SimulatedChannel *SimulatedEventLoop::GetSimulatedChannel(
    const Channel *channel) {
  auto it = channels_->find(SimpleChannel(channel));
  if (it == channels_->end()) {
    it = channels_
             ->emplace(SimpleChannel(channel),
                       std::unique_ptr<SimulatedChannel>(
                           new SimulatedChannel(channel, scheduler_)))
             .first;
  }
  return it->second.get();
}

void SimulatedChannel::MakeRawWatcher(ShmWatcher *watcher) {
  watcher->SetSimulatedChannel(this);
  watchers_.emplace_back(watcher);
}

::std::unique_ptr<RawSender> SimulatedChannel::MakeRawSender(
    EventLoop *event_loop) {
  return ::std::unique_ptr<RawSender>(new SimulatedSender(this, event_loop));
}

::std::unique_ptr<RawFetcher> SimulatedChannel::MakeRawFetcher(
    EventLoop *event_loop) {
  ::std::unique_ptr<SimulatedFetcher> fetcher(
      new SimulatedFetcher(event_loop, this));
  fetchers_.push_back(fetcher.get());
  return ::std::move(fetcher);
}

void SimulatedChannel::Send(std::shared_ptr<SimulatedMessage> message) {
  message->context.queue_index = next_queue_index_.index();
  message->context.data =
      message->data() + channel()->max_size() - message->context.size;
  next_queue_index_ = next_queue_index_.Increment();

  latest_message_ = message;
  if (scheduler_->is_running()) {
    for (ShmWatcher *watcher : watchers_) {
      watcher->Schedule(scheduler_, message);
    }
  }
  for (auto &fetcher : fetchers_) {
    fetcher->Enqueue(message);
  }
}

void SimulatedChannel::UnregisterFetcher(SimulatedFetcher *fetcher) {
  fetchers_.erase(::std::find(fetchers_.begin(), fetchers_.end(), fetcher));
}

SimpleChannel::SimpleChannel(const Channel *channel)
    : name(CHECK_NOTNULL(CHECK_NOTNULL(channel)->name())->str()),
      type(CHECK_NOTNULL(CHECK_NOTNULL(channel)->type())->str()) {}

SimulatedTimerHandler::SimulatedTimerHandler(
    EventScheduler *scheduler, SimulatedEventLoop *simulated_event_loop,
    ::std::function<void()> fn)
    : TimerHandler(simulated_event_loop, std::move(fn)),
      scheduler_(scheduler),
      token_(scheduler_->InvalidToken()) {}

void SimulatedTimerHandler::Setup(monotonic_clock::time_point base,
                                  monotonic_clock::duration repeat_offset) {
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

void SimulatedTimerHandler::HandleEvent() {
  const ::aos::monotonic_clock::time_point monotonic_now =
      scheduler_->monotonic_now();
  if (repeat_offset_ != ::aos::monotonic_clock::zero()) {
    // Reschedule.
    while (base_ <= monotonic_now) base_ += repeat_offset_;
    token_ = scheduler_->Schedule(base_, [this]() { HandleEvent(); });
  } else {
    token_ = scheduler_->InvalidToken();
  }

  Call([monotonic_now]() { return monotonic_now; }, monotonic_now);
}

SimulatedPhasedLoopHandler::SimulatedPhasedLoopHandler(
    EventScheduler *scheduler, SimulatedEventLoop *simulated_event_loop,
    ::std::function<void(int)> fn, const monotonic_clock::duration interval,
    const monotonic_clock::duration offset)
    : PhasedLoopHandler(simulated_event_loop, std::move(fn), interval, offset),
      simulated_event_loop_(simulated_event_loop),
      scheduler_(scheduler),
      token_(scheduler_->InvalidToken()) {}

void SimulatedPhasedLoopHandler::HandleTimerWakeup() {
  monotonic_clock::time_point monotonic_now =
      simulated_event_loop_->monotonic_now();
  Call(
      [monotonic_now]() { return monotonic_now; },
      [this](monotonic_clock::time_point sleep_time) { Schedule(sleep_time); });
}

void SimulatedEventLoop::Take(const Channel *channel) {
  CHECK(!is_running()) << ": Cannot add new objects while running.";

  auto result = taken_.insert(SimpleChannel(channel));
  CHECK(result.second) << ": " << FlatbufferToJson(channel)
                       << " is already being used.";
}

SimulatedEventLoopFactory::SimulatedEventLoopFactory(
    const Configuration *configuration)
    : configuration_(CHECK_NOTNULL(configuration)) {}
SimulatedEventLoopFactory::~SimulatedEventLoopFactory() {}

::std::unique_ptr<EventLoop> SimulatedEventLoopFactory::MakeEventLoop(
    std::string_view name) {
  pid_t tid = tid_;
  ++tid_;
  ::std::unique_ptr<EventLoop> result(new SimulatedEventLoop(
      &scheduler_, &channels_, configuration_, &raw_event_loops_, tid));
  result->set_name(name);
  return result;
}

void SimulatedEventLoopFactory::RunFor(monotonic_clock::duration duration) {
  for (const std::pair<EventLoop *, std::function<void(bool)>> &event_loop :
       raw_event_loops_) {
    event_loop.second(true);
  }
  scheduler_.RunFor(duration);
  for (const std::pair<EventLoop *, std::function<void(bool)>> &event_loop :
       raw_event_loops_) {
    event_loop.second(false);
  }
}

void SimulatedEventLoopFactory::Run() {
  for (const std::pair<EventLoop *, std::function<void(bool)>> &event_loop :
       raw_event_loops_) {
    event_loop.second(true);
  }
  scheduler_.Run();
  for (const std::pair<EventLoop *, std::function<void(bool)>> &event_loop :
       raw_event_loops_) {
    event_loop.second(false);
  }
}

}  // namespace aos
