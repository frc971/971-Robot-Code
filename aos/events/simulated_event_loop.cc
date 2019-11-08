#include "aos/events/simulated_event_loop.h"

#include <algorithm>
#include <deque>

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

class SimulatedChannel {
 public:
  explicit SimulatedChannel(const Channel *channel, EventScheduler *scheduler)
      : channel_(CopyFlatBuffer(channel)),
        scheduler_(scheduler),
        next_queue_index_(ipc_lib::QueueIndex::Zero(channel->max_size())) {}

  ~SimulatedChannel() { CHECK_EQ(0u, fetchers_.size()); }

  // Makes a connected raw sender which calls Send below.
  ::std::unique_ptr<RawSender> MakeRawSender(EventLoop *event_loop);

  // Makes a connected raw fetcher.
  ::std::unique_ptr<RawFetcher> MakeRawFetcher();

  // Registers a watcher for the queue.
  void MakeRawWatcher(
      ::std::function<void(const Context &context, const void *message)>
          watcher);

  // Sends the message to all the connected receivers and fetchers.
  void Send(std::shared_ptr<SimulatedMessage> message);

  // Unregisters a fetcher.
  void UnregisterFetcher(SimulatedFetcher *fetcher);

  std::shared_ptr<SimulatedMessage> latest_message() { return latest_message_; }

  size_t max_size() const { return channel_.message().max_size(); }

  const absl::string_view name() const {
    return channel_.message().name()->string_view();
  }

  const Channel *channel() const { return &channel_.message(); }

 private:
  const FlatbufferDetachedBuffer<Channel> channel_;

  // List of all watchers.
  ::std::vector<
      std::function<void(const Context &context, const void *message)>>
      watchers_;

  // List of all fetchers.
  ::std::vector<SimulatedFetcher *> fetchers_;
  std::shared_ptr<SimulatedMessage> latest_message_;
  EventScheduler *scheduler_;

  ipc_lib::QueueIndex next_queue_index_;
};

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
      : RawSender(simulated_channel->channel()),
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

  bool Send(size_t length) override {
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

  bool Send(const void *msg, size_t size) override {
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

  const std::string_view name() const override {
    return simulated_channel_->name();
  }

 private:
  SimulatedChannel *simulated_channel_;
  EventLoop *event_loop_;

  std::shared_ptr<SimulatedMessage> message_;
};
}  // namespace

class SimulatedFetcher : public RawFetcher {
 public:
  explicit SimulatedFetcher(SimulatedChannel *queue)
      : RawFetcher(queue->channel()), queue_(queue) {}
  ~SimulatedFetcher() { queue_->UnregisterFetcher(this); }

  bool FetchNext() override {
    if (msgs_.size() == 0) return false;

    SetMsg(msgs_.front());
    msgs_.pop_front();
    return true;
  }

  bool Fetch() override {
    if (msgs_.size() == 0) {
      if (!msg_ && queue_->latest_message()) {
        SetMsg(queue_->latest_message());
        return true;
      } else {
        return false;
      }
    }

    // We've had a message enqueued, so we don't need to go looking for the
    // latest message from before we started.
    SetMsg(msgs_.back());
    msgs_.clear();
    return true;
  }

 private:
  friend class SimulatedChannel;

  // Updates the state inside RawFetcher to point to the data in msg_.
  void SetMsg(std::shared_ptr<SimulatedMessage> msg) {
    msg_ = msg;
    data_ = msg_->context.data;
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
                                 ::std::function<void()> fn)
      : scheduler_(scheduler), token_(scheduler_->InvalidToken()), fn_(fn) {}
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
      token_ = scheduler_->InvalidToken();
    }
    fn_();
  }

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
  // Function to be run on the thread
  ::std::function<void()> fn_;
  monotonic_clock::time_point base_;
  monotonic_clock::duration repeat_offset_;
};

class SimulatedPhasedLoopHandler : public PhasedLoopHandler {
 public:
  SimulatedPhasedLoopHandler(EventScheduler *scheduler,
                             ::std::function<void(int)> fn,
                             const monotonic_clock::duration interval,
                             const monotonic_clock::duration offset)
      : simulated_timer_handler_(scheduler, [this]() { HandleTimerWakeup(); }),
        phased_loop_(interval, simulated_timer_handler_.monotonic_now(),
                     offset),
        fn_(fn) {
    // TODO(austin): This assumes time doesn't change between when the
    // constructor is called and when we start running.  It's probably a safe
    // assumption.
    Reschedule();
  }

  void HandleTimerWakeup() {
    fn_(cycles_elapsed_);
    Reschedule();
  }

  void set_interval_and_offset(
      const monotonic_clock::duration interval,
      const monotonic_clock::duration offset) override {
    phased_loop_.set_interval_and_offset(interval, offset);
  }

  void Reschedule() {
    cycles_elapsed_ =
        phased_loop_.Iterate(simulated_timer_handler_.monotonic_now());
    simulated_timer_handler_.Setup(phased_loop_.sleep_time(),
                                   ::aos::monotonic_clock::zero());
  }

 private:
  SimulatedTimerHandler simulated_timer_handler_;

  time::PhasedLoop phased_loop_;

  int cycles_elapsed_ = 1;

  ::std::function<void(int)> fn_;
};

class SimulatedEventLoop : public EventLoop {
 public:
  explicit SimulatedEventLoop(
      EventScheduler *scheduler,
      absl::btree_map<SimpleChannel, std::unique_ptr<SimulatedChannel>>
          *channels,
      const Configuration *configuration,
      std::vector<std::pair<EventLoop *, std::function<void(bool)>>>
          *raw_event_loops)
      : EventLoop(configuration),
        scheduler_(scheduler),
        channels_(channels),
        raw_event_loops_(raw_event_loops) {
    raw_event_loops_->push_back(
        std::make_pair(this, [this](bool value) { set_is_running(value); }));
  }
  ~SimulatedEventLoop() override {
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
    timers_.emplace_back(new SimulatedTimerHandler(scheduler_, callback));
    return timers_.back().get();
  }

  PhasedLoopHandler *AddPhasedLoop(::std::function<void(int)> callback,
                                   const monotonic_clock::duration interval,
                                   const monotonic_clock::duration offset =
                                       ::std::chrono::seconds(0)) override {
    phased_loops_.emplace_back(
        new SimulatedPhasedLoopHandler(scheduler_, callback, interval, offset));
    return phased_loops_.back().get();
  }

  void OnRun(::std::function<void()> on_run) override {
    scheduler_->Schedule(scheduler_->monotonic_now(), on_run);
  }

  void set_name(const std::string_view name) override {
    name_ = std::string(name);
  }
  const std::string_view name() const override { return name_; }

  SimulatedChannel *GetSimulatedChannel(const Channel *channel);

  void Take(const Channel *channel);

  void SetRuntimeRealtimePriority(int /*priority*/) override {
    CHECK(!is_running()) << ": Cannot set realtime priority while running.";
  }

 private:
  EventScheduler *scheduler_;
  absl::btree_map<SimpleChannel, std::unique_ptr<SimulatedChannel>> *channels_;
  std::vector<std::pair<EventLoop *, std::function<void(bool)>>>
      *raw_event_loops_;
  absl::btree_set<SimpleChannel> taken_;
  ::std::vector<std::unique_ptr<TimerHandler>> timers_;
  ::std::vector<std::unique_ptr<PhasedLoopHandler>> phased_loops_;

  ::std::string name_;
};

void SimulatedEventLoop::MakeRawWatcher(
    const Channel *channel,
    std::function<void(const Context &channel, const void *message)> watcher) {
  ValidateChannel(channel);
  Take(channel);
  GetSimulatedChannel(channel)->MakeRawWatcher(watcher);
}

std::unique_ptr<RawSender> SimulatedEventLoop::MakeRawSender(
    const Channel *channel) {
  ValidateChannel(channel);
  Take(channel);
  return GetSimulatedChannel(channel)->MakeRawSender(this);
}

std::unique_ptr<RawFetcher> SimulatedEventLoop::MakeRawFetcher(
    const Channel *channel) {
  ValidateChannel(channel);
  return GetSimulatedChannel(channel)->MakeRawFetcher();
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

void SimulatedChannel::MakeRawWatcher(
    ::std::function<void(const Context &context, const void *message)>
        watcher) {
  watchers_.push_back(watcher);
}

::std::unique_ptr<RawSender> SimulatedChannel::MakeRawSender(
    EventLoop *event_loop) {
  return ::std::unique_ptr<RawSender>(new SimulatedSender(this, event_loop));
}

::std::unique_ptr<RawFetcher> SimulatedChannel::MakeRawFetcher() {
  ::std::unique_ptr<SimulatedFetcher> fetcher(new SimulatedFetcher(this));
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
    for (auto &watcher : watchers_) {
      scheduler_->Schedule(scheduler_->monotonic_now(), [watcher, message]() {
        watcher(message->context, message->context.data);
      });
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

void SimulatedEventLoop::Take(const Channel *channel) {
  CHECK(!is_running()) << ": Cannot add new objects while running.";

  auto result = taken_.insert(SimpleChannel(channel));
  CHECK(result.second) << ": " << FlatbufferToJson(channel)
                       << " is already being used.";
}

SimulatedEventLoopFactory::SimulatedEventLoopFactory(
    const Configuration *configuration)
    : configuration_(configuration) {}
SimulatedEventLoopFactory::~SimulatedEventLoopFactory() {}

::std::unique_ptr<EventLoop> SimulatedEventLoopFactory::MakeEventLoop() {
  return ::std::unique_ptr<EventLoop>(new SimulatedEventLoop(
      &scheduler_, &channels_, configuration_, &raw_event_loops_));
}

void SimulatedEventLoopFactory::RunFor(monotonic_clock::duration duration) {
  for (const std::pair<EventLoop *, std::function<void(bool)>> &event_loop :
       raw_event_loops_) {
    event_loop.second(true);
  }
  scheduler_.RunFor(duration);
  if (!scheduler_.is_running()) {
    for (const std::pair<EventLoop *, std::function<void(bool)>> &event_loop :
         raw_event_loops_) {
      event_loop.second(false);
    }
  }
}

void SimulatedEventLoopFactory::Run() {
  for (const std::pair<EventLoop *, std::function<void(bool)>> &event_loop :
       raw_event_loops_) {
    event_loop.second(true);
  }
  scheduler_.Run();
  if (!scheduler_.is_running()) {
    for (const std::pair<EventLoop *, std::function<void(bool)>> &event_loop :
         raw_event_loops_) {
      event_loop.second(false);
    }
  }
}

}  // namespace aos
