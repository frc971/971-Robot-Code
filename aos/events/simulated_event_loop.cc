#include "aos/events/simulated_event_loop.h"

#include <algorithm>
#include <deque>
#include <string_view>
#include <vector>

#include "absl/container/btree_map.h"
#include "aos/events/aos_logging.h"
#include "aos/events/simulated_network_bridge.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/realtime.h"
#include "aos/util/phased_loop.h"

namespace aos {

class SimulatedEventLoop;
class SimulatedFetcher;
class SimulatedChannel;

namespace {

class ScopedMarkRealtimeRestorer {
 public:
  ScopedMarkRealtimeRestorer(bool rt) : rt_(rt), prior_(MarkRealtime(rt)) {}
  ~ScopedMarkRealtimeRestorer() { CHECK_EQ(rt_, MarkRealtime(prior_)); }

 private:
  const bool rt_;
  const bool prior_;
};

// Container for both a message, and the context for it for simulation.  This
// makes tracking the timestamps associated with the data easy.
struct SimulatedMessage final {
  SimulatedMessage(const SimulatedMessage &) = delete;
  SimulatedMessage &operator=(const SimulatedMessage &) = delete;

  // Creates a SimulatedMessage with size bytes of storage.
  // This is a shared_ptr so we don't have to implement refcounting or copying.
  static std::shared_ptr<SimulatedMessage> Make(SimulatedChannel *channel);

  // Context for the data.
  Context context;

  SimulatedChannel *const channel = nullptr;

  // The data.
  char *data(size_t buffer_size) {
    return RoundChannelData(&actual_data[0], buffer_size);
  }

  // Then the data, including padding on the end so we can align the buffer we
  // actually return from data().
  char actual_data[];

 private:
  SimulatedMessage(SimulatedChannel *channel_in);
  ~SimulatedMessage();

  static void DestroyAndFree(SimulatedMessage *p) {
    p->~SimulatedMessage();
    free(p);
  }
};

}  // namespace

// TODO(Brian): This should be in the anonymous namespace, but that annoys GCC
// for some reason...
class SimulatedWatcher : public WatcherState {
 public:
  SimulatedWatcher(
      SimulatedEventLoop *simulated_event_loop, EventScheduler *scheduler,
      const Channel *channel,
      std::function<void(const Context &context, const void *message)> fn);

  ~SimulatedWatcher() override;

  bool has_run() const;

  void Startup(EventLoop * /*event_loop*/) override {}

  void Schedule(std::shared_ptr<SimulatedMessage> message);

  void HandleEvent();

  void SetSimulatedChannel(SimulatedChannel *channel) {
    simulated_channel_ = channel;
  }

 private:
  void DoSchedule(monotonic_clock::time_point event_time);

  ::std::deque<std::shared_ptr<SimulatedMessage>> msgs_;

  SimulatedEventLoop *const simulated_event_loop_;
  const Channel *const channel_;
  EventScheduler *const scheduler_;
  EventHandler<SimulatedWatcher> event_;
  EventScheduler::Token token_;
  SimulatedChannel *simulated_channel_ = nullptr;
};

class SimulatedChannel {
 public:
  explicit SimulatedChannel(const Channel *channel,
                            std::chrono::nanoseconds channel_storage_duration)
      : channel_(channel),
        channel_storage_duration_(channel_storage_duration),
        next_queue_index_(ipc_lib::QueueIndex::Zero(number_buffers())) {
    available_buffer_indices_.reserve(number_buffers());
    for (int i = 0; i < number_buffers(); ++i) {
      available_buffer_indices_.push_back(i);
    }
  }

  ~SimulatedChannel() {
    latest_message_.reset();
    CHECK_EQ(static_cast<size_t>(number_buffers()),
             available_buffer_indices_.size());
    CHECK_EQ(0u, fetchers_.size())
        << configuration::StrippedChannelToString(channel());
    CHECK_EQ(0u, watchers_.size())
        << configuration::StrippedChannelToString(channel());
    CHECK_EQ(0, sender_count_)
        << configuration::StrippedChannelToString(channel());
  }

  // The number of messages we pretend to have in the queue.
  int queue_size() const {
    return channel()->frequency() *
           std::chrono::duration_cast<std::chrono::duration<double>>(
               channel_storage_duration_)
               .count();
  }

  // The number of extra buffers (beyond the queue) we pretend to have.
  int number_scratch_buffers() const {
    // We need to start creating messages before we know how many
    // senders+readers we'll have, so we need to just pick something which is
    // always big enough.
    return 50;
  }

  int number_buffers() const { return queue_size() + number_scratch_buffers(); }

  int GetBufferIndex() {
    CHECK(!available_buffer_indices_.empty()) << ": This should be impossible";
    const int result = available_buffer_indices_.back();
    available_buffer_indices_.pop_back();
    return result;
  }

  void FreeBufferIndex(int i) {
    // This extra checking has a large performance hit with sanitizers that
    // track memory accesses, so just skip it.
#if !__has_feature(memory_sanitizer) && !__has_feature(address_sanitizer)
    DCHECK(std::find(available_buffer_indices_.begin(),
                     available_buffer_indices_.end(),
                     i) == available_buffer_indices_.end())
        << ": Buffer is not in use: " << i;
#endif
    available_buffer_indices_.push_back(i);
  }

  // Makes a connected raw sender which calls Send below.
  ::std::unique_ptr<RawSender> MakeRawSender(SimulatedEventLoop *event_loop);

  // Makes a connected raw fetcher.
  ::std::unique_ptr<RawFetcher> MakeRawFetcher(EventLoop *event_loop);

  // Registers a watcher for the queue.
  void MakeRawWatcher(SimulatedWatcher *watcher);

  void RemoveWatcher(SimulatedWatcher *watcher) {
    watchers_.erase(std::find(watchers_.begin(), watchers_.end(), watcher));
  }

  // Sends the message to all the connected receivers and fetchers.  Returns the
  // sent queue index.
  uint32_t Send(std::shared_ptr<SimulatedMessage> message);

  // Unregisters a fetcher.
  void UnregisterFetcher(SimulatedFetcher *fetcher);

  std::shared_ptr<SimulatedMessage> latest_message() { return latest_message_; }

  size_t max_size() const { return channel()->max_size(); }

  const std::string_view name() const {
    return channel()->name()->string_view();
  }

  const Channel *channel() const { return channel_; }

  void CountSenderCreated() {
    CheckBufferCount();
    if (sender_count_ >= channel()->num_senders()) {
      LOG(FATAL) << "Failed to create sender on "
                 << configuration::CleanedChannelToString(channel())
                 << ", too many senders.";
    }
    ++sender_count_;
  }

  void CountSenderDestroyed() {
    --sender_count_;
    CHECK_GE(sender_count_, 0);
  }

 private:
  void CheckBufferCount() {
    int reader_count = 0;
    if (channel()->read_method() == ReadMethod::PIN) {
      reader_count = watchers_.size() + fetchers_.size();
    }
    CHECK_LT(reader_count + sender_count_, number_scratch_buffers());
  }

  void CheckReaderCount() {
    if (channel()->read_method() != ReadMethod::PIN) {
      return;
    }
    CheckBufferCount();
    const int reader_count = watchers_.size() + fetchers_.size();
    if (reader_count >= channel()->num_readers()) {
      LOG(FATAL) << "Failed to create reader on "
                 << configuration::CleanedChannelToString(channel())
                 << ", too many readers.";
    }
  }

  const Channel *const channel_;
  const std::chrono::nanoseconds channel_storage_duration_;

  // List of all watchers.
  ::std::vector<SimulatedWatcher *> watchers_;

  // List of all fetchers.
  ::std::vector<SimulatedFetcher *> fetchers_;
  std::shared_ptr<SimulatedMessage> latest_message_;

  ipc_lib::QueueIndex next_queue_index_;

  int sender_count_ = 0;

  std::vector<uint16_t> available_buffer_indices_;
};

namespace {

std::shared_ptr<SimulatedMessage> SimulatedMessage::Make(
    SimulatedChannel *channel) {
  // The allocations in here are due to infrastructure and don't count in the no
  // mallocs in RT code.
  ScopedNotRealtime nrt;
  const size_t size = channel->max_size();
  SimulatedMessage *const message = reinterpret_cast<SimulatedMessage *>(
      malloc(sizeof(SimulatedMessage) + size + kChannelDataAlignment - 1));
  new (message) SimulatedMessage(channel);
  message->context.size = size;
  message->context.data = message->data(size);

  return std::shared_ptr<SimulatedMessage>(message,
                                           &SimulatedMessage::DestroyAndFree);
}

SimulatedMessage::SimulatedMessage(SimulatedChannel *channel_in)
    : channel(channel_in) {
  context.buffer_index = channel->GetBufferIndex();
}

SimulatedMessage::~SimulatedMessage() {
  channel->FreeBufferIndex(context.buffer_index);
}

class SimulatedSender : public RawSender {
 public:
  SimulatedSender(SimulatedChannel *simulated_channel,
                  SimulatedEventLoop *event_loop);
  ~SimulatedSender() override;

  void *data() override {
    if (!message_) {
      message_ = SimulatedMessage::Make(simulated_channel_);
    }
    return message_->data(simulated_channel_->max_size());
  }

  size_t size() override { return simulated_channel_->max_size(); }

  bool DoSend(size_t length,
              aos::monotonic_clock::time_point monotonic_remote_time,
              aos::realtime_clock::time_point realtime_remote_time,
              uint32_t remote_queue_index,
              const UUID &remote_boot_uuid) override;

  bool DoSend(const void *msg, size_t size,
              aos::monotonic_clock::time_point monotonic_remote_time,
              aos::realtime_clock::time_point realtime_remote_time,
              uint32_t remote_queue_index,
              const UUID &remote_boot_uuid) override;

  int buffer_index() override {
    // First, ensure message_ is allocated.
    data();
    return message_->context.buffer_index;
  }

 private:
  SimulatedChannel *simulated_channel_;
  SimulatedEventLoop *event_loop_;

  std::shared_ptr<SimulatedMessage> message_;
};
}  // namespace

class SimulatedFetcher : public RawFetcher {
 public:
  explicit SimulatedFetcher(EventLoop *event_loop,
                            SimulatedChannel *simulated_channel)
      : RawFetcher(event_loop, simulated_channel->channel()),
        simulated_channel_(simulated_channel) {}
  ~SimulatedFetcher() { simulated_channel_->UnregisterFetcher(this); }

  std::pair<bool, monotonic_clock::time_point> DoFetchNext() override {
    // The allocations in here are due to infrastructure and don't count in the
    // no mallocs in RT code.
    ScopedNotRealtime nrt;
    if (msgs_.size() == 0) {
      return std::make_pair(false, monotonic_clock::min_time);
    }

    CHECK(!fell_behind_) << ": Got behind on "
                         << configuration::StrippedChannelToString(
                                simulated_channel_->channel());

    SetMsg(msgs_.front());
    msgs_.pop_front();
    return std::make_pair(true, event_loop()->monotonic_now());
  }

  std::pair<bool, monotonic_clock::time_point> DoFetch() override {
    // The allocations in here are due to infrastructure and don't count in the
    // no mallocs in RT code.
    ScopedNotRealtime nrt;
    if (msgs_.size() == 0) {
      // TODO(austin): Can we just do this logic unconditionally?  It is a lot
      // simpler.  And call clear, obviously.
      if (!msg_ && simulated_channel_->latest_message()) {
        SetMsg(simulated_channel_->latest_message());
        return std::make_pair(true, event_loop()->monotonic_now());
      } else {
        return std::make_pair(false, monotonic_clock::min_time);
      }
    }

    // We've had a message enqueued, so we don't need to go looking for the
    // latest message from before we started.
    SetMsg(msgs_.back());
    msgs_.clear();
    fell_behind_ = false;
    return std::make_pair(true, event_loop()->monotonic_now());
  }

 private:
  friend class SimulatedChannel;

  // Updates the state inside RawFetcher to point to the data in msg_.
  void SetMsg(std::shared_ptr<SimulatedMessage> msg) {
    msg_ = msg;
    context_ = msg_->context;
    if (channel()->read_method() != ReadMethod::PIN) {
      context_.buffer_index = -1;
    }
    if (context_.remote_queue_index == 0xffffffffu) {
      context_.remote_queue_index = context_.queue_index;
    }
    if (context_.monotonic_remote_time == aos::monotonic_clock::min_time) {
      context_.monotonic_remote_time = context_.monotonic_event_time;
    }
    if (context_.realtime_remote_time == aos::realtime_clock::min_time) {
      context_.realtime_remote_time = context_.realtime_event_time;
    }
  }

  // Internal method for Simulation to add a message to the buffer.
  void Enqueue(std::shared_ptr<SimulatedMessage> buffer) {
    msgs_.emplace_back(buffer);
    if (fell_behind_ ||
        msgs_.size() > static_cast<size_t>(simulated_channel_->queue_size())) {
      fell_behind_ = true;
      // Might as well empty out all the intermediate messages now.
      while (msgs_.size() > 1) {
        msgs_.pop_front();
      }
    }
  }

  SimulatedChannel *simulated_channel_;
  std::shared_ptr<SimulatedMessage> msg_;

  // Messages queued up but not in use.
  ::std::deque<std::shared_ptr<SimulatedMessage>> msgs_;

  // Whether we're currently "behind", which means a FetchNext call will fail.
  bool fell_behind_ = false;
};

class SimulatedTimerHandler : public TimerHandler {
 public:
  explicit SimulatedTimerHandler(EventScheduler *scheduler,
                                 SimulatedEventLoop *simulated_event_loop,
                                 ::std::function<void()> fn);
  ~SimulatedTimerHandler() { Disable(); }

  void Setup(monotonic_clock::time_point base,
             monotonic_clock::duration repeat_offset) override;

  void HandleEvent();

  void Disable() override;

 private:
  SimulatedEventLoop *simulated_event_loop_;
  EventHandler<SimulatedTimerHandler> event_;
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
  ~SimulatedPhasedLoopHandler();

  void HandleEvent();

  void Schedule(monotonic_clock::time_point sleep_time) override;

 private:
  SimulatedEventLoop *simulated_event_loop_;
  EventHandler<SimulatedPhasedLoopHandler> event_;

  EventScheduler *scheduler_;
  EventScheduler::Token token_;
};

class SimulatedEventLoop : public EventLoop {
 public:
  explicit SimulatedEventLoop(
      EventScheduler *scheduler, NodeEventLoopFactory *node_event_loop_factory,
      absl::btree_map<SimpleChannel, std::unique_ptr<SimulatedChannel>>
          *channels,
      const Configuration *configuration,
      std::vector<std::pair<EventLoop *, std::function<void(bool)>>>
          *raw_event_loops,
      const Node *node, pid_t tid)
      : EventLoop(CHECK_NOTNULL(configuration)),
        scheduler_(scheduler),
        node_event_loop_factory_(node_event_loop_factory),
        channels_(channels),
        raw_event_loops_(raw_event_loops),
        node_(node),
        tid_(tid) {
    raw_event_loops_->push_back(std::make_pair(this, [this](bool value) {
      if (!has_setup_) {
        Setup();
        has_setup_ = true;
      }
      set_is_running(value);
      has_run_ = true;
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

  bool has_run() const { return has_run_; }

  std::chrono::nanoseconds send_delay() const { return send_delay_; }
  void set_send_delay(std::chrono::nanoseconds send_delay) {
    send_delay_ = send_delay;
  }

  ::aos::monotonic_clock::time_point monotonic_now() override {
    return node_event_loop_factory_->monotonic_now();
  }

  ::aos::realtime_clock::time_point realtime_now() override {
    return node_event_loop_factory_->realtime_now();
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
    CHECK(!is_running()) << ": Cannot register OnRun callback while running.";
    scheduler_->ScheduleOnRun([this, on_run = std::move(on_run)]() {
      ScopedMarkRealtimeRestorer rt(priority() > 0);
      on_run();
    });
  }

  const Node *node() const override { return node_; }

  void set_name(const std::string_view name) override {
    name_ = std::string(name);
  }
  const std::string_view name() const override { return name_; }

  SimulatedChannel *GetSimulatedChannel(const Channel *channel);

  void SetRuntimeRealtimePriority(int priority) override {
    CHECK(!is_running()) << ": Cannot set realtime priority while running.";
    priority_ = priority;
  }

  int priority() const override { return priority_; }

  void SetRuntimeAffinity(const cpu_set_t & /*cpuset*/) override {
    CHECK(!is_running()) << ": Cannot set affinity while running.";
  }

  void Setup() {
    MaybeScheduleTimingReports();
    if (!skip_logger_) {
      log_sender_.Initialize(MakeSender<logging::LogMessageFbs>("/aos"));
      log_impl_ = log_sender_.implementation();
    }
  }

  int NumberBuffers(const Channel *channel) override;

  const UUID &boot_uuid() const override {
    return node_event_loop_factory_->boot_uuid();
  }

 private:
  friend class SimulatedTimerHandler;
  friend class SimulatedPhasedLoopHandler;
  friend class SimulatedWatcher;

  void HandleEvent() {
    while (true) {
      if (EventCount() == 0 || PeekEvent()->event_time() > monotonic_now()) {
        break;
      }

      EventLoopEvent *event = PopEvent();
      event->HandleEvent();
    }
  }

  pid_t GetTid() override { return tid_; }

  EventScheduler *scheduler_;
  NodeEventLoopFactory *node_event_loop_factory_;
  absl::btree_map<SimpleChannel, std::unique_ptr<SimulatedChannel>> *channels_;
  std::vector<std::pair<EventLoop *, std::function<void(bool)>>>
      *raw_event_loops_;

  ::std::string name_;

  int priority_ = 0;

  bool has_setup_ = false;

  std::chrono::nanoseconds send_delay_;

  const Node *const node_;
  const pid_t tid_;

  AosLogToFbs log_sender_;
  std::shared_ptr<logging::LogImplementation> log_impl_ = nullptr;

  bool has_run_ = false;
};

void SimulatedEventLoopFactory::set_send_delay(
    std::chrono::nanoseconds send_delay) {
  send_delay_ = send_delay;
  for (std::pair<EventLoop *, std::function<void(bool)>> &loop :
       raw_event_loops_) {
    reinterpret_cast<SimulatedEventLoop *>(loop.first)
        ->set_send_delay(send_delay_);
  }
}

void SimulatedEventLoop::MakeRawWatcher(
    const Channel *channel,
    std::function<void(const Context &channel, const void *message)> watcher) {
  TakeWatcher(channel);

  std::unique_ptr<SimulatedWatcher> shm_watcher(
      new SimulatedWatcher(this, scheduler_, channel, std::move(watcher)));

  GetSimulatedChannel(channel)->MakeRawWatcher(shm_watcher.get());
  NewWatcher(std::move(shm_watcher));

  // Order of operations gets kinda wonky if we let people make watchers after
  // running once.  If someone has a valid use case, we can reconsider.
  CHECK(!has_run()) << ": Can't add a watcher after running.";
}

std::unique_ptr<RawSender> SimulatedEventLoop::MakeRawSender(
    const Channel *channel) {
  TakeSender(channel);

  return GetSimulatedChannel(channel)->MakeRawSender(this);
}

std::unique_ptr<RawFetcher> SimulatedEventLoop::MakeRawFetcher(
    const Channel *channel) {
  ChannelIndex(channel);

  if (!configuration::ChannelIsReadableOnNode(channel, node())) {
    LOG(FATAL) << "Channel { \"name\": \"" << channel->name()->string_view()
               << "\", \"type\": \"" << channel->type()->string_view()
               << "\" } is not able to be fetched on this node.  Check your "
                  "configuration.";
  }

  return GetSimulatedChannel(channel)->MakeRawFetcher(this);
}

SimulatedChannel *SimulatedEventLoop::GetSimulatedChannel(
    const Channel *channel) {
  auto it = channels_->find(SimpleChannel(channel));
  if (it == channels_->end()) {
    it =
        channels_
            ->emplace(
                SimpleChannel(channel),
                std::unique_ptr<SimulatedChannel>(new SimulatedChannel(
                    channel, std::chrono::nanoseconds(
                                 configuration()->channel_storage_duration()))))
            .first;
  }
  return it->second.get();
}

int SimulatedEventLoop::NumberBuffers(const Channel *channel) {
  return GetSimulatedChannel(channel)->number_buffers();
}

SimulatedWatcher::SimulatedWatcher(
    SimulatedEventLoop *simulated_event_loop, EventScheduler *scheduler,
    const Channel *channel,
    std::function<void(const Context &context, const void *message)> fn)
    : WatcherState(simulated_event_loop, channel, std::move(fn)),
      simulated_event_loop_(simulated_event_loop),
      channel_(channel),
      scheduler_(scheduler),
      event_(this),
      token_(scheduler_->InvalidToken()) {}

SimulatedWatcher::~SimulatedWatcher() {
  simulated_event_loop_->RemoveEvent(&event_);
  if (token_ != scheduler_->InvalidToken()) {
    scheduler_->Deschedule(token_);
  }
  CHECK_NOTNULL(simulated_channel_)->RemoveWatcher(this);
}

bool SimulatedWatcher::has_run() const {
  return simulated_event_loop_->has_run();
}

void SimulatedWatcher::Schedule(std::shared_ptr<SimulatedMessage> message) {
  monotonic_clock::time_point event_time =
      simulated_event_loop_->monotonic_now();

  // Messages are queued in order.  If we are the first, add ourselves.
  // Otherwise, don't.
  if (msgs_.size() == 0) {
    event_.set_event_time(message->context.monotonic_event_time);
    simulated_event_loop_->AddEvent(&event_);

    DoSchedule(event_time);
  }

  msgs_.emplace_back(message);
}

void SimulatedWatcher::HandleEvent() {
  VLOG(1) << "Watcher " << configuration::CleanedChannelToString(channel_);
  CHECK_NE(msgs_.size(), 0u) << ": No events to handle.";

  const monotonic_clock::time_point monotonic_now =
      simulated_event_loop_->monotonic_now();
  logging::ScopedLogRestorer prev_logger;
  if (simulated_event_loop_->log_impl_) {
    prev_logger.Swap(simulated_event_loop_->log_impl_);
  }
  Context context = msgs_.front()->context;

  if (channel_->read_method() != ReadMethod::PIN) {
    context.buffer_index = -1;
  }
  if (context.remote_queue_index == 0xffffffffu) {
    context.remote_queue_index = context.queue_index;
  }
  if (context.monotonic_remote_time == aos::monotonic_clock::min_time) {
    context.monotonic_remote_time = context.monotonic_event_time;
  }
  if (context.realtime_remote_time == aos::realtime_clock::min_time) {
    context.realtime_remote_time = context.realtime_event_time;
  }

  {
    ScopedMarkRealtimeRestorer rt(simulated_event_loop_->priority() > 0);
    DoCallCallback([monotonic_now]() { return monotonic_now; }, context);
  }

  msgs_.pop_front();
  if (token_ != scheduler_->InvalidToken()) {
    scheduler_->Deschedule(token_);
    token_ = scheduler_->InvalidToken();
  }
  if (msgs_.size() != 0) {
    event_.set_event_time(msgs_.front()->context.monotonic_event_time);
    simulated_event_loop_->AddEvent(&event_);

    DoSchedule(event_.event_time());
  }
}

void SimulatedWatcher::DoSchedule(monotonic_clock::time_point event_time) {
  CHECK(token_ == scheduler_->InvalidToken())
      << ": May not schedule multiple times";
  token_ = scheduler_->Schedule(
      event_time + simulated_event_loop_->send_delay(), [this]() {
        DCHECK(token_ != scheduler_->InvalidToken());
        token_ = scheduler_->InvalidToken();
        simulated_event_loop_->HandleEvent();
      });
}

void SimulatedChannel::MakeRawWatcher(SimulatedWatcher *watcher) {
  CheckReaderCount();
  watcher->SetSimulatedChannel(this);
  watchers_.emplace_back(watcher);
}

::std::unique_ptr<RawSender> SimulatedChannel::MakeRawSender(
    SimulatedEventLoop *event_loop) {
  return ::std::unique_ptr<RawSender>(new SimulatedSender(this, event_loop));
}

::std::unique_ptr<RawFetcher> SimulatedChannel::MakeRawFetcher(
    EventLoop *event_loop) {
  CheckReaderCount();
  ::std::unique_ptr<SimulatedFetcher> fetcher(
      new SimulatedFetcher(event_loop, this));
  fetchers_.push_back(fetcher.get());
  return ::std::move(fetcher);
}

uint32_t SimulatedChannel::Send(std::shared_ptr<SimulatedMessage> message) {
  const uint32_t queue_index = next_queue_index_.index();
  message->context.queue_index = queue_index;
  message->context.data = message->data(channel()->max_size()) +
                          channel()->max_size() - message->context.size;

  DCHECK(channel()->has_schema())
      << ": Missing schema for channel "
      << configuration::StrippedChannelToString(channel());
  DCHECK(flatbuffers::Verify(
      *channel()->schema(), *channel()->schema()->root_table(),
      static_cast<const uint8_t *>(message->context.data),
      message->context.size))
      << ": Corrupted flatbuffer on " << channel()->name()->c_str() << " "
      << channel()->type()->c_str();

  next_queue_index_ = next_queue_index_.Increment();

  latest_message_ = message;
  for (SimulatedWatcher *watcher : watchers_) {
    if (watcher->has_run()) {
      watcher->Schedule(message);
    }
  }
  for (auto &fetcher : fetchers_) {
    fetcher->Enqueue(message);
  }

  return queue_index;
}

void SimulatedChannel::UnregisterFetcher(SimulatedFetcher *fetcher) {
  fetchers_.erase(::std::find(fetchers_.begin(), fetchers_.end(), fetcher));
}

SimulatedSender::SimulatedSender(SimulatedChannel *simulated_channel,
                                 SimulatedEventLoop *event_loop)
    : RawSender(event_loop, simulated_channel->channel()),
      simulated_channel_(simulated_channel),
      event_loop_(event_loop) {
  simulated_channel_->CountSenderCreated();
}

SimulatedSender::~SimulatedSender() {
  simulated_channel_->CountSenderDestroyed();
}

bool SimulatedSender::DoSend(size_t length,
                             monotonic_clock::time_point monotonic_remote_time,
                             realtime_clock::time_point realtime_remote_time,
                             uint32_t remote_queue_index,
                             const UUID &remote_boot_uuid) {
  // The allocations in here are due to infrastructure and don't count in the
  // no mallocs in RT code.
  ScopedNotRealtime nrt;
  CHECK_LE(length, size()) << ": Attempting to send too big a message.";
  message_->context.monotonic_event_time = event_loop_->monotonic_now();
  message_->context.monotonic_remote_time = monotonic_remote_time;
  message_->context.remote_queue_index = remote_queue_index;
  message_->context.realtime_event_time = event_loop_->realtime_now();
  message_->context.realtime_remote_time = realtime_remote_time;
  message_->context.remote_boot_uuid = remote_boot_uuid;
  CHECK_LE(length, message_->context.size);
  message_->context.size = length;

  // TODO(austin): Track sending too fast.
  sent_queue_index_ = simulated_channel_->Send(message_);
  monotonic_sent_time_ = event_loop_->monotonic_now();
  realtime_sent_time_ = event_loop_->realtime_now();

  // Drop the reference to the message so that we allocate a new message for
  // next time.  Otherwise we will continue to reuse the same memory for all
  // messages and corrupt it.
  message_.reset();
  return true;
}

bool SimulatedSender::DoSend(const void *msg, size_t size,
                             monotonic_clock::time_point monotonic_remote_time,
                             realtime_clock::time_point realtime_remote_time,
                             uint32_t remote_queue_index,
                             const UUID &remote_boot_uuid) {
  CHECK_LE(size, this->size())
      << ": Attempting to send too big a message on "
      << configuration::CleanedChannelToString(simulated_channel_->channel());

  // This is wasteful, but since flatbuffers fill from the back end of the
  // queue, we need it to be full sized.
  message_ = SimulatedMessage::Make(simulated_channel_);

  // Now fill in the message.  size is already populated above, and
  // queue_index will be populated in simulated_channel_.  Put this at the
  // back of the data segment.
  memcpy(message_->data(simulated_channel_->max_size()) +
             simulated_channel_->max_size() - size,
         msg, size);

  return DoSend(size, monotonic_remote_time, realtime_remote_time,
                remote_queue_index, remote_boot_uuid);
}

SimulatedTimerHandler::SimulatedTimerHandler(
    EventScheduler *scheduler, SimulatedEventLoop *simulated_event_loop,
    ::std::function<void()> fn)
    : TimerHandler(simulated_event_loop, std::move(fn)),
      simulated_event_loop_(simulated_event_loop),
      event_(this),
      scheduler_(scheduler),
      token_(scheduler_->InvalidToken()) {}

void SimulatedTimerHandler::Setup(monotonic_clock::time_point base,
                                  monotonic_clock::duration repeat_offset) {
  // The allocations in here are due to infrastructure and don't count in the no
  // mallocs in RT code.
  ScopedNotRealtime nrt;
  Disable();
  const ::aos::monotonic_clock::time_point monotonic_now =
      simulated_event_loop_->monotonic_now();
  base_ = base;
  repeat_offset_ = repeat_offset;
  token_ = scheduler_->Schedule(std::max(base, monotonic_now), [this]() {
    DCHECK(token_ != scheduler_->InvalidToken());
    token_ = scheduler_->InvalidToken();
    simulated_event_loop_->HandleEvent();
  });
  event_.set_event_time(base_);
  simulated_event_loop_->AddEvent(&event_);
}

void SimulatedTimerHandler::HandleEvent() {
  VLOG(1) << "Timer " << name();
  const ::aos::monotonic_clock::time_point monotonic_now =
      simulated_event_loop_->monotonic_now();
  logging::ScopedLogRestorer prev_logger;
  if (simulated_event_loop_->log_impl_) {
    prev_logger.Swap(simulated_event_loop_->log_impl_);
  }
  if (token_ != scheduler_->InvalidToken()) {
    scheduler_->Deschedule(token_);
    token_ = scheduler_->InvalidToken();
  }
  if (repeat_offset_ != ::aos::monotonic_clock::zero()) {
    // Reschedule.
    while (base_ <= monotonic_now) base_ += repeat_offset_;
    token_ = scheduler_->Schedule(base_, [this]() {
      DCHECK(token_ != scheduler_->InvalidToken());
      token_ = scheduler_->InvalidToken();
      simulated_event_loop_->HandleEvent();
    });
    event_.set_event_time(base_);
    simulated_event_loop_->AddEvent(&event_);
  }

  {
    ScopedMarkRealtimeRestorer rt(simulated_event_loop_->priority() > 0);
    Call([monotonic_now]() { return monotonic_now; }, monotonic_now);
  }
}

void SimulatedTimerHandler::Disable() {
  simulated_event_loop_->RemoveEvent(&event_);
  if (token_ != scheduler_->InvalidToken()) {
    scheduler_->Deschedule(token_);
    token_ = scheduler_->InvalidToken();
  }
}

SimulatedPhasedLoopHandler::SimulatedPhasedLoopHandler(
    EventScheduler *scheduler, SimulatedEventLoop *simulated_event_loop,
    ::std::function<void(int)> fn, const monotonic_clock::duration interval,
    const monotonic_clock::duration offset)
    : PhasedLoopHandler(simulated_event_loop, std::move(fn), interval, offset),
      simulated_event_loop_(simulated_event_loop),
      event_(this),
      scheduler_(scheduler),
      token_(scheduler_->InvalidToken()) {}

SimulatedPhasedLoopHandler::~SimulatedPhasedLoopHandler() {
  if (token_ != scheduler_->InvalidToken()) {
    scheduler_->Deschedule(token_);
    token_ = scheduler_->InvalidToken();
  }
  simulated_event_loop_->RemoveEvent(&event_);
}

void SimulatedPhasedLoopHandler::HandleEvent() {
  VLOG(1) << "Phased loop " << name();
  monotonic_clock::time_point monotonic_now =
      simulated_event_loop_->monotonic_now();
  logging::ScopedLogRestorer prev_logger;
  if (simulated_event_loop_->log_impl_) {
    prev_logger.Swap(simulated_event_loop_->log_impl_);
  }

  {
    ScopedMarkRealtimeRestorer rt(simulated_event_loop_->priority() > 0);
    Call([monotonic_now]() { return monotonic_now; },
         [this](monotonic_clock::time_point sleep_time) {
           Schedule(sleep_time);
         });
  }
}

void SimulatedPhasedLoopHandler::Schedule(
    monotonic_clock::time_point sleep_time) {
  // The allocations in here are due to infrastructure and don't count in the no
  // mallocs in RT code.
  ScopedNotRealtime nrt;
  if (token_ != scheduler_->InvalidToken()) {
    scheduler_->Deschedule(token_);
    token_ = scheduler_->InvalidToken();
  }
  token_ = scheduler_->Schedule(sleep_time, [this]() {
    DCHECK(token_ != scheduler_->InvalidToken());
    token_ = scheduler_->InvalidToken();
    simulated_event_loop_->HandleEvent();
  });
  event_.set_event_time(sleep_time);
  simulated_event_loop_->AddEvent(&event_);
}

NodeEventLoopFactory::NodeEventLoopFactory(
    EventSchedulerScheduler *scheduler_scheduler,
    SimulatedEventLoopFactory *factory, const Node *node,
    std::vector<std::pair<EventLoop *, std::function<void(bool)>>>
        *raw_event_loops)
    : factory_(factory), node_(node), raw_event_loops_(raw_event_loops) {
  scheduler_scheduler->AddEventScheduler(&scheduler_);
}

SimulatedEventLoopFactory::SimulatedEventLoopFactory(
    const Configuration *configuration)
    : configuration_(CHECK_NOTNULL(configuration)),
      nodes_(configuration::GetNodes(configuration_)) {
  CHECK(IsInitialized()) << ": Need to initialize AOS first.";
  for (const Node *node : nodes_) {
    node_factories_.emplace_back(new NodeEventLoopFactory(
        &scheduler_scheduler_, this, node, &raw_event_loops_));
  }

  if (configuration::MultiNode(configuration)) {
    bridge_ = std::make_unique<message_bridge::SimulatedMessageBridge>(this);
  }
}

SimulatedEventLoopFactory::~SimulatedEventLoopFactory() {}

NodeEventLoopFactory *SimulatedEventLoopFactory::GetNodeEventLoopFactory(
    const Node *node) {
  auto result = std::find_if(
      node_factories_.begin(), node_factories_.end(),
      [node](const std::unique_ptr<NodeEventLoopFactory> &node_factory) {
        return node_factory->node() == node;
      });

  CHECK(result != node_factories_.end())
      << ": Failed to find node " << FlatbufferToJson(node);

  return result->get();
}

void SimulatedEventLoopFactory::SetTimeConverter(
    TimeConverter *time_converter) {
  for (std::unique_ptr<NodeEventLoopFactory> &factory : node_factories_) {
    factory->SetTimeConverter(time_converter);
  }
}

::std::unique_ptr<EventLoop> SimulatedEventLoopFactory::MakeEventLoop(
    std::string_view name, const Node *node) {
  if (node == nullptr) {
    CHECK(!configuration::MultiNode(configuration()))
        << ": Can't make a single node event loop in a multi-node world.";
  } else {
    CHECK(configuration::MultiNode(configuration()))
        << ": Can't make a multi-node event loop in a single-node world.";
  }
  return GetNodeEventLoopFactory(node)->MakeEventLoop(name);
}

::std::unique_ptr<EventLoop> NodeEventLoopFactory::MakeEventLoop(
    std::string_view name) {
  CHECK(!scheduler_.is_running())
      << ": Can't create an event loop while running";

  pid_t tid = tid_;
  ++tid_;
  ::std::unique_ptr<SimulatedEventLoop> result(new SimulatedEventLoop(
      &scheduler_, this, &channels_, factory_->configuration(),
      raw_event_loops_, node_, tid));
  result->set_name(name);
  result->set_send_delay(factory_->send_delay());
  return std::move(result);
}

void NodeEventLoopFactory::Disconnect(const Node *other) {
  factory_->bridge_->Disconnect(node_, other);
}
void NodeEventLoopFactory::Connect(const Node *other) {
  factory_->bridge_->Connect(node_, other);
}

void SimulatedEventLoopFactory::RunFor(monotonic_clock::duration duration) {
  for (const std::pair<EventLoop *, std::function<void(bool)>> &event_loop :
       raw_event_loops_) {
    event_loop.second(true);
  }
  scheduler_scheduler_.RunFor(duration);
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
  scheduler_scheduler_.Run();
  for (const std::pair<EventLoop *, std::function<void(bool)>> &event_loop :
       raw_event_loops_) {
    event_loop.second(false);
  }
}

void SimulatedEventLoopFactory::Exit() { scheduler_scheduler_.Exit(); }

void SimulatedEventLoopFactory::DisableForwarding(const Channel *channel) {
  CHECK(bridge_) << ": Can't disable forwarding without a message bridge.";
  bridge_->DisableForwarding(channel);
}

void SimulatedEventLoopFactory::DisableStatistics() {
  CHECK(bridge_) << ": Can't disable statistics without a message bridge.";
  bridge_->DisableStatistics();
}

void SimulatedEventLoopFactory::SkipTimingReport() {
  CHECK(bridge_) << ": Can't skip timing reports without a message bridge.";
  bridge_->SkipTimingReport();
}

}  // namespace aos
