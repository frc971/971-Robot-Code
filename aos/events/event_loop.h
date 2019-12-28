#ifndef AOS_EVENTS_EVENT_LOOP_H_
#define AOS_EVENTS_EVENT_LOOP_H_

#include <atomic>
#include <string>
#include <string_view>

#include "aos/configuration.h"
#include "aos/configuration_generated.h"
#include "aos/events/event_loop_event.h"
#include "aos/events/event_loop_generated.h"
#include "aos/events/timing_statistics.h"
#include "aos/flatbuffers.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/time/time.h"
#include "aos/util/phased_loop.h"
#include "flatbuffers/flatbuffers.h"
#include "glog/logging.h"

DECLARE_bool(timing_reports);
DECLARE_int32(timing_report_ms);

namespace aos {

class EventLoop;
class WatcherState;

// Struct available on Watchers, Fetchers, Timers, and PhasedLoops with context
// about the current message.
struct Context {
  // Time that the message was sent on this node, or the timer was triggered.
  monotonic_clock::time_point monotonic_event_time;
  // Realtime the message was sent on this node.  This is set to min_time for
  // Timers and PhasedLoops.
  realtime_clock::time_point realtime_event_time;

  // For a single-node configuration, these two are identical to *_event_time.
  // In a multinode configuration, these are the times that the message was
  // sent on the original node.
  monotonic_clock::time_point monotonic_remote_time;
  realtime_clock::time_point realtime_remote_time;

  // The rest are only valid for Watchers and Fetchers.
  // Index in the queue.
  uint32_t queue_index;
  // Index into the remote queue.  Useful to determine if data was lost.  In a
  // single-node configuration, this will match queue_index.
  uint32_t remote_queue_index;

  // Size of the data sent.
  size_t size;
  // Pointer to the data.
  void *data;
};

// Raw version of fetcher. Contains a local variable that the fetcher will
// update.  This is used for reflection and as an interface to implement typed
// fetchers.
class RawFetcher {
 public:
  RawFetcher(EventLoop *event_loop, const Channel *channel);
  RawFetcher(const RawFetcher &) = delete;
  RawFetcher &operator=(const RawFetcher &) = delete;
  virtual ~RawFetcher();

  // Fetches the next message in the queue without blocking. Returns true if
  // there was a new message and we got it.
  bool FetchNext();

  // Fetches the latest message without blocking.
  bool Fetch();

  // Returns the channel this fetcher uses.
  const Channel *channel() const { return channel_; }
  // Returns the context for the current message.
  const Context &context() const { return context_; }

 protected:
  EventLoop *event_loop() { return event_loop_; }

  Context context_;

 private:
  friend class EventLoop;
  // Implementation
  virtual std::pair<bool, monotonic_clock::time_point> DoFetchNext() = 0;
  virtual std::pair<bool, monotonic_clock::time_point> DoFetch() = 0;

  EventLoop *event_loop_;
  const Channel *channel_;

  internal::RawFetcherTiming timing_;
};

// Raw version of sender.  Sends a block of data.  This is used for reflection
// and as a building block to implement typed senders.
class RawSender {
 public:
  RawSender(EventLoop *event_loop, const Channel *channel);
  RawSender(const RawSender &) = delete;
  RawSender &operator=(const RawSender &) = delete;

  virtual ~RawSender();

  // Sends a message without copying it.  The users starts by copying up to
  // size() bytes into the data backed by data().  They then call Send to send.
  // Returns true on a successful send.
  // If provided, monotonic_remote_time, realtime_remote_time, and
  // remote_queue_index are attached to the message and are available in the
  // context on the read side.  If they are not populated, the read side will
  // get the sent times instead.
  virtual void *data() = 0;
  virtual size_t size() = 0;
  bool Send(size_t size,
            aos::monotonic_clock::time_point monotonic_remote_time =
                aos::monotonic_clock::min_time,
            aos::realtime_clock::time_point realtime_remote_time =
                aos::realtime_clock::min_time,
            uint32_t remote_queue_index = 0xffffffffu);

  // Sends a single block of data by copying it.
  // The remote arguments have the same meaning as in Send above.
  bool Send(const void *data, size_t size,
            aos::monotonic_clock::time_point monotonic_remote_time =
                aos::monotonic_clock::min_time,
            aos::realtime_clock::time_point realtime_remote_time =
                aos::realtime_clock::min_time,
            uint32_t remote_queue_index = 0xffffffffu);

  const Channel *channel() const { return channel_; }

  // Returns the time_points that the last message was sent at.
  aos::monotonic_clock::time_point monotonic_sent_time() const {
    return monotonic_sent_time_;
  }
  aos::realtime_clock::time_point realtime_sent_time() const {
    return realtime_sent_time_;
  }
  // Returns the queue index that this was sent with.
  uint32_t sent_queue_index() const { return sent_queue_index_; }

 protected:
  EventLoop *event_loop() { return event_loop_; }

  aos::monotonic_clock::time_point monotonic_sent_time_ =
      aos::monotonic_clock::min_time;
  aos::realtime_clock::time_point realtime_sent_time_ =
      aos::realtime_clock::min_time;
  uint32_t sent_queue_index_ = 0xffffffff;

 private:
  friend class EventLoop;

  virtual bool DoSend(const void *data, size_t size,
                      aos::monotonic_clock::time_point monotonic_remote_time,
                      aos::realtime_clock::time_point realtime_remote_time,
                      uint32_t remote_queue_index) = 0;
  virtual bool DoSend(size_t size,
                      aos::monotonic_clock::time_point monotonic_remote_time,
                      aos::realtime_clock::time_point realtime_remote_time,
                      uint32_t remote_queue_index) = 0;

  EventLoop *event_loop_;
  const Channel *channel_;

  internal::RawSenderTiming timing_;
};

// Fetches the newest message from a channel.
// This provides a polling based interface for channels.
template <typename T>
class Fetcher {
 public:
  Fetcher() {}

  // Fetches the next message. Returns true if it fetched a new message.  This
  // method will only return messages sent after the Fetcher was created.
  bool FetchNext() { return fetcher_->FetchNext(); }

  // Fetches the most recent message. Returns true if it fetched a new message.
  // This will return the latest message regardless of if it was sent before or
  // after the fetcher was created.
  bool Fetch() { return fetcher_->Fetch(); }

  // Returns a pointer to the contained flatbuffer, or nullptr if there is no
  // available message.
  const T *get() const {
    return fetcher_->context().data != nullptr
               ? flatbuffers::GetRoot<T>(
                     reinterpret_cast<const char *>(fetcher_->context().data))
               : nullptr;
  }

  // Returns the context holding timestamps and other metadata about the
  // message.
  const Context &context() const { return fetcher_->context(); }

  const T &operator*() const { return *get(); }
  const T *operator->() const { return get(); }

 private:
  friend class EventLoop;
  Fetcher(::std::unique_ptr<RawFetcher> fetcher)
      : fetcher_(::std::move(fetcher)) {}
  ::std::unique_ptr<RawFetcher> fetcher_;
};

// Sends messages to a channel.
template <typename T>
class Sender {
 public:
  Sender() {}

  // Represents a single message about to be sent to the queue.
  // The lifecycle goes:
  //
  // Builder builder = sender.MakeBuilder();
  // T::Builder t_builder = builder.MakeBuilder<T>();
  // Populate(&t_builder);
  // builder.Send(t_builder.Finish());
  class Builder {
   public:
    Builder(RawSender *sender, void *data, size_t size)
        : alloc_(data, size), fbb_(size, &alloc_), sender_(sender) {
      fbb_.ForceDefaults(1);
    }

    flatbuffers::FlatBufferBuilder *fbb() { return &fbb_; }

    template <typename T2>
    typename T2::Builder MakeBuilder() {
      return typename T2::Builder(fbb_);
    }

    bool Send(flatbuffers::Offset<T> offset) {
      fbb_.Finish(offset);
      return sender_->Send(fbb_.GetSize());
    }

    // CHECKs that this message was sent.
    void CheckSent() { fbb_.Finished(); }

   private:
    PreallocatedAllocator alloc_;
    flatbuffers::FlatBufferBuilder fbb_;
    RawSender *sender_;
  };

  // Constructs an above builder.
  Builder MakeBuilder();

  // Sends a prebuilt flatbuffer.
  bool Send(const Flatbuffer<T> &flatbuffer);

  // Returns the name of the underlying queue.
  const Channel *channel() const { return sender_->channel(); }

 private:
  friend class EventLoop;
  Sender(std::unique_ptr<RawSender> sender) : sender_(std::move(sender)) {}
  std::unique_ptr<RawSender> sender_;
};

// Interface for timers
class TimerHandler {
 public:
  virtual ~TimerHandler();

  // Timer should sleep until base, base + offset, base + offset * 2, ...
  // If repeat_offset isn't set, the timer only expires once.
  virtual void Setup(monotonic_clock::time_point base,
                     monotonic_clock::duration repeat_offset =
                         ::aos::monotonic_clock::zero()) = 0;

  // Stop future calls to callback().
  virtual void Disable() = 0;

  // Sets and gets the name of the timer.  Set this if you want a descriptive
  // name in the timing report.
  void set_name(std::string_view name) { name_ = std::string(name); }
  const std::string_view name() const { return name_; }

 protected:
  TimerHandler(EventLoop *event_loop, std::function<void()> fn);

  void Call(std::function<monotonic_clock::time_point()> get_time,
            monotonic_clock::time_point event_time);

 private:
  friend class EventLoop;

  EventLoop *event_loop_;
  // The function to call when Call is called.
  std::function<void()> fn_;
  std::string name_;

  internal::TimerTiming timing_;
};

// Interface for phased loops.  They are built on timers.
class PhasedLoopHandler {
 public:
  virtual ~PhasedLoopHandler();

  // Sets the interval and offset.  Any changes to interval and offset only take
  // effect when the handler finishes running.
  void set_interval_and_offset(const monotonic_clock::duration interval,
                               const monotonic_clock::duration offset) {
    phased_loop_.set_interval_and_offset(interval, offset);
  }

  // Sets and gets the name of the timer.  Set this if you want a descriptive
  // name in the timing report.
  void set_name(std::string_view name) { name_ = std::string(name); }
  const std::string_view name() const { return name_; }

 protected:
  void Call(std::function<monotonic_clock::time_point()> get_time,
            std::function<void(monotonic_clock::time_point)> schedule);

  PhasedLoopHandler(EventLoop *event_loop, std::function<void(int)> fn,
                    const monotonic_clock::duration interval,
                    const monotonic_clock::duration offset);

 private:
  friend class EventLoop;

  void Reschedule(std::function<void(monotonic_clock::time_point)> schedule,
                  monotonic_clock::time_point monotonic_now) {
    cycles_elapsed_ += phased_loop_.Iterate(monotonic_now);
    schedule(phased_loop_.sleep_time());
  }

  virtual void Schedule(monotonic_clock::time_point sleep_time) = 0;

  EventLoop *event_loop_;
  std::function<void(int)> fn_;
  std::string name_;
  time::PhasedLoop phased_loop_;

  int cycles_elapsed_ = 0;

  internal::TimerTiming timing_;
};

class EventLoop {
 public:
  EventLoop(const Configuration *configuration)
      : timing_report_(flatbuffers::DetachedBuffer()),
        configuration_(configuration) {}

  virtual ~EventLoop();

  // Current time.
  virtual monotonic_clock::time_point monotonic_now() = 0;
  virtual realtime_clock::time_point realtime_now() = 0;

  // Note, it is supported to create:
  //   multiple fetchers, and (one sender or one watcher) per <name, type>
  //   tuple.

  // Makes a class that will always fetch the most recent value
  // sent to the provided channel.
  template <typename T>
  Fetcher<T> MakeFetcher(const std::string_view channel_name) {
    const Channel *channel =
        configuration::GetChannel(configuration_, channel_name,
                                  T::GetFullyQualifiedName(), name(), node());
    CHECK(channel != nullptr)
        << ": Channel { \"name\": \"" << channel_name << "\", \"type\": \""
        << T::GetFullyQualifiedName() << "\" } not found in config.";

    if (!configuration::ChannelIsReadableOnNode(channel, node())) {
      LOG(FATAL) << "Channel { \"name\": \"" << channel_name
                 << "\", \"type\": \"" << T::GetFullyQualifiedName()
                 << "\" } is not able to be fetched on this node.  Check your "
                    "configuration.";
    }

    return Fetcher<T>(MakeRawFetcher(channel));
  }

  // Makes class that allows constructing and sending messages to
  // the provided channel.
  template <typename T>
  Sender<T> MakeSender(const std::string_view channel_name) {
    const Channel *channel =
        configuration::GetChannel(configuration_, channel_name,
                                  T::GetFullyQualifiedName(), name(), node());
    CHECK(channel != nullptr)
        << ": Channel { \"name\": \"" << channel_name << "\", \"type\": \""
        << T::GetFullyQualifiedName() << "\" } not found in config.";

    if (!configuration::ChannelIsSendableOnNode(channel, node())) {
      LOG(FATAL) << "Channel { \"name\": \"" << channel_name
                 << "\", \"type\": \"" << T::GetFullyQualifiedName()
                 << "\" } is not able to be sent on this node.  Check your "
                    "configuration.";
    }

    return Sender<T>(MakeRawSender(channel));
  }

  // This will watch messages sent to the provided channel.
  //
  // Watch is a functor that have a call signature like so:
  // void Event(const MessageType& type);
  //
  // TODO(parker): Need to support ::std::bind.  For now, use lambdas.
  // TODO(austin): Do we need a functor?  Or is a std::function good enough?
  template <typename Watch>
  void MakeWatcher(const std::string_view name, Watch &&w);

  // The passed in function will be called when the event loop starts.
  // Use this to run code once the thread goes into "real-time-mode",
  virtual void OnRun(::std::function<void()> on_run) = 0;

  // Gets the name of the event loop.  This is the application name.
  virtual const std::string_view name() const = 0;

  // Returns the node that this event loop is running on.  Returns nullptr if we
  // are running in single-node mode.
  virtual const Node *node() const = 0;

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

  // TODO(austin): OnExit for cleanup.

  // Threadsafe.
  bool is_running() const { return is_running_.load(); }

  // Sets the scheduler priority to run the event loop at.  This may not be
  // called after we go into "real-time-mode".
  virtual void SetRuntimeRealtimePriority(int priority) = 0;
  virtual int priority() const = 0;

  // Fetches new messages from the provided channel (path, type).
  //
  // Note: this channel must be a member of the exact configuration object this
  // was built with.
  virtual std::unique_ptr<RawFetcher> MakeRawFetcher(
      const Channel *channel) = 0;

  // Watches channel (name, type) for new messages.
  virtual void MakeRawWatcher(
      const Channel *channel,
      std::function<void(const Context &context, const void *message)>
          watcher) = 0;

  // Creates a raw sender for the provided channel.  This is used for reflection
  // based sending.
  // Note: this ignores any node constraints.  Ignore at your own peril.
  virtual std::unique_ptr<RawSender> MakeRawSender(const Channel *channel) = 0;

  // Returns the context for the current callback.
  const Context &context() const { return context_; }

  // Returns the configuration that this event loop was built with.
  const Configuration *configuration() const { return configuration_; }

  // Prevents the event loop from sending a timing report.
  void SkipTimingReport() { skip_timing_report_ = true; }

 protected:
  // Sets the name of the event loop.  This is the application name.
  virtual void set_name(const std::string_view name) = 0;

  void set_is_running(bool value) { is_running_.store(value); }

  // Validates that channel exists inside configuration_ and finds its index.
  int ChannelIndex(const Channel *channel);

  // Context available for watchers, timers, and phased loops.
  Context context_;

  friend class RawSender;
  friend class TimerHandler;
  friend class RawFetcher;
  friend class PhasedLoopHandler;
  friend class WatcherState;

  // Methods used to implement timing reports.
  void NewSender(RawSender *sender);
  void DeleteSender(RawSender *sender);
  TimerHandler *NewTimer(std::unique_ptr<TimerHandler> timer);
  PhasedLoopHandler *NewPhasedLoop(
      std::unique_ptr<PhasedLoopHandler> phased_loop);
  void NewFetcher(RawFetcher *fetcher);
  void DeleteFetcher(RawFetcher *fetcher);
  WatcherState *NewWatcher(std::unique_ptr<WatcherState> watcher);

  std::vector<RawSender *> senders_;
  std::vector<RawFetcher *> fetchers_;

  std::vector<std::unique_ptr<TimerHandler>> timers_;
  std::vector<std::unique_ptr<PhasedLoopHandler>> phased_loops_;
  std::vector<std::unique_ptr<WatcherState>> watchers_;

  void SendTimingReport();
  void UpdateTimingReport();
  void MaybeScheduleTimingReports();

  std::unique_ptr<RawSender> timing_report_sender_;

  // Tracks which event sources (timers and watchers) have data, and which
  // don't.  Added events may not change their event_time().
  // TODO(austin): Test case 1: timer triggers at t1, handler takes until after
  // t2 to run, t2 should then be picked up without a context switch.
  void AddEvent(EventLoopEvent *event);
  void RemoveEvent(EventLoopEvent *event);
  size_t EventCount() const { return events_.size(); }
  EventLoopEvent *PopEvent();
  EventLoopEvent *PeekEvent() { return events_.front(); }
  void ReserveEvents();

  std::vector<EventLoopEvent *> events_;

 private:
  virtual pid_t GetTid() = 0;

  FlatbufferDetachedBuffer<timing::Report> timing_report_;

  ::std::atomic<bool> is_running_{false};

  const Configuration *configuration_;

  // If true, don't send out timing reports.
  bool skip_timing_report_ = false;
};

}  // namespace aos

#include "aos/events/event_loop_tmpl.h"

#endif  // AOS_EVENTS_EVENT_LOOP_H
