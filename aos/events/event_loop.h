#ifndef AOS_EVENTS_EVENT_LOOP_H_
#define AOS_EVENTS_EVENT_LOOP_H_

#include <atomic>
#include <string>
#include <string_view>

#include "aos/configuration.h"
#include "aos/configuration_generated.h"
#include "aos/flatbuffers.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/time/time.h"
#include "flatbuffers/flatbuffers.h"
#include "glog/logging.h"

namespace aos {

// Struct available on Watchers and Fetchers with context about the current
// message.
struct Context {
  // Time that the message was sent.
  monotonic_clock::time_point monotonic_sent_time;
  realtime_clock::time_point realtime_sent_time;
  // Index in the queue.
  uint32_t queue_index;
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
  RawFetcher(const Channel *channel) : channel_(channel) {}
  virtual ~RawFetcher() {}

  // Non-blocking fetch of the next message in the queue. Returns true if there
  // was a new message and we got it.
  virtual bool FetchNext() = 0;

  // Non-blocking fetch of the latest message:
  virtual bool Fetch() = 0;

  // Returns a pointer to data in the most recent message, or nullptr if there
  // is no message.
  const void *most_recent_data() const { return data_; }

  const Context &context() const { return context_; }

  const Channel *channel() const { return channel_; }

 protected:
  RawFetcher(const RawFetcher &) = delete;
  RawFetcher &operator=(const RawFetcher &) = delete;

  void *data_ = nullptr;
  Context context_;
  const Channel *channel_;
};

// Raw version of sender.  Sends a block of data.  This is used for reflection
// and as a building block to implement typed senders.
class RawSender {
 public:
  RawSender(const Channel *channel) : channel_(channel) {}
  virtual ~RawSender() {}

  // Sends a message without copying it.  The users starts by copying up to
  // size() bytes into the data backed by data().  They then call Send to send.
  // Returns true on a successful send.
  virtual void *data() = 0;
  virtual size_t size() = 0;
  virtual bool Send(size_t size) = 0;

  // Sends a single block of data by copying it.
  virtual bool Send(const void *data, size_t size) = 0;

  // Returns the name of this sender.
  virtual const std::string_view name() const = 0;

  const Channel *channel() const { return channel_; }

 protected:
  RawSender(const RawSender &) = delete;
  RawSender &operator=(const RawSender &) = delete;

  const Channel *channel_;
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
    return fetcher_->most_recent_data() != nullptr
               ? flatbuffers::GetRoot<T>(reinterpret_cast<const char *>(
                     fetcher_->most_recent_data()))
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

  // Returns the name of the underlying queue.
  const std::string_view name() const { return sender_->name(); }

 private:
  friend class EventLoop;
  Sender(std::unique_ptr<RawSender> sender) : sender_(std::move(sender)) {}
  std::unique_ptr<RawSender> sender_;
};

// Interface for timers
class TimerHandler {
 public:
  virtual ~TimerHandler() {}

  // Timer should sleep until base, base + offset, base + offset * 2, ...
  // If repeat_offset isn't set, the timer only expires once.
  virtual void Setup(monotonic_clock::time_point base,
                     monotonic_clock::duration repeat_offset =
                         ::aos::monotonic_clock::zero()) = 0;

  // Stop future calls to callback().
  virtual void Disable() = 0;
};

// Interface for phased loops.  They are built on timers.
class PhasedLoopHandler {
 public:
  virtual ~PhasedLoopHandler() {}

  // Sets the interval and offset.  Any changes to interval and offset only take
  // effect when the handler finishes running.
  virtual void set_interval_and_offset(
      const monotonic_clock::duration interval,
      const monotonic_clock::duration offset) = 0;
};

// TODO(austin): Ping pong example apps, and then start doing introspection.
// TODO(austin): Timing reporter.  Publish statistics on latencies of
// handlers.
class EventLoop {
 public:
  EventLoop(const Configuration *configuration)
      : configuration_(configuration) {}

  virtual ~EventLoop() {}

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
    const Channel *channel = configuration::GetChannel(
        configuration_, channel_name, T::GetFullyQualifiedName(), name());
    CHECK(channel != nullptr)
        << ": Channel { \"name\": \"" << channel_name << "\", \"type\": \""
        << T::GetFullyQualifiedName() << "\" } not found in config.";

    return Fetcher<T>(MakeRawFetcher(channel));
  }

  // Makes class that allows constructing and sending messages to
  // the provided channel.
  template <typename T>
  Sender<T> MakeSender(const std::string_view channel_name) {
    const Channel *channel = configuration::GetChannel(
        configuration_, channel_name, T::GetFullyQualifiedName(), name());
    CHECK(channel != nullptr)
        << ": Channel { \"name\": \"" << channel_name << "\", \"type\": \""
        << T::GetFullyQualifiedName() << "\" } not found in config.";

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

  // Sets the name of the event loop.  This is the application name.
  virtual void set_name(const std::string_view name) = 0;
  // Gets the name of the event loop.
  virtual const std::string_view name() const = 0;

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

  // TODO(austin): OnExit

  // Threadsafe.
  bool is_running() const { return is_running_.load(); }

  // Sets the scheduler priority to run the event loop at.  This may not be
  // called after we go into "real-time-mode".
  virtual void SetRuntimeRealtimePriority(int priority) = 0;

  // Fetches new messages from the provided channel (path, type).  Note: this
  // channel must be a member of the exact configuration object this was built
  // with.
  virtual std::unique_ptr<RawFetcher> MakeRawFetcher(
      const Channel *channel) = 0;

  // Will watch channel (name, type) for new messages
  virtual void MakeRawWatcher(
      const Channel *channel,
      std::function<void(const Context &context, const void *message)>
          watcher) = 0;

  // Returns the context for the current message.
  // TODO(austin): Fill out whatever is useful for timers.
  const Context &context() const { return context_; }

  // Returns the configuration that this event loop was built with.
  const Configuration *configuration() const { return configuration_; }

  // Will send new messages from channel (path, type).
  virtual std::unique_ptr<RawSender> MakeRawSender(const Channel *channel) = 0;

 protected:
  void set_is_running(bool value) { is_running_.store(value); }

  // Validates that channel exists inside configuration_.
  void ValidateChannel(const Channel *channel);

 private:
  ::std::atomic<bool> is_running_{false};

  // Context available for watchers.
  Context context_;

  const Configuration *configuration_;
};

}  // namespace aos

#include "aos/events/event_loop_tmpl.h"

#endif  // AOS_EVENTS_EVENT_LOOP_H
