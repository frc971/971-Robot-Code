#ifndef AOS_EVENTS_EVENT_LOOP_H_
#define AOS_EVENTS_EVENT_LOOP_H_

#include <sched.h>

#include <atomic>
#include <ostream>
#include <string>
#include <string_view>

#include "absl/container/btree_set.h"
#include "aos/configuration.h"
#include "aos/configuration_generated.h"
#include "aos/events/channel_preallocated_allocator.h"
#include "aos/events/event_loop_event.h"
#include "aos/events/event_loop_generated.h"
#include "aos/events/timing_statistics.h"
#include "aos/flatbuffers.h"
#include "aos/ftrace.h"
#include "aos/ipc_lib/data_alignment.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/time/time.h"
#include "aos/util/phased_loop.h"
#include "aos/uuid.h"
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

  // The rest are only valid for Watchers and Fetchers.

  // For a single-node configuration, these two are identical to *_event_time.
  // In a multinode configuration, these are the times that the message was
  // sent on the original node.
  monotonic_clock::time_point monotonic_remote_time;
  realtime_clock::time_point realtime_remote_time;

  // Index in the queue.
  uint32_t queue_index;
  // Index into the remote queue.  Useful to determine if data was lost.  In a
  // single-node configuration, this will match queue_index.
  uint32_t remote_queue_index;

  // Size of the data sent.
  size_t size;
  // Pointer to the data.
  const void *data;

  // Index of the message buffer. This will be in [0, NumberBuffers) on
  // read_method=PIN channels, and -1 for other channels.
  //
  // This only tells you about the underlying storage for this message, not
  // anything about its position in the queue. This is only useful for advanced
  // zero-copy use cases, on read_method=PIN channels.
  //
  // This will uniquely identify a message on this channel at a point in time.
  // For senders, this point in time is while the sender has the message. With
  // read_method==PIN, this point in time includes while the caller has access
  // to this context. For other read_methods, this point in time may be before
  // the caller has access to this context, which makes this pretty useless.
  int buffer_index;

  // UUID of the remote node which sent this message, or this node in the case
  // of events which are local to this node.
  UUID source_boot_uuid = UUID::Zero();

  // Efficiently copies the flatbuffer into a FlatbufferVector, allocating
  // memory in the process.  It is vital that T matches the type of the
  // underlying flatbuffer.
  template <typename T>
  FlatbufferVector<T> CopyFlatBuffer() const {
    ResizeableBuffer buffer;
    buffer.resize(size);
    memcpy(buffer.data(), data, size);
    return FlatbufferVector<T>(std::move(buffer));
  }
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
  const EventLoop *event_loop() const { return event_loop_; }

  Context context_;

 private:
  friend class EventLoop;
  // Implementation
  virtual std::pair<bool, monotonic_clock::time_point> DoFetchNext() = 0;
  virtual std::pair<bool, monotonic_clock::time_point> DoFetch() = 0;

  EventLoop *const event_loop_;
  const Channel *const channel_;
  const std::string ftrace_prefix_;

  internal::RawFetcherTiming timing_;
  Ftrace ftrace_;
};

using SharedSpan = std::shared_ptr<const absl::Span<const uint8_t>>;

// Holds storage for a span object and the data referenced by that span for
// compatibility with SharedSpan users. If constructed with MakeSharedSpan, span
// points to only the aligned segment of the entire data.
struct AlignedOwningSpan {
  AlignedOwningSpan(absl::Span<const uint8_t> new_span) : span(new_span) {}

  AlignedOwningSpan(const AlignedOwningSpan &) = delete;
  AlignedOwningSpan &operator=(const AlignedOwningSpan &) = delete;
  absl::Span<const uint8_t> span;
  char *data() { return reinterpret_cast<char *>(this + 1); }
};

// Constructs a span which owns its data through a shared_ptr. The owning span
// points to a const view of the data; also returns a temporary mutable span
// which is only valid while the const shared span is kept alive.
std::pair<SharedSpan, absl::Span<uint8_t>> MakeSharedSpan(size_t size);

// Raw version of sender.  Sends a block of data.  This is used for reflection
// and as a building block to implement typed senders.
class RawSender {
 public:
  using SharedSpan = std::shared_ptr<const absl::Span<const uint8_t>>;

  // This looks a little ugly with no space, but please leave it so clang-format
  // doesn't keep changing it. Bug is filed at
  // <https://github.com/llvm/llvm-project/issues/55457>.
  enum class [[nodiscard]] Error{
      // Represents success and no error
      kOk,

      // Error for messages on channels being sent faster than their
      // frequency and channel storage duration allow
      kMessagesSentTooFast,
      // Access to Redzone was attempted in Sender Queue
      kInvalidRedzone,
  };

  RawSender(EventLoop *event_loop, const Channel *channel);
  RawSender(const RawSender &) = delete;
  RawSender &operator=(const RawSender &) = delete;

  virtual ~RawSender();

  // Returns the buffer to write new messages into. This is always valid, and
  // may change after calling any of the Send functions.
  virtual void *data() = 0;
  virtual size_t size() = 0;

  // Sends a message without copying it.  The users starts by copying up to
  // size() bytes into the data backed by data().  They then call Send to send.
  // Returns Error::kOk on a successful send, or
  // Error::kMessagesSentTooFast if messages were sent too fast. If provided,
  // monotonic_remote_time, realtime_remote_time, and remote_queue_index are
  // attached to the message and are available in the context on the read side.
  // If they are not populated, the read side will get the sent times instead.
  Error Send(size_t size);
  Error Send(size_t size, monotonic_clock::time_point monotonic_remote_time,
             realtime_clock::time_point realtime_remote_time,
             uint32_t remote_queue_index, const UUID &source_boot_uuid);

  // Sends a single block of data by copying it.
  // The remote arguments have the same meaning as in Send above.
  // Returns Error::kMessagesSentTooFast if messages were sent too fast
  Error Send(const void *data, size_t size);
  Error Send(const void *data, size_t size,
             monotonic_clock::time_point monotonic_remote_time,
             realtime_clock::time_point realtime_remote_time,
             uint32_t remote_queue_index, const UUID &source_boot_uuid);

  // CHECKs that no sending Error occurred and logs the channel_ data if
  // one did
  void CheckOk(const Error err);

  // Sends a single block of data by refcounting it to avoid copies.  The data
  // must not change after being passed into Send. The remote arguments have the
  // same meaning as in Send above.
  Error Send(const SharedSpan data);
  Error Send(const SharedSpan data,
             monotonic_clock::time_point monotonic_remote_time,
             realtime_clock::time_point realtime_remote_time,
             uint32_t remote_queue_index, const UUID &remote_boot_uuid);

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

  // Returns the associated flatbuffers-style allocator. This must be
  // deallocated before the message is sent.
  ChannelPreallocatedAllocator *fbb_allocator() {
    fbb_allocator_ = ChannelPreallocatedAllocator(
        reinterpret_cast<uint8_t *>(data()), size(), channel());
    return &fbb_allocator_;
  }

  // Index of the buffer which is currently exposed by data() and the various
  // other accessors. This is the message the caller should be filling out.
  virtual int buffer_index() = 0;

 protected:
  EventLoop *event_loop() { return event_loop_; }
  const EventLoop *event_loop() const { return event_loop_; }

  monotonic_clock::time_point monotonic_sent_time_ = monotonic_clock::min_time;
  realtime_clock::time_point realtime_sent_time_ = realtime_clock::min_time;
  uint32_t sent_queue_index_ = 0xffffffff;

 private:
  friend class EventLoop;

  virtual Error DoSend(const void *data, size_t size,
                       monotonic_clock::time_point monotonic_remote_time,
                       realtime_clock::time_point realtime_remote_time,
                       uint32_t remote_queue_index,
                       const UUID &source_boot_uuid) = 0;
  virtual Error DoSend(size_t size,
                       monotonic_clock::time_point monotonic_remote_time,
                       realtime_clock::time_point realtime_remote_time,
                       uint32_t remote_queue_index,
                       const UUID &source_boot_uuid) = 0;
  virtual Error DoSend(const SharedSpan data,
                       monotonic_clock::time_point monotonic_remote_time,
                       realtime_clock::time_point realtime_remote_time,
                       uint32_t remote_queue_index,
                       const UUID &source_boot_uuid);

  void RecordSendResult(const Error error, size_t message_size);

  EventLoop *const event_loop_;
  const Channel *const channel_;
  const std::string ftrace_prefix_;

  internal::RawSenderTiming timing_;
  Ftrace ftrace_;

  ChannelPreallocatedAllocator fbb_allocator_{nullptr, 0, nullptr};
};

// Needed for compatibility with glog
std::ostream &operator<<(std::ostream &os, const RawSender::Error err);

// Fetches the newest message from a channel.
// This provides a polling based interface for channels.
template <typename T>
class Fetcher {
 public:
  Fetcher() {}

  // Fetches the next message. Returns true if it fetched a new message.  This
  // method will only return messages sent after the Fetcher was created.
  bool FetchNext() {
    const bool result = CHECK_NOTNULL(fetcher_)->FetchNext();
    if (result) {
      CheckChannelDataAlignment(fetcher_->context().data,
                                fetcher_->context().size);
    }
    return result;
  }

  // Fetches the most recent message. Returns true if it fetched a new message.
  // This will return the latest message regardless of if it was sent before or
  // after the fetcher was created.
  bool Fetch() {
    const bool result = CHECK_NOTNULL(fetcher_)->Fetch();
    if (result) {
      CheckChannelDataAlignment(fetcher_->context().data,
                                fetcher_->context().size);
    }
    return result;
  }

  // Returns a pointer to the contained flatbuffer, or nullptr if there is no
  // available message.
  const T *get() const {
    return CHECK_NOTNULL(fetcher_)->context().data != nullptr
               ? flatbuffers::GetRoot<T>(
                     reinterpret_cast<const char *>(fetcher_->context().data))
               : nullptr;
  }

  // Returns the channel this fetcher uses
  const Channel *channel() const { return CHECK_NOTNULL(fetcher_)->channel(); }

  // Returns the context holding timestamps and other metadata about the
  // message.
  const Context &context() const { return CHECK_NOTNULL(fetcher_)->context(); }

  const T &operator*() const { return *get(); }
  const T *operator->() const { return get(); }

  // Returns true if this fetcher is valid and connected to a channel. If you,
  // e.g., are using TryMakeFetcher, then you must check valid() before
  // attempting to use the Fetcher.
  bool valid() const { return static_cast<bool>(fetcher_); }

  // Copies the current flatbuffer into a FlatbufferVector.
  FlatbufferVector<T> CopyFlatBuffer() const {
    return context().template CopyFlatBuffer<T>();
  }

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
    Builder(RawSender *sender, ChannelPreallocatedAllocator *allocator)
        : fbb_(allocator->size(), allocator),
          allocator_(allocator),
          sender_(CHECK_NOTNULL(sender)) {
      CheckChannelDataAlignment(allocator->data(), allocator->size());
      fbb_.ForceDefaults(true);
    }
    Builder() {}
    Builder(const Builder &) = delete;
    Builder(Builder &&) = default;

    Builder &operator=(const Builder &) = delete;
    Builder &operator=(Builder &&) = default;

    flatbuffers::FlatBufferBuilder *fbb() { return &fbb_; }

    template <typename T2>
    typename T2::Builder MakeBuilder() {
      return typename T2::Builder(fbb_);
    }

    RawSender::Error Send(flatbuffers::Offset<T> offset) {
      fbb_.Finish(offset);
      const auto err = sender_->Send(fbb_.GetSize());
      // Ensure fbb_ knows it shouldn't access the memory any more.
      fbb_ = flatbuffers::FlatBufferBuilder();
      return err;
    }

    // Equivalent to RawSender::CheckOk
    void CheckOk(const RawSender::Error err) { sender_->CheckOk(err); };

    // CHECKs that this message was sent.
    void CheckSent() {
      CHECK(!allocator_->is_allocated()) << ": Message was not sent yet";
    }

    // Detaches a buffer, for later use calling Sender::Send directly.
    //
    // Note that the underlying memory remains with the Sender, so creating
    // another Builder before destroying the FlatbufferDetachedBuffer will fail.
    FlatbufferDetachedBuffer<T> Detach(flatbuffers::Offset<T> offset) {
      fbb_.Finish(offset);
      return fbb_.Release();
    }

   private:
    flatbuffers::FlatBufferBuilder fbb_;
    ChannelPreallocatedAllocator *allocator_;
    RawSender *sender_;
  };

  // Constructs an above builder.
  //
  // Only a single one of these may be "alive" for this object at any point in
  // time. After calling Send on the result, it is no longer "alive". This means
  // that you must manually reset a variable holding the return value (by
  // assigning a default-constructed Builder to it) before calling this method
  // again to overwrite the value in the variable.
  Builder MakeBuilder();

  // Sends a prebuilt flatbuffer.
  // This will copy the data out of the provided flatbuffer, and so does not
  // have to correspond to an existing Builder.
  RawSender::Error Send(const NonSizePrefixedFlatbuffer<T> &flatbuffer);

  // Sends a prebuilt flatbuffer which was detached from a Builder created via
  // MakeBuilder() on this object.
  RawSender::Error SendDetached(FlatbufferDetachedBuffer<T> detached);

  // Equivalent to RawSender::CheckOk
  void CheckOk(const RawSender::Error err) {
    CHECK_NOTNULL(sender_)->CheckOk(err);
  };

  // Returns the name of the underlying queue, if valid.  You must check valid()
  // first.
  const Channel *channel() const { return CHECK_NOTNULL(sender_)->channel(); }

  // Returns true if the Sender is a valid Sender. If you, e.g., are using
  // TryMakeSender, then you must check valid() before attempting to use the
  // Sender.
  // TODO(austin): Deprecate the operator bool.
  operator bool() const { return sender_ ? true : false; }
  bool valid() const { return static_cast<bool>(sender_); }

  // Returns the time_points that the last message was sent at.
  aos::monotonic_clock::time_point monotonic_sent_time() const {
    return CHECK_NOTNULL(sender_)->monotonic_sent_time();
  }
  aos::realtime_clock::time_point realtime_sent_time() const {
    return CHECK_NOTNULL(sender_)->realtime_sent_time();
  }
  // Returns the queue index that this was sent with.
  uint32_t sent_queue_index() const {
    return CHECK_NOTNULL(sender_)->sent_queue_index();
  }

  // Returns the buffer index which MakeBuilder() will expose access to. This is
  // the buffer the caller can fill out.
  int buffer_index() const { return CHECK_NOTNULL(sender_)->buffer_index(); }

 private:
  friend class EventLoop;
  Sender(std::unique_ptr<RawSender> sender) : sender_(std::move(sender)) {}
  std::unique_ptr<RawSender> sender_;
};

// Class for keeping a count of send failures on a certain channel
class SendFailureCounter {
 public:
  inline void Count(const RawSender::Error err) {
    failures_ += static_cast<size_t>(err != RawSender::Error::kOk);
    just_failed_ = (err != RawSender::Error::kOk);
  }

  inline size_t failures() const { return failures_; }
  inline bool just_failed() const { return just_failed_; }

 private:
  size_t failures_ = 0;
  bool just_failed_ = false;
};

// Interface for timers.
class TimerHandler {
 public:
  virtual ~TimerHandler();

  // Timer should sleep until base, base + offset, base + offset * 2, ...
  // If repeat_offset isn't set, the timer only expires once.
  // base must be greater than or equal to zero.
  virtual void Setup(monotonic_clock::time_point base,
                     monotonic_clock::duration repeat_offset =
                         ::aos::monotonic_clock::zero()) = 0;

  // Stop future calls to callback().
  virtual void Disable() = 0;

  // Check if the timer is disabled
  virtual bool IsDisabled() = 0;

  // Sets and gets the name of the timer.  Set this if you want a descriptive
  // name in the timing report.
  void set_name(std::string_view name) { name_ = std::string(name); }
  const std::string_view name() const { return name_; }

 protected:
  TimerHandler(EventLoop *event_loop, std::function<void()> fn);

  template <typename T>
  monotonic_clock::time_point Call(T get_time,
                                   monotonic_clock::time_point event_time);

 private:
  friend class EventLoop;

  EventLoop *event_loop_;
  // The function to call when Call is called.
  std::function<void()> fn_;
  std::string name_;

  internal::TimerTiming timing_;
  Ftrace ftrace_;
};

// Interface for phased loops. They are built on timers.
class PhasedLoopHandler {
 public:
  virtual ~PhasedLoopHandler();

  // Sets the interval and offset. Any changes to interval and offset only take
  // effect when the handler finishes running or upon a call to Reschedule().
  //
  // The precise semantics of the monotonic_now parameter are defined in
  // phased_loop.h. The way that it gets used in this interface is to control
  // what the cycles elapsed counter will read on the following call to your
  // handler. In an idealized simulation environment, if you call
  // set_interval_and_offset() during the phased loop offset without setting
  // monotonic_now, then you should always see a count of 1 on the next cycle.
  //
  // With the default behavior (this is called inside your handler and with
  // monotonic_now = nullopt), the next phased loop call will occur at most
  // interval time after the current time. Note that in many cases this will
  // *not* be the preferred behavior (in most cases, you would likely aim to
  // keep the average frequency of the calls reasonably consistent).
  //
  // A combination of the monotonic_now parameter and manually calling
  // Reschedule() outside of the phased loop handler can be used to alter the
  // behavior of cycles_elapsed. For the default behavior, you can set
  // monotonic_now to the current time. If you call set_interval_and_offset()
  // and Reschedule() with the same monotonic_now, that will cause the next
  // callback to occur in the range  (monotonic_now, monotonic_now + interval]
  // and get called with a count of 1 (if the event is actually able to happen
  // when it is scheduled to). E.g., if you are just adjusting the phased loop
  // offset (but not the interval) and want to maintain a consistent frequency,
  // you may call these functions with monotonic_now = now + interval / 2.
  void set_interval_and_offset(
      const monotonic_clock::duration interval,
      const monotonic_clock::duration offset,
      std::optional<monotonic_clock::time_point> monotonic_now = std::nullopt) {
    phased_loop_.set_interval_and_offset(interval, offset, monotonic_now);
  }

  // Reruns the scheduler for the phased loop, scheduling the next callback at
  // the next available time after monotonic_now. This allows
  // set_interval_and_offset() to be called and have the changes take effect
  // before the next handler finishes. This will have no effect if run during
  // the phased loop's own handler.
  void Reschedule(monotonic_clock::time_point monotonic_now) {
    cycles_elapsed_ += phased_loop_.Iterate(monotonic_now);
    Schedule(phased_loop_.sleep_time());
  }

  // Sets and gets the name of the timer. Set this if you want a descriptive
  // name in the timing report.
  void set_name(std::string_view name) { name_ = std::string(name); }
  const std::string_view name() const { return name_; }

 protected:
  void Call(std::function<monotonic_clock::time_point()> get_time);

  PhasedLoopHandler(EventLoop *event_loop, std::function<void(int)> fn,
                    const monotonic_clock::duration interval,
                    const monotonic_clock::duration offset);

 private:
  friend class EventLoop;

  virtual void Schedule(monotonic_clock::time_point sleep_time) = 0;

  EventLoop *event_loop_;
  std::function<void(int)> fn_;
  std::string name_;
  time::PhasedLoop phased_loop_;

  int cycles_elapsed_ = 0;

  internal::TimerTiming timing_;
  Ftrace ftrace_;
};

// Note, it is supported to create only:
//   multiple fetchers, and (one sender or one watcher) per <name, type>
//   tuple.
class EventLoop {
 public:
  // Holds configuration by reference for the lifetime of this object. It may
  // never be mutated externally in any way.
  EventLoop(const Configuration *configuration);

  virtual ~EventLoop();

  // Current time.
  virtual monotonic_clock::time_point monotonic_now() const = 0;
  virtual realtime_clock::time_point realtime_now() const = 0;

  template <typename T>
  const Channel *GetChannel(const std::string_view channel_name) {
    return configuration::GetChannel(configuration(), channel_name,
                                     T::GetFullyQualifiedName(), name(), node(),
                                     true);
  }
  // Returns true if the channel exists in the configuration.
  template <typename T>
  bool HasChannel(const std::string_view channel_name) {
    return GetChannel<T>(channel_name) != nullptr;
  }

  // Like MakeFetcher, but returns an invalid fetcher if the given channel is
  // not readable on this node or does not exist. You must check valid() on the
  // Fetcher before using it.
  template <typename T>
  Fetcher<T> TryMakeFetcher(const std::string_view channel_name) {
    const Channel *const channel = GetChannel<T>(channel_name);
    if (channel == nullptr) {
      return Fetcher<T>();
    }

    if (!configuration::ChannelIsReadableOnNode(channel, node())) {
      return Fetcher<T>();
    }

    return Fetcher<T>(MakeRawFetcher(channel));
  }

  // Makes a class that will always fetch the most recent value
  // sent to the provided channel.
  template <typename T>
  Fetcher<T> MakeFetcher(const std::string_view channel_name) {
    CHECK(HasChannel<T>(channel_name))
        << ": Channel { \"name\": \"" << channel_name << "\", \"type\": \""
        << T::GetFullyQualifiedName() << "\" } not found in config.";

    Fetcher<T> result = TryMakeFetcher<T>(channel_name);
    if (!result.valid()) {
      LOG(FATAL) << "Channel { \"name\": \"" << channel_name
                 << "\", \"type\": \"" << T::GetFullyQualifiedName()
                 << "\" } is not able to be fetched on this node.  Check your "
                    "configuration.";
    }

    return result;
  }

  // Like MakeSender, but returns an invalid sender if the given channel is
  // not sendable on this node or does not exist. You must check valid() on the
  // Sender before using it.
  template <typename T>
  Sender<T> TryMakeSender(const std::string_view channel_name) {
    const Channel *channel = GetChannel<T>(channel_name);
    if (channel == nullptr) {
      return Sender<T>();
    }

    if (!configuration::ChannelIsSendableOnNode(channel, node())) {
      return Sender<T>();
    }

    return Sender<T>(MakeRawSender(channel));
  }

  // Makes class that allows constructing and sending messages to
  // the provided channel.
  template <typename T>
  Sender<T> MakeSender(const std::string_view channel_name) {
    CHECK(HasChannel<T>(channel_name))
        << ": Channel { \"name\": \"" << channel_name << "\", \"type\": \""
        << T::GetFullyQualifiedName() << "\" } not found in config for "
        << name()
        << (configuration::MultiNode(configuration())
                ? absl::StrCat(" on node ", node()->name()->string_view())
                : ".");

    Sender<T> result = TryMakeSender<T>(channel_name);
    if (!result) {
      LOG(FATAL) << "Channel { \"name\": \"" << channel_name
                 << "\", \"type\": \"" << T::GetFullyQualifiedName()
                 << "\" } is not able to be sent on this node.  Check your "
                    "configuration.";
    }

    return result;
  }

  // This will watch messages sent to the provided channel.
  //
  // w must have a non-polymorphic operator() (aka it can only be called with a
  // single set of arguments; no overloading or templates). It must be callable
  // with this signature:
  //   void(const MessageType &);
  //
  // Lambdas are a common form for w. A std::function will work too.
  //
  // Note that bind expressions have polymorphic call operators, so they are not
  // allowed.
  //
  // We template Watch as a whole instead of using std::function<void(const T
  // &)> to allow deducing MessageType from lambdas and other things which are
  // implicitly convertible to std::function, but not actually std::function
  // instantiations. Template deduction guides might allow solving this
  // differently in newer versions of C++, but those have their own corner
  // cases.
  template <typename Watch>
  void MakeWatcher(const std::string_view channel_name, Watch &&w);

  // Like MakeWatcher, but doesn't have access to the message data. This may be
  // implemented to use less resources than an equivalent MakeWatcher.
  //
  // The function will still have access to context(), although that will have
  // its data field set to nullptr.
  template <typename MessageType>
  void MakeNoArgWatcher(const std::string_view channel_name,
                        std::function<void()> w);

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
  // TODO(milind): callback should take the number of cycles elapsed as a
  // parameter.
  virtual TimerHandler *AddTimer(::std::function<void()> callback) = 0;

  // Creates a timer that executes callback periodically at the specified
  // interval and offset.  Returns a PhasedLoopHandler for interacting with the
  // timer.
  virtual PhasedLoopHandler *AddPhasedLoop(
      ::std::function<void(int)> callback,
      const monotonic_clock::duration interval,
      const monotonic_clock::duration offset = ::std::chrono::seconds(0)) = 0;

  // TODO(austin): OnExit for cleanup.

  // May be safely called from any thread.
  bool is_running() const { return is_running_.load(); }

  // Sets the scheduler priority to run the event loop at.  This may not be
  // called after we go into "real-time-mode".
  virtual void SetRuntimeRealtimePriority(int priority) = 0;
  // Defaults to 0 if this loop will not run realtime.
  virtual int runtime_realtime_priority() const = 0;

  static cpu_set_t DefaultAffinity();

  // Sets the scheduler affinity to run the event loop with. This may only be
  // called before Run().
  virtual void SetRuntimeAffinity(const cpu_set_t &cpuset) = 0;
  // Defaults to DefaultAffinity() if this loop will not run pinned.
  virtual const cpu_set_t &runtime_affinity() const = 0;

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

  // Watches channel (name, type) for new messages, without needing to extract
  // the message contents. Default implementation simply re-uses MakeRawWatcher.
  virtual void MakeRawNoArgWatcher(
      const Channel *channel,
      std::function<void(const Context &context)> watcher) {
    MakeRawWatcher(channel, [watcher](const Context &context, const void *) {
      Context new_context = context;
      new_context.data = nullptr;
      new_context.buffer_index = -1;
      watcher(new_context);
    });
  }

  // Creates a raw sender for the provided channel.  This is used for reflection
  // based sending.
  // Note: this ignores any node constraints.  Ignore at your own peril.
  virtual std::unique_ptr<RawSender> MakeRawSender(const Channel *channel) = 0;

  // Returns the context for the current callback.
  const Context &context() const { return context_; }

  // Returns the configuration that this event loop was built with.
  const Configuration *configuration() const { return configuration_; }

  // Prevents the event loop from sending a timing report.
  void SkipTimingReport();

  // Prevents AOS_LOG being sent to message on /aos.
  void SkipAosLog() { skip_logger_ = true; }

  // Returns the number of buffers for this channel. This corresponds with the
  // range of Context::buffer_index values for this channel.
  virtual int NumberBuffers(const Channel *channel) = 0;

  // Returns the boot UUID.
  virtual const UUID &boot_uuid() const = 0;

 protected:
  // Sets the name of the event loop.  This is the application name.
  virtual void set_name(const std::string_view name) = 0;

  void set_is_running(bool value) { is_running_.store(value); }

  // Validates that channel exists inside configuration_ and finds its index.
  int ChannelIndex(const Channel *channel);

  // Returns the state for the watcher on the corresponding channel. This
  // watcher must exist before calling this.
  WatcherState *GetWatcherState(const Channel *channel);

  // Returns a Sender's protected RawSender.
  template <typename T>
  static RawSender *GetRawSender(aos::Sender<T> *sender) {
    return sender->sender_.get();
  }

  // Returns a Fetcher's protected RawFetcher.
  template <typename T>
  static RawFetcher *GetRawFetcher(aos::Fetcher<T> *fetcher) {
    return fetcher->fetcher_.get();
  }

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

  // Tracks that we have a (single) watcher on the given channel.
  void TakeWatcher(const Channel *channel);
  // Tracks that we have at least one sender on the given channel.
  void TakeSender(const Channel *channel);

  std::vector<RawSender *> senders_;
  std::vector<RawFetcher *> fetchers_;

  std::vector<std::unique_ptr<TimerHandler>> timers_;
  std::vector<std::unique_ptr<PhasedLoopHandler>> phased_loops_;
  std::vector<std::unique_ptr<WatcherState>> watchers_;

  // Does nothing if timing reports are disabled.
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
  size_t event_generation_ = 1;

  // If true, don't send AOS_LOG to /aos
  bool skip_logger_ = false;

  // Sets context_ for a timed event which is supposed to happen at the provided
  // time.
  void SetTimerContext(monotonic_clock::time_point monotonic_event_time);
  // Clears context_ so it only has invalid times and elements in it.
  void ClearContext();

 private:
  virtual pid_t GetTid() = 0;

  FlatbufferDetachedBuffer<timing::Report> timing_report_;

  ::std::atomic<bool> is_running_{false};

  const Configuration *configuration_;

  // If true, don't send out timing reports.
  bool skip_timing_report_ = false;

  SendFailureCounter timing_report_failure_counter_;

  absl::btree_set<const Channel *> taken_watchers_, taken_senders_;
};

// Interface for terminating execution of an EventLoop.
//
// Prefer this over binding a lambda to an Exit() method when passing ownership
// in complicated ways because implementations should have assertions to catch
// it outliving the object it's referring to, instead of having a
// use-after-free.
//
// This is not exposed by EventLoop directly because different EventLoop
// implementations provide this functionality at different scopes, or possibly
// not at all.
class ExitHandle {
 public:
  ExitHandle() = default;
  virtual ~ExitHandle() = default;

  // Exits some set of event loops. Details depend on the implementation.
  //
  // This means no more events will be processed, but any currently being
  // processed will finish.
  virtual void Exit() = 0;
};

}  // namespace aos

#include "aos/events/event_loop_tmpl.h"  // IWYU pragma: export

#endif  // AOS_EVENTS_EVENT_LOOP_H
