#ifndef AOS_EVENTS_EVENT_LOOP_TMPL_H_
#define AOS_EVENTS_EVENT_LOOP_TMPL_H_

#include <inttypes.h>
#include <stdint.h>

#include <type_traits>

#include "aos/events/event_loop.h"
#include "glog/logging.h"

namespace aos {
namespace event_loop_internal {

// From a watch functor, specializations of this will extract the message type
// of the template argument. If T is not a valid message type, there will be no
// matching specialization.
//
// This is just the forward declaration, which will be used by one of the
// following specializations to match valid argument types.
template <class T>
struct watch_message_type_trait;

// From a watch functor, this will extract the message type of the argument.
// This is the template specialization.
template <class ClassType, class ReturnType, class A1>
struct watch_message_type_trait<ReturnType (ClassType::*)(A1) const> {
  using message_type = typename std::decay<A1>::type;
};

}  // namespace event_loop_internal

template <typename T>
typename Sender<T>::Builder Sender<T>::MakeBuilder() {
  return Builder(sender_.get(), sender_->fbb_allocator());
}

template <typename Watch>
void EventLoop::MakeWatcher(const std::string_view channel_name, Watch &&w) {
  using MessageType =
      typename event_loop_internal::watch_message_type_trait<decltype(
          &Watch::operator())>::message_type;
  const Channel *channel = configuration::GetChannel(
      configuration_, channel_name, MessageType::GetFullyQualifiedName(),
      name(), node());

  CHECK(channel != nullptr)
      << ": Channel { \"name\": \"" << channel_name << "\", \"type\": \""
      << MessageType::GetFullyQualifiedName()
      << "\" } not found in config for application " << name() << ".";

  MakeRawWatcher(channel,
                 [this, w](const Context &context, const void *message) {
                   context_ = context;
                   w(*flatbuffers::GetRoot<MessageType>(
                       reinterpret_cast<const char *>(message)));
                 });
}

template <typename MessageType>
void EventLoop::MakeNoArgWatcher(const std::string_view channel_name,
                                 std::function<void()> w) {
  const Channel *channel = configuration::GetChannel(
      configuration_, channel_name, MessageType::GetFullyQualifiedName(),
      name(), node());
  CHECK(channel != nullptr)
      << ": Channel { \"name\": \"" << channel_name << "\", \"type\": \""
      << MessageType::GetFullyQualifiedName()
      << "\" } not found in config for application " << name() << ".";
  MakeRawNoArgWatcher(channel, [this, w](const Context &context) {
    context_ = context;
    w();
  });
}

inline bool RawFetcher::FetchNext() {
  const auto result = DoFetchNext();
  if (result.first) {
    timing_.fetcher->mutate_count(timing_.fetcher->count() + 1);
    const monotonic_clock::time_point monotonic_time = result.second;
    ftrace_.FormatMessage(
        "%.*s: fetch next: now=%" PRId64 " event=%" PRId64 " queue=%" PRIu32,
        static_cast<int>(ftrace_prefix_.size()), ftrace_prefix_.data(),
        static_cast<int64_t>(monotonic_time.time_since_epoch().count()),
        static_cast<int64_t>(
            context_.monotonic_event_time.time_since_epoch().count()),
        context_.queue_index);
    const float latency =
        std::chrono::duration_cast<std::chrono::duration<float>>(
            monotonic_time - context_.monotonic_event_time)
            .count();
    timing_.latency.Add(latency);
    return true;
  }
  ftrace_.FormatMessage(
      "%.*s: fetch next: still event=%" PRId64 " queue=%" PRIu32,
      static_cast<int>(ftrace_prefix_.size()), ftrace_prefix_.data(),
      static_cast<int64_t>(
          context_.monotonic_event_time.time_since_epoch().count()),
      context_.queue_index);
  return false;
}

inline bool RawFetcher::Fetch() {
  const auto result = DoFetch();
  if (result.first) {
    timing_.fetcher->mutate_count(timing_.fetcher->count() + 1);
    const monotonic_clock::time_point monotonic_time = result.second;
    ftrace_.FormatMessage(
        "%.*s: fetch latest: now=%" PRId64 " event=%" PRId64 " queue=%" PRIu32,
        static_cast<int>(ftrace_prefix_.size()), ftrace_prefix_.data(),
        static_cast<int64_t>(monotonic_time.time_since_epoch().count()),
        static_cast<int64_t>(
            context_.monotonic_event_time.time_since_epoch().count()),
        context_.queue_index);
    const float latency =
        std::chrono::duration_cast<std::chrono::duration<float>>(
            monotonic_time - context_.monotonic_event_time)
            .count();
    timing_.latency.Add(latency);
    return true;
  }
  ftrace_.FormatMessage(
      "%.*s: fetch latest: still event=%" PRId64 " queue=%" PRIu32,
      static_cast<int>(ftrace_prefix_.size()), ftrace_prefix_.data(),
      static_cast<int64_t>(
          context_.monotonic_event_time.time_since_epoch().count()),
      context_.queue_index);
  return false;
}

inline bool RawSender::Send(size_t size) {
  return Send(size, monotonic_clock::min_time, realtime_clock::min_time,
              0xffffffffu, event_loop_->boot_uuid());
}

inline bool RawSender::Send(
    size_t size, aos::monotonic_clock::time_point monotonic_remote_time,
    aos::realtime_clock::time_point realtime_remote_time,
    uint32_t remote_queue_index, const UUID &uuid) {
  if (DoSend(size, monotonic_remote_time, realtime_remote_time,
             remote_queue_index, uuid)) {
    timing_.size.Add(size);
    timing_.sender->mutate_count(timing_.sender->count() + 1);
    ftrace_.FormatMessage(
        "%.*s: sent internal: event=%" PRId64 " queue=%" PRIu32,
        static_cast<int>(ftrace_prefix_.size()), ftrace_prefix_.data(),
        static_cast<int64_t>(monotonic_sent_time().time_since_epoch().count()),
        sent_queue_index());
    return true;
  }
  return false;
}

inline bool RawSender::Send(const void *data, size_t size) {
  return Send(data, size, monotonic_clock::min_time, realtime_clock::min_time,
              0xffffffffu, event_loop_->boot_uuid());
}

inline bool RawSender::Send(
    const void *data, size_t size,
    aos::monotonic_clock::time_point monotonic_remote_time,
    aos::realtime_clock::time_point realtime_remote_time,
    uint32_t remote_queue_index, const UUID &uuid) {
  if (DoSend(data, size, monotonic_remote_time, realtime_remote_time,
             remote_queue_index, uuid)) {
    timing_.size.Add(size);
    timing_.sender->mutate_count(timing_.sender->count() + 1);
    ftrace_.FormatMessage(
        "%.*s: sent external: event=%" PRId64 " queue=%" PRIu32,
        static_cast<int>(ftrace_prefix_.size()), ftrace_prefix_.data(),
        static_cast<int64_t>(monotonic_sent_time().time_since_epoch().count()),
        sent_queue_index());
    return true;
  }
  return false;
}

inline monotonic_clock::time_point TimerHandler::Call(
    std::function<monotonic_clock::time_point()> get_time,
    monotonic_clock::time_point event_time) {
  CHECK_NOTNULL(timing_.timer);
  const monotonic_clock::time_point monotonic_start_time = get_time();

  event_loop_->context_.monotonic_event_time = event_time;
  event_loop_->context_.monotonic_remote_time = monotonic_clock::min_time;
  event_loop_->context_.realtime_remote_time =
      event_loop_->context_.realtime_event_time = realtime_clock::min_time;
  event_loop_->context_.queue_index = 0xffffffffu;
  event_loop_->context_.size = 0;
  event_loop_->context_.data = nullptr;
  event_loop_->context_.buffer_index = -1;
  event_loop_->context_.remote_boot_uuid = event_loop_->boot_uuid();

  ftrace_.FormatMessage(
      "timer: %.*s: start now=%" PRId64 " event=%" PRId64,
      static_cast<int>(name_.size()), name_.data(),
      static_cast<int64_t>(monotonic_start_time.time_since_epoch().count()),
      static_cast<int64_t>(event_time.time_since_epoch().count()));
  {
    const float start_latency =
        std::chrono::duration_cast<std::chrono::duration<float>>(
            monotonic_start_time - event_time)
            .count();
    timing_.wakeup_latency.Add(start_latency);
  }
  timing_.timer->mutate_count(timing_.timer->count() + 1);
  fn_();

  const monotonic_clock::time_point monotonic_end_time = get_time();
  ftrace_.FormatMessage(
      "timer: %.*s: end now=%" PRId64, static_cast<int>(name_.size()),
      name_.data(),
      static_cast<int64_t>(monotonic_end_time.time_since_epoch().count()));

  const float handler_latency =
      std::chrono::duration_cast<std::chrono::duration<float>>(
          monotonic_end_time - monotonic_start_time)
          .count();
  timing_.handler_time.Add(handler_latency);
  return monotonic_start_time;
}

inline void PhasedLoopHandler::Call(
    std::function<monotonic_clock::time_point()> get_time,
    std::function<void(monotonic_clock::time_point)> schedule) {
  // Read time directly to save a vtable indirection...
  const monotonic_clock::time_point monotonic_start_time = get_time();

  // Update the context to hold the desired wakeup time.
  event_loop_->context_.monotonic_event_time = phased_loop_.sleep_time();
  event_loop_->context_.monotonic_remote_time = monotonic_clock::min_time;
  event_loop_->context_.realtime_remote_time =
      event_loop_->context_.realtime_event_time = realtime_clock::min_time;
  event_loop_->context_.queue_index = 0xffffffffu;
  event_loop_->context_.size = 0;
  event_loop_->context_.data = nullptr;
  event_loop_->context_.buffer_index = -1;
  event_loop_->context_.remote_boot_uuid = event_loop_->boot_uuid();

  // Compute how many cycles elapsed and schedule the next wakeup.
  Reschedule(schedule, monotonic_start_time);

  ftrace_.FormatMessage(
      "phased: %.*s: start now=%" PRId64 " event=%" PRId64 " cycles=%d",
      static_cast<int>(name_.size()), name_.data(),
      static_cast<int64_t>(monotonic_start_time.time_since_epoch().count()),
      static_cast<int64_t>(
          phased_loop_.sleep_time().time_since_epoch().count()),
      cycles_elapsed_);
  {
    const float start_latency =
        std::chrono::duration_cast<std::chrono::duration<float>>(
            monotonic_start_time - event_loop_->context_.monotonic_event_time)
            .count();
    timing_.wakeup_latency.Add(start_latency);
  }
  timing_.timer->mutate_count(timing_.timer->count() + 1);

  // Call the function with the elapsed cycles.
  fn_(cycles_elapsed_);
  cycles_elapsed_ = 0;

  const monotonic_clock::time_point monotonic_end_time = get_time();
  ftrace_.FormatMessage(
      "phased: %.*s: end now=%" PRId64, static_cast<int>(name_.size()),
      name_.data(),
      static_cast<int64_t>(monotonic_end_time.time_since_epoch().count()));

  const float handler_latency =
      std::chrono::duration_cast<std::chrono::duration<float>>(
          monotonic_end_time - monotonic_start_time)
          .count();
  timing_.handler_time.Add(handler_latency);

  // If the handler took too long so we blew by the previous deadline, we
  // want to just try for the next deadline.  Reschedule.
  if (monotonic_end_time > phased_loop_.sleep_time()) {
    Reschedule(schedule, monotonic_end_time);
  }
}

// Class to automate the timing report generation for watchers.
class WatcherState {
 public:
  WatcherState(
      EventLoop *event_loop, const Channel *channel,
      std::function<void(const Context &context, const void *message)> fn)
      : channel_index_(event_loop->ChannelIndex(channel)),
        ftrace_prefix_(configuration::StrippedChannelToString(channel)),
        fn_(std::move(fn)) {}

  virtual ~WatcherState() {}

  // Calls the callback, measuring time with get_time, with the provided
  // context.
  void DoCallCallback(std::function<monotonic_clock::time_point()> get_time,
                      Context context) {
    if (context.data) {
      CheckChannelDataAlignment(context.data, context.size);
    }
    const monotonic_clock::time_point monotonic_start_time = get_time();
    ftrace_.FormatMessage(
        "%.*s: watcher start: now=%" PRId64 " event=%" PRId64 " queue=%" PRIu32,
        static_cast<int>(ftrace_prefix_.size()), ftrace_prefix_.data(),
        static_cast<int64_t>(monotonic_start_time.time_since_epoch().count()),
        static_cast<int64_t>(
            context.monotonic_event_time.time_since_epoch().count()),
        context.queue_index);
    {
      const float start_latency =
          std::chrono::duration_cast<std::chrono::duration<float>>(
              monotonic_start_time - context.monotonic_event_time)
              .count();
      wakeup_latency_.Add(start_latency);
    }
    watcher_->mutate_count(watcher_->count() + 1);
    fn_(context, context.data);

    const monotonic_clock::time_point monotonic_end_time = get_time();
    ftrace_.FormatMessage(
        "%.*s: watcher end: now=%" PRId64,
        static_cast<int>(ftrace_prefix_.size()), ftrace_prefix_.data(),
        static_cast<int64_t>(monotonic_end_time.time_since_epoch().count()));

    const float handler_latency =
        std::chrono::duration_cast<std::chrono::duration<float>>(
            monotonic_end_time - monotonic_start_time)
            .count();
    handler_time_.Add(handler_latency);
  }

  int channel_index() const { return channel_index_; }

  void set_timing_report(timing::Watcher *watcher);
  void ResetReport();

  virtual void Startup(EventLoop *event_loop) = 0;

 protected:
  const int channel_index_;
  const std::string ftrace_prefix_;

  const std::function<void(const Context &context, const void *message)> fn_;

  internal::TimingStatistic wakeup_latency_;
  internal::TimingStatistic handler_time_;
  timing::Watcher *watcher_ = nullptr;

  Ftrace ftrace_;
};

template <typename T>
bool Sender<T>::Send(const NonSizePrefixedFlatbuffer<T> &flatbuffer) {
  return sender_->Send(flatbuffer.span().data(), flatbuffer.span().size());
}

template <typename T>
bool Sender<T>::SendDetached(FlatbufferDetachedBuffer<T> detached) {
  CHECK_EQ(static_cast<void *>(detached.span().data() + detached.span().size() -
                               sender_->size()),
           sender_->data())
      << ": May only send the buffer detached from this Sender";
  return sender_->Send(detached.span().size());
}

}  // namespace aos

#endif  // AOS_EVENTS_EVENT_LOOP_TMPL_H
