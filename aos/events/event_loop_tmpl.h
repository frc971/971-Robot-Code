#ifndef AOS_EVENTS_EVENT_LOOP_TMPL_H_
#define AOS_EVENTS_EVENT_LOOP_TMPL_H_

#include <type_traits>
#include "aos/events/event_loop.h"
#include "glog/logging.h"

namespace aos {

// From a watch functor, this will extract the message type of the argument.
// This is the template forward declaration, and it extracts the call operator
// as a PTMF to be used by the following specialization.
template <class T>
struct watch_message_type_trait
    : watch_message_type_trait<decltype(&T::operator())> {};

// From a watch functor, this will extract the message type of the argument.
// This is the template specialization.
template <class ClassType, class ReturnType, class A1>
struct watch_message_type_trait<ReturnType (ClassType::*)(A1) const> {
  using message_type = typename std::decay<A1>::type;
};

template <typename T>
typename Sender<T>::Builder Sender<T>::MakeBuilder() {
  return Builder(sender_.get(), sender_->data(), sender_->size());
}

template <typename Watch>
void EventLoop::MakeWatcher(const std::string_view channel_name, Watch &&w) {
  using T = typename watch_message_type_trait<Watch>::message_type;
  const Channel *channel = configuration::GetChannel(
      configuration_, channel_name, T::GetFullyQualifiedName(), name(), node());

  CHECK(channel != nullptr)
      << ": Channel { \"name\": \"" << channel_name << "\", \"type\": \""
      << T::GetFullyQualifiedName() << "\" } not found in config.";

  if (!configuration::ChannelIsReadableOnNode(channel, node())) {
    LOG(FATAL) << "Channel { \"name\": \"" << channel_name << "\", \"type\": \""
               << T::GetFullyQualifiedName()
               << "\" } is not able to be watched on this node.  Check your "
                  "configuration.";
  }

  return MakeRawWatcher(
      channel, [this, w](const Context &context, const void *message) {
        context_ = context;
        w(*flatbuffers::GetRoot<T>(reinterpret_cast<const char *>(message)));
      });
}

inline bool RawFetcher::FetchNext() {
  const auto result = DoFetchNext();
  if (result.first) {
    timing_.fetcher->mutate_count(timing_.fetcher->count() + 1);
    const monotonic_clock::time_point monotonic_time = result.second;
    const float latency =
        std::chrono::duration_cast<std::chrono::duration<float>>(
            monotonic_time - context_.monotonic_event_time)
            .count();
    timing_.latency.Add(latency);
    return true;
  }
  return false;
}

inline bool RawFetcher::Fetch() {
  const auto result = DoFetch();
  if (result.first) {
    timing_.fetcher->mutate_count(timing_.fetcher->count() + 1);
    const monotonic_clock::time_point monotonic_time = result.second;
    const float latency =
        std::chrono::duration_cast<std::chrono::duration<float>>(
            monotonic_time - context_.monotonic_event_time)
            .count();
    timing_.latency.Add(latency);
    return true;
  }
  return false;
}

inline bool RawSender::Send(
    size_t size, aos::monotonic_clock::time_point monotonic_remote_time,
    aos::realtime_clock::time_point realtime_remote_time,
    uint32_t remote_queue_index) {
  if (DoSend(size, monotonic_remote_time, realtime_remote_time,
             remote_queue_index)) {
    timing_.size.Add(size);
    timing_.sender->mutate_count(timing_.sender->count() + 1);
    return true;
  }
  return false;
}

inline bool RawSender::Send(
    const void *data, size_t size,
    aos::monotonic_clock::time_point monotonic_remote_time,
    aos::realtime_clock::time_point realtime_remote_time,
    uint32_t remote_queue_index) {
  if (DoSend(data, size, monotonic_remote_time, realtime_remote_time,
             remote_queue_index)) {
    timing_.size.Add(size);
    timing_.sender->mutate_count(timing_.sender->count() + 1);
    return true;
  }
  return false;
}

inline void TimerHandler::Call(
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

  const float handler_latency =
      std::chrono::duration_cast<std::chrono::duration<float>>(
          monotonic_end_time - monotonic_start_time)
          .count();
  timing_.handler_time.Add(handler_latency);
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

  // Compute how many cycles elapsed and schedule the next wakeup.
  Reschedule(schedule, monotonic_start_time);

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

  const float handler_latency =
      std::chrono::duration_cast<std::chrono::duration<float>>(
          monotonic_end_time - monotonic_start_time)
          .count();
  timing_.handler_time.Add(handler_latency);

  // If the handler too too long so we blew by the previous deadline, we
  // want to just try for the next deadline.  Rescuedule.
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
      : channel_index_(event_loop->ChannelIndex(channel)), fn_(std::move(fn)) {}

  virtual ~WatcherState() {}

  // Calls the callback, measuring time with get_time, with the provided
  // context.
  void DoCallCallback(std::function<monotonic_clock::time_point()> get_time,
                      Context context) {
    const monotonic_clock::time_point monotonic_start_time = get_time();
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

  std::function<void(const Context &context, const void *message)> fn_;

  internal::TimingStatistic wakeup_latency_;
  internal::TimingStatistic handler_time_;
  timing::Watcher *watcher_ = nullptr;
};

template <typename T>
bool Sender<T>::Send(const Flatbuffer<T> &flatbuffer) {
  return sender_->Send(flatbuffer.data(), flatbuffer.size());
}

}  // namespace aos

#endif  // AOS_EVENTS_EVENT_LOOP_TMPL_H
