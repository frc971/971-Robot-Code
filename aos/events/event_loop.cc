#include "aos/events/event_loop.h"

#include "glog/logging.h"

#include "aos/configuration.h"
#include "aos/configuration_generated.h"
#include "aos/logging/implementations.h"
#include "aos/realtime.h"

DEFINE_bool(timing_reports, true, "Publish timing reports.");
DEFINE_int32(timing_report_ms, 1000,
             "Period in milliseconds to publish timing reports at.");

namespace aos {
namespace {
void CheckAlignment(const Channel *channel) {
  if (channel->max_size() % alignof(flatbuffers::largest_scalar_t) != 0) {
    LOG(FATAL) << "max_size() (" << channel->max_size()
               << ") is not a multiple of alignment ("
               << alignof(flatbuffers::largest_scalar_t) << ") for channel "
               << configuration::CleanedChannelToString(channel) << ".";
  }
}

std::string_view ErrorToString(const RawSender::Error err) {
  switch (err) {
    case RawSender::Error::kOk:
      return "RawSender::Error::kOk";
    case RawSender::Error::kMessagesSentTooFast:
      return "RawSender::Error::kMessagesSentTooFast";
    case RawSender::Error::kInvalidRedzone:
      return "RawSender::Error::kInvalidRedzone";
  }
  LOG(FATAL) << "Unknown error given with code " << static_cast<int>(err);
}
}  // namespace

std::optional<std::string> EventLoop::default_version_string_;

std::pair<SharedSpan, absl::Span<uint8_t>> MakeSharedSpan(size_t size) {
  AlignedOwningSpan *const span = reinterpret_cast<AlignedOwningSpan *>(
      malloc(sizeof(AlignedOwningSpan) + size + kChannelDataAlignment - 1));

  absl::Span<uint8_t> mutable_span(
      reinterpret_cast<uint8_t *>(RoundChannelData(span->data(), size)), size);
  // Use the placement new operator to construct an actual absl::Span in place.
  new (span) AlignedOwningSpan(mutable_span);

  return std::make_pair(
      SharedSpan(std::shared_ptr<AlignedOwningSpan>(span,
                                                    [](AlignedOwningSpan *s) {
                                                      s->~AlignedOwningSpan();
                                                      free(s);
                                                    }),
                 &span->span),
      mutable_span);
}

std::ostream &operator<<(std::ostream &os, const RawSender::Error err) {
  os << ErrorToString(err);
  return os;
}

void RawSender::CheckOk(const RawSender::Error err) {
  CHECK_EQ(err, Error::kOk) << "Messages were sent too fast on channel: "
                            << configuration::CleanedChannelToString(channel_);
}

RawSender::RawSender(EventLoop *event_loop, const Channel *channel)
    : event_loop_(event_loop),
      channel_(channel),
      ftrace_prefix_(configuration::StrippedChannelToString(channel)),
      timing_(event_loop_->ChannelIndex(channel)) {
  event_loop_->NewSender(this);
}

RawSender::~RawSender() { event_loop_->DeleteSender(this); }

RawSender::Error RawSender::DoSend(
    const SharedSpan data, monotonic_clock::time_point monotonic_remote_time,
    realtime_clock::time_point realtime_remote_time,
    uint32_t remote_queue_index, const UUID &source_boot_uuid) {
  return DoSend(data->data(), data->size(), monotonic_remote_time,
                realtime_remote_time, remote_queue_index, source_boot_uuid);
}

void RawSender::RecordSendResult(const Error error, size_t message_size) {
  switch (error) {
    case Error::kOk: {
      if (timing_.sender) {
        timing_.size.Add(message_size);
        timing_.sender->mutate_count(timing_.sender->count() + 1);
      }
      break;
    }
    case Error::kMessagesSentTooFast:
      timing_.IncrementError(timing::SendError::MESSAGE_SENT_TOO_FAST);
      break;
    case Error::kInvalidRedzone:
      timing_.IncrementError(timing::SendError::INVALID_REDZONE);
      break;
  }
}

RawFetcher::RawFetcher(EventLoop *event_loop, const Channel *channel)
    : event_loop_(event_loop),
      channel_(channel),
      ftrace_prefix_(configuration::StrippedChannelToString(channel)),
      timing_(event_loop_->ChannelIndex(channel)) {
  context_.monotonic_event_time = monotonic_clock::min_time;
  context_.monotonic_remote_time = monotonic_clock::min_time;
  context_.realtime_event_time = realtime_clock::min_time;
  context_.realtime_remote_time = realtime_clock::min_time;
  context_.queue_index = 0xffffffff;
  context_.remote_queue_index = 0xffffffffu;
  context_.size = 0;
  context_.data = nullptr;
  context_.buffer_index = -1;
  event_loop_->NewFetcher(this);
}

RawFetcher::~RawFetcher() { event_loop_->DeleteFetcher(this); }

TimerHandler::TimerHandler(EventLoop *event_loop, std::function<void()> fn)
    : event_loop_(event_loop), fn_(std::move(fn)) {}

TimerHandler::~TimerHandler() {}

PhasedLoopHandler::PhasedLoopHandler(EventLoop *event_loop,
                                     std::function<void(int)> fn,
                                     const monotonic_clock::duration interval,
                                     const monotonic_clock::duration offset)
    : event_loop_(event_loop),
      fn_(std::move(fn)),
      phased_loop_(interval, event_loop_->monotonic_now(), offset) {
  event_loop_->OnRun([this]() {
    const monotonic_clock::time_point monotonic_now =
        event_loop_->monotonic_now();
    phased_loop_.Reset(monotonic_now);
    Reschedule(monotonic_now);
    // Reschedule here will count cycles elapsed before now, and then the
    // reschedule before running the handler will count the time that elapsed
    // then. So clear the count here.
    cycles_elapsed_ = 0;
  });
}

PhasedLoopHandler::~PhasedLoopHandler() {}

EventLoop::EventLoop(const Configuration *configuration)
    : version_string_(default_version_string_),
      timing_report_(flatbuffers::DetachedBuffer()),
      configuration_(configuration) {}

EventLoop::~EventLoop() {
  if (!senders_.empty()) {
    for (const RawSender *sender : senders_) {
      LOG(ERROR) << "  Sender "
                 << configuration::StrippedChannelToString(sender->channel())
                 << " still open";
    }
  }
  CHECK_EQ(senders_.size(), 0u) << ": Not all senders destroyed";
  CHECK_EQ(events_.size(), 0u) << ": Not all events unregistered";
}

void EventLoop::SkipTimingReport() {
  skip_timing_report_ = true;
  timing_report_ = flatbuffers::DetachedBuffer();

  for (size_t i = 0; i < timers_.size(); ++i) {
    timers_[i]->timing_.set_timing_report(nullptr);
  }

  for (size_t i = 0; i < phased_loops_.size(); ++i) {
    phased_loops_[i]->timing_.set_timing_report(nullptr);
  }

  for (size_t i = 0; i < watchers_.size(); ++i) {
    watchers_[i]->set_timing_report(nullptr);
  }

  for (size_t i = 0; i < senders_.size(); ++i) {
    senders_[i]->timing_.set_timing_report(nullptr);
  }

  for (size_t i = 0; i < fetchers_.size(); ++i) {
    fetchers_[i]->timing_.set_timing_report(nullptr);
  }
}

int EventLoop::ChannelIndex(const Channel *channel) {
  return configuration::ChannelIndex(configuration_, channel);
}

WatcherState *EventLoop::GetWatcherState(const Channel *channel) {
  const int channel_index = ChannelIndex(channel);
  for (const std::unique_ptr<WatcherState> &watcher : watchers_) {
    if (watcher->channel_index() == channel_index) {
      return watcher.get();
    }
  }
  LOG(FATAL) << "No watcher found for channel";
}

void EventLoop::NewSender(RawSender *sender) {
  senders_.emplace_back(sender);
  UpdateTimingReport();
}
void EventLoop::DeleteSender(RawSender *sender) {
  CHECK(!is_running());
  auto s = std::find(senders_.begin(), senders_.end(), sender);
  CHECK(s != senders_.end()) << ": Sender not in senders list";
  senders_.erase(s);
  UpdateTimingReport();
}

TimerHandler *EventLoop::NewTimer(std::unique_ptr<TimerHandler> timer) {
  timers_.emplace_back(std::move(timer));
  UpdateTimingReport();
  return timers_.back().get();
}

PhasedLoopHandler *EventLoop::NewPhasedLoop(
    std::unique_ptr<PhasedLoopHandler> phased_loop) {
  phased_loops_.emplace_back(std::move(phased_loop));
  UpdateTimingReport();
  return phased_loops_.back().get();
}

void EventLoop::NewFetcher(RawFetcher *fetcher) {
  CheckAlignment(fetcher->channel());

  fetchers_.emplace_back(fetcher);
  UpdateTimingReport();
}

void EventLoop::DeleteFetcher(RawFetcher *fetcher) {
  CHECK(!is_running());
  auto f = std::find(fetchers_.begin(), fetchers_.end(), fetcher);
  CHECK(f != fetchers_.end()) << ": Fetcher not in fetchers list";
  fetchers_.erase(f);
  UpdateTimingReport();
}

WatcherState *EventLoop::NewWatcher(std::unique_ptr<WatcherState> watcher) {
  watchers_.emplace_back(std::move(watcher));

  UpdateTimingReport();

  return watchers_.back().get();
}

void EventLoop::TakeWatcher(const Channel *channel) {
  CHECK(!is_running()) << ": Cannot add new objects while running.";
  ChannelIndex(channel);

  CheckAlignment(channel);

  CHECK(taken_senders_.find(channel) == taken_senders_.end())
      << ": " << configuration::CleanedChannelToString(channel)
      << " is already being used for sending. Can't make a watcher on the "
         "same event loop.";

  auto result = taken_watchers_.insert(channel);
  CHECK(result.second) << ": " << configuration::CleanedChannelToString(channel)
                       << " is already being used.";

  if (!configuration::ChannelIsReadableOnNode(channel, node())) {
    LOG(FATAL) << ": " << configuration::CleanedChannelToString(channel)
               << " is not able to be watched on this node.  Check your "
                  "configuration.";
  }
}

void EventLoop::TakeSender(const Channel *channel) {
  CHECK(!is_running()) << ": Cannot add new objects while running.";
  ChannelIndex(channel);

  CheckAlignment(channel);

  CHECK(taken_watchers_.find(channel) == taken_watchers_.end())
      << ": Channel " << configuration::CleanedChannelToString(channel)
      << " is already being used.";

  // We don't care if this is a duplicate.
  taken_senders_.insert(channel);
}

void EventLoop::SendTimingReport() {
  if (!timing_report_sender_) {
    // Timing reports are disabled, so nothing for us to do.
    return;
  }

  // We need to do a fancy dance here to get all the accounting to work right.
  // We want to copy the memory here, but then send after resetting. Otherwise
  // the send for the timing report won't be counted in the timing report.
  //
  // Also, flatbuffers build from the back end.  So place this at the back end
  // of the buffer.  We only have to care because we are using this in a very
  // raw fashion.
  CHECK_LE(timing_report_.span().size(), timing_report_sender_->size())
      << ": Timing report bigger than the sender size for " << name() << ".";
  std::copy(timing_report_.span().data(),
            timing_report_.span().data() + timing_report_.span().size(),
            reinterpret_cast<uint8_t *>(timing_report_sender_->data()) +
                timing_report_sender_->size() - timing_report_.span().size());

  for (const std::unique_ptr<TimerHandler> &timer : timers_) {
    timer->timing_.ResetTimingReport();
  }
  for (const std::unique_ptr<WatcherState> &watcher : watchers_) {
    watcher->ResetReport();
  }
  for (const std::unique_ptr<PhasedLoopHandler> &phased_loop : phased_loops_) {
    phased_loop->timing_.ResetTimingReport();
  }
  for (RawSender *sender : senders_) {
    sender->timing_.ResetTimingReport();
  }
  for (RawFetcher *fetcher : fetchers_) {
    fetcher->timing_.ResetTimingReport();
  }
  // TODO(milind): If we fail to send, we don't want to reset the timing report.
  // We would need to move the reset after the send, and then find the correct
  // timing report and set the reports with it instead of letting the sender do
  // this. If we failed to send, we wouldn't reset or set the reports, so they
  // can accumalate until the next send.
  timing_report_failure_counter_.Count(
      timing_report_sender_->Send(timing_report_.span().size()));
}

void EventLoop::UpdateTimingReport() {
  if (skip_timing_report_) {
    return;
  }

  // We need to support senders and fetchers changing while we are setting up
  // the event loop.  Otherwise we can't fetch or send until the loop runs. This
  // means that on each change, we need to redo all this work.  This makes setup
  // more expensive, but not by all that much on a modern processor.

  // Now, build up a report with everything pre-filled out.
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);

  // Pre-fill in the defaults for timers.
  std::vector<flatbuffers::Offset<timing::Timer>> timer_offsets;
  for (const std::unique_ptr<TimerHandler> &timer : timers_) {
    flatbuffers::Offset<timing::Statistic> wakeup_latency_offset =
        timing::CreateStatistic(fbb);
    flatbuffers::Offset<timing::Statistic> handler_time_offset =
        timing::CreateStatistic(fbb);
    flatbuffers::Offset<flatbuffers::String> name_offset;
    if (timer->name().size() != 0) {
      name_offset = fbb.CreateString(timer->name());
    }

    timing::Timer::Builder timer_builder(fbb);

    if (timer->name().size() != 0) {
      timer_builder.add_name(name_offset);
    }
    timer_builder.add_wakeup_latency(wakeup_latency_offset);
    timer_builder.add_handler_time(handler_time_offset);
    timer_builder.add_count(0);
    timer_offsets.emplace_back(timer_builder.Finish());
  }

  // Pre-fill in the defaults for phased_loops.
  std::vector<flatbuffers::Offset<timing::Timer>> phased_loop_offsets;
  for (const std::unique_ptr<PhasedLoopHandler> &phased_loop : phased_loops_) {
    flatbuffers::Offset<timing::Statistic> wakeup_latency_offset =
        timing::CreateStatistic(fbb);
    flatbuffers::Offset<timing::Statistic> handler_time_offset =
        timing::CreateStatistic(fbb);
    flatbuffers::Offset<flatbuffers::String> name_offset;
    if (phased_loop->name().size() != 0) {
      name_offset = fbb.CreateString(phased_loop->name());
    }

    timing::Timer::Builder timer_builder(fbb);

    if (phased_loop->name().size() != 0) {
      timer_builder.add_name(name_offset);
    }
    timer_builder.add_wakeup_latency(wakeup_latency_offset);
    timer_builder.add_handler_time(handler_time_offset);
    timer_builder.add_count(0);
    phased_loop_offsets.emplace_back(timer_builder.Finish());
  }

  // Pre-fill in the defaults for watchers.
  std::vector<flatbuffers::Offset<timing::Watcher>> watcher_offsets;
  for (const std::unique_ptr<WatcherState> &watcher : watchers_) {
    flatbuffers::Offset<timing::Statistic> wakeup_latency_offset =
        timing::CreateStatistic(fbb);
    flatbuffers::Offset<timing::Statistic> handler_time_offset =
        timing::CreateStatistic(fbb);

    timing::Watcher::Builder watcher_builder(fbb);

    watcher_builder.add_channel_index(watcher->channel_index());
    watcher_builder.add_wakeup_latency(wakeup_latency_offset);
    watcher_builder.add_handler_time(handler_time_offset);
    watcher_builder.add_count(0);
    watcher_offsets.emplace_back(watcher_builder.Finish());
  }

  // Pre-fill in the defaults for senders.
  std::vector<flatbuffers::Offset<timing::Sender>> sender_offsets;
  for (RawSender *sender : senders_) {
    flatbuffers::Offset<timing::Statistic> size_offset =
        timing::CreateStatistic(fbb);

    const flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<timing::SendErrorCount>>>
        error_counts_offset = sender->timing_.error_counter.Initialize(&fbb);

    timing::Sender::Builder sender_builder(fbb);

    sender_builder.add_channel_index(sender->timing_.channel_index);
    sender_builder.add_size(size_offset);
    sender_builder.add_error_counts(error_counts_offset);
    sender_builder.add_count(0);
    sender_offsets.emplace_back(sender_builder.Finish());
  }

  // Pre-fill in the defaults for fetchers.
  std::vector<flatbuffers::Offset<timing::Fetcher>> fetcher_offsets;
  for (RawFetcher *fetcher : fetchers_) {
    flatbuffers::Offset<timing::Statistic> latency_offset =
        timing::CreateStatistic(fbb);

    timing::Fetcher::Builder fetcher_builder(fbb);

    fetcher_builder.add_channel_index(fetcher->timing_.channel_index);
    fetcher_builder.add_count(0);
    fetcher_builder.add_latency(latency_offset);
    fetcher_offsets.emplace_back(fetcher_builder.Finish());
  }

  // Then build the final report.
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<timing::Timer>>>
      timers_offset;
  if (timer_offsets.size() > 0) {
    timers_offset = fbb.CreateVector(timer_offsets);
  }

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<timing::Timer>>>
      phased_loops_offset;
  if (phased_loop_offsets.size() > 0) {
    phased_loops_offset = fbb.CreateVector(phased_loop_offsets);
  }

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<timing::Watcher>>>
      watchers_offset;
  if (watcher_offsets.size() > 0) {
    watchers_offset = fbb.CreateVector(watcher_offsets);
  }

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<timing::Sender>>>
      senders_offset;
  if (sender_offsets.size() > 0) {
    senders_offset = fbb.CreateVector(sender_offsets);
  }

  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<timing::Fetcher>>>
      fetchers_offset;
  if (fetcher_offsets.size() > 0) {
    fetchers_offset = fbb.CreateVector(fetcher_offsets);
  }

  flatbuffers::Offset<flatbuffers::String> name_offset =
      fbb.CreateString(name());

  const flatbuffers::Offset<flatbuffers::String> version_offset =
      version_string_.has_value() ? fbb.CreateString(version_string_.value())
                                  : flatbuffers::Offset<flatbuffers::String>();

  timing::Report::Builder report_builder(fbb);
  report_builder.add_name(name_offset);
  report_builder.add_version(version_offset);
  report_builder.add_pid(GetTid());
  if (timer_offsets.size() > 0) {
    report_builder.add_timers(timers_offset);
  }
  if (phased_loop_offsets.size() > 0) {
    report_builder.add_phased_loops(phased_loops_offset);
  }
  if (watcher_offsets.size() > 0) {
    report_builder.add_watchers(watchers_offset);
  }
  if (sender_offsets.size() > 0) {
    report_builder.add_senders(senders_offset);
  }
  if (fetcher_offsets.size() > 0) {
    report_builder.add_fetchers(fetchers_offset);
  }
  report_builder.add_send_failures(timing_report_failure_counter_.failures());
  fbb.Finish(report_builder.Finish());

  timing_report_ = FlatbufferDetachedBuffer<timing::Report>(fbb.Release());

  // Now that the pointers are stable, pass them to the timers and watchers to
  // be updated.
  for (size_t i = 0; i < timers_.size(); ++i) {
    timers_[i]->timing_.set_timing_report(
        timing_report_.mutable_message()->mutable_timers()->GetMutableObject(
            i));
  }

  for (size_t i = 0; i < phased_loops_.size(); ++i) {
    phased_loops_[i]->timing_.set_timing_report(
        timing_report_.mutable_message()
            ->mutable_phased_loops()
            ->GetMutableObject(i));
  }

  for (size_t i = 0; i < watchers_.size(); ++i) {
    watchers_[i]->set_timing_report(
        timing_report_.mutable_message()->mutable_watchers()->GetMutableObject(
            i));
  }

  for (size_t i = 0; i < senders_.size(); ++i) {
    senders_[i]->timing_.set_timing_report(
        timing_report_.mutable_message()->mutable_senders()->GetMutableObject(
            i));
  }

  for (size_t i = 0; i < fetchers_.size(); ++i) {
    fetchers_[i]->timing_.set_timing_report(
        timing_report_.mutable_message()->mutable_fetchers()->GetMutableObject(
            i));
  }
}

void EventLoop::MaybeScheduleTimingReports() {
  if (FLAGS_timing_reports && !skip_timing_report_) {
    CHECK(!timing_report_sender_) << ": Timing reports already scheduled.";
    // Make a raw sender for the report.
    const Channel *channel = configuration::GetChannel(
        configuration(), "/aos", timing::Report::GetFullyQualifiedName(),
        name(), node());
    CHECK(channel != nullptr) << ": Failed to look up {\"name\": \"/aos\", "
                                 "\"type\": \"aos.timing.Report\"} on node "
                              << FlatbufferToJson(node());

    // Since we are using a RawSender, validity isn't checked.  So check it
    // ourselves.
    if (!configuration::ChannelIsSendableOnNode(channel, node())) {
      LOG(FATAL) << "Channel { \"name\": \"/aos"
                 << channel->name()->string_view() << "\", \"type\": \""
                 << channel->type()->string_view()
                 << "\" } is not able to be sent on this node.  Check your "
                    "configuration.";
    }
    CHECK(channel != nullptr) << ": Channel { \"name\": \"/aos\", \"type\": \""
                              << timing::Report::GetFullyQualifiedName()
                              << "\" } not found in config.";
    timing_report_sender_ = MakeRawSender(channel);

    // Register a handler which sends the report out by copying the raw data
    // from the prebuilt and subsequently modified report.
    TimerHandler *timing_reports_timer =
        AddTimer([this]() { SendTimingReport(); });

    // Set it up to send once per second.
    timing_reports_timer->set_name("timing_reports");
    OnRun([this, timing_reports_timer]() {
      timing_reports_timer->Schedule(
          monotonic_now() + std::chrono::milliseconds(FLAGS_timing_report_ms),
          std::chrono::milliseconds(FLAGS_timing_report_ms));
    });

    UpdateTimingReport();
  }
}

void EventLoop::ReserveEvents() {
  events_.reserve(timers_.size() + phased_loops_.size() + watchers_.size());
}

namespace {
bool CompareEvents(const EventLoopEvent *first, const EventLoopEvent *second) {
  if (first->event_time() > second->event_time()) {
    return true;
  }
  if (first->event_time() < second->event_time()) {
    return false;
  }
  return first->generation() > second->generation();
}
}  // namespace

void EventLoop::AddEvent(EventLoopEvent *event) {
  DCHECK(std::find(events_.begin(), events_.end(), event) == events_.end());
  DCHECK(event->generation() == 0);
  event->set_generation(++event_generation_);
  events_.push_back(event);
  std::push_heap(events_.begin(), events_.end(), CompareEvents);
}

void EventLoop::RemoveEvent(EventLoopEvent *event) {
  auto e = std::find(events_.begin(), events_.end(), event);
  if (e != events_.end()) {
    DCHECK(event->generation() != 0);
    events_.erase(e);
    std::make_heap(events_.begin(), events_.end(), CompareEvents);
    event->Invalidate();
  }
}

EventLoopEvent *EventLoop::PopEvent() {
  EventLoopEvent *result = events_.front();
  std::pop_heap(events_.begin(), events_.end(), CompareEvents);
  events_.pop_back();
  result->Invalidate();
  return result;
}

void EventLoop::ClearContext() {
  context_.monotonic_event_time = monotonic_clock::min_time;
  context_.monotonic_remote_time = monotonic_clock::min_time;
  context_.realtime_event_time = realtime_clock::min_time;
  context_.realtime_remote_time = realtime_clock::min_time;
  context_.queue_index = 0xffffffffu;
  context_.remote_queue_index = 0xffffffffu;
  context_.size = 0u;
  context_.data = nullptr;
  context_.buffer_index = -1;
  context_.source_boot_uuid = boot_uuid();
}

void EventLoop::SetTimerContext(
    monotonic_clock::time_point monotonic_event_time) {
  context_.monotonic_event_time = monotonic_event_time;
  context_.monotonic_remote_time = monotonic_clock::min_time;
  context_.realtime_event_time = realtime_clock::min_time;
  context_.realtime_remote_time = realtime_clock::min_time;
  context_.queue_index = 0xffffffffu;
  context_.remote_queue_index = 0xffffffffu;
  context_.size = 0u;
  context_.data = nullptr;
  context_.buffer_index = -1;
  context_.source_boot_uuid = boot_uuid();
}

cpu_set_t EventLoop::DefaultAffinity() { return aos::DefaultAffinity(); }

void EventLoop::SetDefaultVersionString(std::string_view version) {
  default_version_string_ = version;
}

void EventLoop::SetVersionString(std::string_view version) {
  CHECK(!is_running())
      << ": Can't do things that might alter the timing report while running.";
  version_string_ = version;

  UpdateTimingReport();
}

void WatcherState::set_timing_report(timing::Watcher *watcher) {
  watcher_ = watcher;
  if (!watcher) {
    wakeup_latency_.set_statistic(nullptr);
    handler_time_.set_statistic(nullptr);
  } else {
    wakeup_latency_.set_statistic(watcher->mutable_wakeup_latency());
    handler_time_.set_statistic(watcher->mutable_handler_time());
  }
}

void WatcherState::ResetReport() {
  if (!watcher_) {
    return;
  }

  wakeup_latency_.Reset();
  handler_time_.Reset();
  watcher_->mutate_count(0);
}

}  // namespace aos
