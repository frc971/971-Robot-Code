#include "aos/events/event_loop.h"

#include "aos/configuration.h"
#include "aos/configuration_generated.h"
#include "glog/logging.h"

DEFINE_bool(timing_reports, true, "Publish timing reports.");
DEFINE_int32(timing_report_ms, 1000,
             "Period in milliseconds to publish timing reports at.");

namespace aos {

RawSender::RawSender(EventLoop *event_loop, const Channel *channel)
    : event_loop_(event_loop),
      channel_(channel),
      timing_(event_loop_->ChannelIndex(channel)) {
  event_loop_->NewSender(this);
}

RawSender::~RawSender() { event_loop_->DeleteSender(this); }

RawFetcher::RawFetcher(EventLoop *event_loop, const Channel *channel)
    : event_loop_(event_loop),
      channel_(channel),
      timing_(event_loop_->ChannelIndex(channel)) {
  context_.monotonic_event_time = monotonic_clock::min_time;
  context_.monotonic_remote_time = monotonic_clock::min_time;
  context_.realtime_event_time = realtime_clock::min_time;
  context_.realtime_remote_time = realtime_clock::min_time;
  context_.queue_index = 0xffffffff;
  context_.size = 0;
  context_.data = nullptr;
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
    Reschedule(
        [this](monotonic_clock::time_point sleep_time) {
          Schedule(sleep_time);
        },
        monotonic_now);
    // The first time, we'll double count.  Reschedule here will count cycles
    // elapsed before now, and then the reschedule before runing the handler
    // will count the time that elapsed then.  So clear the count here.
    cycles_elapsed_ = 0;
  });
}

PhasedLoopHandler::~PhasedLoopHandler() {}

EventLoop::~EventLoop() {
  CHECK_EQ(senders_.size(), 0u) << ": Not all senders destroyed";
  CHECK_EQ(events_.size(), 0u) << ": Not all events unregistered";
}

int EventLoop::ChannelIndex(const Channel *channel) {
  CHECK(configuration_->channels() != nullptr) << ": No channels";

  auto c = std::find(configuration_->channels()->begin(),
                     configuration_->channels()->end(), channel);
  CHECK(c != configuration_->channels()->end())
      << ": Channel pointer not found in configuration()->channels()";

  return std::distance(configuration()->channels()->begin(), c);
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

void EventLoop::SendTimingReport() {
  // We need to do a fancy dance here to get all the accounting to work right.
  // We want to copy the memory here, but then send after resetting. Otherwise
  // the send for the timing report won't be counted in the timing report.
  //
  // Also, flatbuffers build from the back end.  So place this at the back end
  // of the buffer.  We only have to care because we are using this in a very
  // raw fashion.
  CHECK_LE(timing_report_.size(), timing_report_sender_->size())
      << ": Timing report bigger than the sender size.";
  std::copy(timing_report_.data(),
            timing_report_.data() + timing_report_.size(),
            reinterpret_cast<uint8_t *>(timing_report_sender_->data()) +
                timing_report_sender_->size() - timing_report_.size());

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
  timing_report_sender_->Send(timing_report_.size());
}

void EventLoop::UpdateTimingReport() {
  // We need to support senders and fetchers changing while we are setting up
  // the event loop.  Otherwise we can't fetch or send until the loop runs. This
  // means that on each change, we need to redo all this work.  This makes setup
  // more expensive, but not by all that much on a modern processor.

  // Now, build up a report with everything pre-filled out.
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(1);

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
  for (const RawSender *sender : senders_) {
    flatbuffers::Offset<timing::Statistic> size_offset =
        timing::CreateStatistic(fbb);

    timing::Sender::Builder sender_builder(fbb);

    sender_builder.add_channel_index(sender->timing_.channel_index);
    sender_builder.add_size(size_offset);
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

  timing::Report::Builder report_builder(fbb);
  report_builder.add_name(name_offset);
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
      timing_reports_timer->Setup(
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
  return first->event_time() > second->event_time();
}
}  // namespace

void EventLoop::AddEvent(EventLoopEvent *event) {
  DCHECK(std::find(events_.begin(), events_.end(), event) == events_.end());
  events_.push_back(event);
  std::push_heap(events_.begin(), events_.end(), CompareEvents);
}

void EventLoop::RemoveEvent(EventLoopEvent *event) {
  auto e = std::find(events_.begin(), events_.end(), event);
  if (e != events_.end()) {
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

void WatcherState::set_timing_report(timing::Watcher *watcher) {
  CHECK_NOTNULL(watcher);
  watcher_ = watcher;
  wakeup_latency_.set_statistic(watcher->mutable_wakeup_latency());
  handler_time_.set_statistic(watcher->mutable_handler_time());
}

void WatcherState::ResetReport() {
  wakeup_latency_.Reset();
  handler_time_.Reset();
  watcher_->mutate_count(0);
}

}  // namespace aos
