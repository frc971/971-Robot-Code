#include "aos/events/timing_report_dump_lib.h"

#include <iomanip>
#include <iostream>

namespace aos {
TimingReportDump::TimingReportDump(aos::EventLoop *event_loop,
                                   AccumulateStatistics accumulate,
                                   StreamResults stream)
    : event_loop_(event_loop), accumulate_(accumulate), stream_(stream) {
  // We watch on the timing report channel, so we can't send timing reports.
  event_loop_->SkipTimingReport();
  event_loop_->MakeWatcher("/aos", [this](const timing::Report &report) {
    HandleTimingReport(report);
  });
}

namespace {
std::ostream &operator<<(std::ostream &os, const timing::Statistic &stats) {
  // Use default width for numbers (in case any previous things set a different
  // width in the output stream).
  const auto num_width = std::setw(0);
  os << std::setfill(' ') << num_width << stats.average() << " [" << num_width
     << stats.min() << ", " << num_width << stats.max() << "] std " << num_width
     << stats.standard_deviation();
  return os;
}

// Generates a table of the specified strings, such that the columns are all
// spaced equally.
// Will use prefix for indentation.
template <size_t kColumns>
void PrintTable(std::ostream *os, std::string_view prefix,
                const std::vector<std::array<std::string, kColumns>> &table) {
  std::array<size_t, kColumns> widths;
  widths.fill(0);
  for (const auto &row : table) {
    for (size_t ii = 0; ii < kColumns; ++ii) {
      widths.at(ii) = std::max(widths.at(ii), row.at(ii).size());
    }
  }
  const std::string kSep = " | ";
  for (const auto &row : table) {
    *os << prefix << std::setfill(' ');
    for (size_t ii = 0; ii < widths.size(); ++ii) {
      *os << std::setw(widths.at(ii)) << row.at(ii);
      if (ii + 1 != widths.size()) {
        *os << " | ";
      }
    }
    *os << std::endl;
  }
}

// Spacing to use for indentation.
const std::string kIndent = "  ";

std::string MaybeNodeName(std::string_view prefix_if_node,
                               const aos::Node *node) {
  if (node == nullptr) {
    return "";
  }
  return absl::StrCat(prefix_if_node, node->name()->string_view());
}
}  // namespace

void TimingReportDump::PrintTimers(
    std::ostream *os, std::string_view name,
    const flatbuffers::Vector<flatbuffers::Offset<timing::Timer>> &timers) {
  *os << kIndent << name << " (" << timers.size() << "):" << std::endl;
  std::vector<std::array<std::string, 4>> rows;
  rows.push_back({"Name", "Count", "Wakeup Latency", "Handler Time"});
  for (const timing::Timer *timer : timers) {
    std::stringstream wakeup_latency_stats;
    CHECK(timer->has_wakeup_latency());
    wakeup_latency_stats << *timer->wakeup_latency();
    std::stringstream handler_time_stats;
    CHECK(timer->has_handler_time());
    handler_time_stats << *timer->handler_time();
    rows.push_back({timer->has_name() ? timer->name()->str() : "",
                    std::to_string(timer->count()), wakeup_latency_stats.str(),
                    handler_time_stats.str()});
  }
  PrintTable(os, kIndent + kIndent, rows);
}

void TimingReportDump::PrintWatchers(
    std::ostream *os,
    const flatbuffers::Vector<flatbuffers::Offset<timing::Watcher>> &watchers) {
  *os << kIndent << "Watchers (" << watchers.size() << "):" << std::endl;
  std::vector<std::array<std::string, 5>> rows;
  rows.push_back(
      {"Channel Name", "Type", "Count", "Wakeup Latency", "Handler Time"});
  for (const timing::Watcher *watcher : watchers) {
    const Channel *channel = GetChannel(watcher->channel_index());
    std::stringstream latency_stats;
    std::stringstream handler_stats;
    CHECK(watcher->has_wakeup_latency());
    CHECK(watcher->has_handler_time());
    latency_stats << *watcher->wakeup_latency();
    handler_stats << *watcher->handler_time();
    rows.push_back({channel->name()->str(), channel->type()->str(),
                    std::to_string(watcher->count()), latency_stats.str(),
                    handler_stats.str()});
  }
  PrintTable(os, kIndent + kIndent, rows);
}

void TimingReportDump::PrintSenders(
    std::ostream *os,
    const flatbuffers::Vector<flatbuffers::Offset<timing::Sender>> &senders) {
  *os << kIndent << "Senders (" << senders.size() << "):" << std::endl;
  std::vector<std::array<std::string, 5>> rows;
  rows.push_back({"Channel Name", "Type", "Count", "Size", "Errors"});
  for (const timing::Sender *sender : senders) {
    const Channel *channel = GetChannel(sender->channel_index());
    std::stringstream size_stats;
    CHECK(sender->has_size());
    size_stats << *sender->size();
    std::stringstream errors;
    CHECK(sender->has_error_counts());
    for (size_t ii = 0; ii < sender->error_counts()->size(); ++ii) {
      const size_t error_count =
          CHECK_NOTNULL(sender->error_counts()->Get(ii))->count();
      errors << error_count;
      if (error_count > 0) {
        // Put send errors onto stderr so that people just interested in
        // outright errors can find them more readily.
        LOG(INFO) << configuration::StrippedChannelToString(channel) << ": "
                  << error_count << " "
                  << timing::EnumNamesSendError()[static_cast<uint8_t>(
                         sender->error_counts()->Get(ii)->error())]
                  << " errors.";
      }
      if (ii + 1 != sender->error_counts()->size()) {
        errors << ", ";
      }
    }
    rows.push_back({channel->name()->str(), channel->type()->str(),
                    std::to_string(sender->count()), size_stats.str(),
                    errors.str()});
  }
  PrintTable(os, kIndent + kIndent, rows);
}

void TimingReportDump::PrintFetchers(
    std::ostream *os,
    const flatbuffers::Vector<flatbuffers::Offset<timing::Fetcher>> &fetchers) {
  *os << kIndent << "Fetchers (" << fetchers.size() << "):" << std::endl;
  std::vector<std::array<std::string, 4>> rows;
  rows.push_back({"Channel Name", "Type", "Count", "Latency"});
  for (const timing::Fetcher *fetcher : fetchers) {
    const Channel *channel = GetChannel(fetcher->channel_index());
    std::stringstream latency_stats;
    CHECK(fetcher->has_latency());
    latency_stats << *fetcher->latency();
    rows.push_back({channel->name()->str(), channel->type()->str(),
                    std::to_string(fetcher->count()), latency_stats.str()});
  }
  PrintTable(os, kIndent + kIndent, rows);
}

void TimingReportDump::HandleTimingReport(const timing::Report &report) {
  CHECK(report.has_name());
  if (name_filter_.has_value() &&
      name_filter_.value() != report.name()->string_view()) {
    return;
  }
  if (stream_ == StreamResults::kYes) {
    PrintReport(report);
  }

  if (accumulate_ == AccumulateStatistics::kYes) {
    AccumulateReport(report);
  }
}

void TimingReportDump::PrintReport(const timing::Report &report) {
  VLOG(1) << FlatbufferToJson(&report);
  if (report.send_failures() != 0) {
    LOG(INFO) << "Failed to send " << report.send_failures()
              << " timing report(s) in " << report.name()->string_view();
  }
  std::cout << report.name()->string_view() << "[" << report.pid() << "] ("
            << MaybeNodeName("", event_loop_->node()) << ") ("
            << event_loop_->context().monotonic_event_time << ","
            << event_loop_->context().realtime_event_time << "):" << std::endl;
  if (report.has_watchers() && report.watchers()->size() > 0) {
    PrintWatchers(&std::cout, *report.watchers());
  }
  if (report.has_senders() && report.senders()->size() > 0) {
    PrintSenders(&std::cout, *report.senders());
  }
  if (report.has_fetchers() && report.fetchers()->size() > 0) {
    PrintFetchers(&std::cout, *report.fetchers());
  }
  if (report.has_timers() && report.timers()->size() > 0) {
    PrintTimers(&std::cout, "Timers", *report.timers());
  }
  if (report.has_phased_loops() && report.phased_loops()->size() > 0) {
    PrintTimers(&std::cout, "Phased Loops", *report.phased_loops());
  }
}

TimingReportDump::~TimingReportDump() {
  if (accumulate_ == AccumulateStatistics::kYes) {
    if (accumulated_statistics_.size() > 0) {
      std::cout << "\nAccumulated timing reports "
                << MaybeNodeName(" for node ", event_loop_->node()) << ":\n\n";
    }
    for (const auto &pair : accumulated_statistics_) {
      flatbuffers::FlatBufferBuilder fbb;
      fbb.Finish(timing::Report::Pack(fbb, &pair.second));
      FlatbufferDetachedBuffer<timing::Report> report_buffer(fbb.Release());
      const timing::Report &report = report_buffer.message();
      if (name_filter_.has_value() &&
          name_filter_.value() != report.name()->string_view()) {
        return;
      }
      PrintReport(report);
    }
  }
}

namespace {
// Helper function to combine the aggregate statistics from two entries. Most of
// the complexity is in combining the standard deviations.
void CombineStatistics(const size_t addition_count,
                       const timing::StatisticT &addition,
                       const size_t accumulator_count,
                       timing::StatisticT *accumulator) {
  if (addition_count == 0) {
    return;
  }
  // Separate isnan handler to special-case the timing reports (see comment
  // below).
  if (accumulator_count == 0 || std::isnan(accumulator->average)) {
    *accumulator = addition;
    return;
  }
  const double old_average = accumulator->average;
  accumulator->average = (accumulator->average * accumulator_count +
                          addition.average * addition_count) /
                         (accumulator_count + addition_count);
  accumulator->max = std::max(accumulator->max, addition.max);
  accumulator->min = std::min(accumulator->min, addition.min);
  // Borrowing the process from
  // https://math.stackexchange.com/questions/2971315/how-do-i-combine-standard-deviations-of-two-groups
  // which gives:
  //
  // std_x^2 = sum((x_i - avg(x))^2) / (N - 1)
  //
  // If combining two distributions, x and y, with N and M elements
  // respectively, into a combined distribution z, we will have:
  //
  // std_z^2 = (sum((x_i - avg(z))^2) + sum((y_i - avg(z)^2)) / (N + M - 1)
  // (x_i - avg(z)) = ((x_i - avg(x)) + (avg(x) - avg(z))) =
  //   (x_i - avg(x))^2 + 2 * (x_i - avg(x)) * (avg(x) - avg(z)) + (avg(x) -
  //   avg(z))^2
  // Note that when we do the sum, there is a sum(x - avg(x)) term that we just
  // zero out.
  // sum((x_i - avg(z))^2) = (N - 1) * std_x^2 + N * (avg(x) - avg(z))^2
  // std_z^2 = ((N - 1) * std_x^2 + N * (avg(x) - avg(z))^2 + (M - 1) * std_y^2
  //           + M * (avg(y) - avg(z)^2)) / (N + M - 1)
  const double N = addition_count;
  const double M = accumulator_count;
  const double var_x = std::pow(addition.standard_deviation, 2);
  const double var_y = std::pow(accumulator->standard_deviation, 2);
  const double avg_x = addition.average;
  const double avg_y = old_average;
  const double new_variance =
      ((N - 1) * var_x + N * std::pow(avg_x - accumulator->average, 2) +
       (M - 1) * var_y + M * std::pow(avg_y - accumulator->average, 2)) /
      (N + M - 1);
  accumulator->standard_deviation = std::sqrt(new_variance);
}

void CombineTimers(
    const std::vector<std::unique_ptr<timing::TimerT>> &new_timers,
    std::vector<std::unique_ptr<timing::TimerT>> *aggregate_timers) {
  for (auto &timer : new_timers) {
    auto timer_iter =
        std::find_if(aggregate_timers->begin(), aggregate_timers->end(),
                     [&timer](const std::unique_ptr<timing::TimerT> &val) {
                       // For many/most timers, the name will be empty, so we
                       // just aggregate all of the empty ones together.
                       return val->name == timer->name;
                     });
    if (timer_iter == aggregate_timers->end()) {
      aggregate_timers->emplace_back(new timing::TimerT());
      *aggregate_timers->back() = *timer;
    } else {
      CombineStatistics(timer->count, *timer->wakeup_latency,
                        (*timer_iter)->count,
                        (*timer_iter)->wakeup_latency.get());
      // TODO(james): This isn't actually correct *for the timing report timer
      // itself*. On the very first timing report that a process sends out, it
      // will have count = 1, wakeup_latency = <something real>, handler_time =
      // nan, because we are still handling the timer.
      CombineStatistics(timer->count, *timer->handler_time,
                        (*timer_iter)->count,
                        (*timer_iter)->handler_time.get());
      (*timer_iter)->count += timer->count;
    }
  }
}
}  // namespace

void TimingReportDump::AccumulateReport(const timing::Report &raw_report) {
  CHECK(raw_report.has_pid());
  CHECK(raw_report.has_name());
  const std::pair<pid_t, std::string> map_key(raw_report.pid(),
                                              raw_report.name()->str());
  if (accumulated_statistics_.count(map_key) == 0) {
    accumulated_statistics_[map_key].name = raw_report.name()->str();
    accumulated_statistics_[map_key].pid = raw_report.pid();
  }

  timing::ReportT report;
  raw_report.UnPackTo(&report);
  timing::ReportT *summary = &accumulated_statistics_[map_key];
  for (auto &watcher : report.watchers) {
    auto watcher_iter =
        std::find_if(summary->watchers.begin(), summary->watchers.end(),
                     [&watcher](const std::unique_ptr<timing::WatcherT> &val) {
                       return val->channel_index == watcher->channel_index;
                     });
    if (watcher_iter == summary->watchers.end()) {
      summary->watchers.push_back(std::move(watcher));
    } else {
      CombineStatistics(watcher->count, *watcher->wakeup_latency,
                        (*watcher_iter)->count,
                        (*watcher_iter)->wakeup_latency.get());
      CombineStatistics(watcher->count, *watcher->handler_time,
                        (*watcher_iter)->count,
                        (*watcher_iter)->handler_time.get());
      (*watcher_iter)->count += watcher->count;
    }
  }
  for (auto &sender : report.senders) {
    auto sender_iter =
        std::find_if(summary->senders.begin(), summary->senders.end(),
                     [&sender](const std::unique_ptr<timing::SenderT> &val) {
                       return val->channel_index == sender->channel_index;
                     });
    if (sender_iter == summary->senders.end()) {
      summary->senders.push_back(std::move(sender));
    } else {
      CombineStatistics(sender->count, *sender->size, (*sender_iter)->count,
                        (*sender_iter)->size.get());
      (*sender_iter)->count += sender->count;
      CHECK_EQ((*sender_iter)->error_counts.size(),
               sender->error_counts.size());
      for (size_t ii = 0; ii < sender->error_counts.size(); ++ii) {
        (*sender_iter)->error_counts[ii]->count +=
            sender->error_counts[ii]->count;
      }
    }
  }
  for (auto &fetcher : report.fetchers) {
    auto fetcher_iter =
        std::find_if(summary->fetchers.begin(), summary->fetchers.end(),
                     [&fetcher](const std::unique_ptr<timing::FetcherT> &val) {
                       return val->channel_index == fetcher->channel_index;
                     });
    if (fetcher_iter == summary->fetchers.end()) {
      summary->fetchers.push_back(std::move(fetcher));
    } else {
      CombineStatistics(fetcher->count, *fetcher->latency,
                        (*fetcher_iter)->count, (*fetcher_iter)->latency.get());
      (*fetcher_iter)->count += fetcher->count;
    }
  }
  CombineTimers(report.timers, &summary->timers);
  CombineTimers(report.phased_loops, &summary->phased_loops);
  summary->send_failures += report.send_failures;
}

const Channel *TimingReportDump::GetChannel(int index) {
  CHECK_LE(0, index);
  CHECK_GT(event_loop_->configuration()->channels()->size(),
           static_cast<size_t>(index));
  return event_loop_->configuration()->channels()->Get(index);
}

}  // namespace aos
