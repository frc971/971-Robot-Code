#include <unistd.h>

#include <iomanip>
#include <iostream>  // IWYU pragma: keep

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"

#include "aos/aos_cli_utils.h"
#include "aos/configuration.h"
#include "aos/ftrace.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/realtime.h"

ABSL_FLAG(int32_t, priority, -1, "If set, the RT priority to run at.");
ABSL_FLAG(double, max_jitter, 0.01,
          "The max time in milliseconds between messages before marking it "
          "as too late.");
ABSL_FLAG(bool, print_jitter, true,
          "If true, print jitter events.  These will impact RT performance.");
ABSL_DECLARE_FLAG(bool, enable_ftrace);
ABSL_FLAG(bool, print_latency_stats, false,
          "If true, print latency stats.  These will impact RT performance.");

namespace aos {

class State {
 public:
  State(Ftrace *ftrace, aos::EventLoop *event_loop, const Channel *channel)
      : ftrace_(ftrace),
        channel_(channel),
        channel_name_(aos::configuration::StrippedChannelToString(channel_)) {
    LOG(INFO) << "Watching for jitter on " << channel_name_;

    event_loop->MakeRawWatcher(
        channel_, [this](const aos::Context &context, const void *message) {
          HandleMessage(context, message);
        });

    if (absl::GetFlag(FLAGS_print_latency_stats)) {
      timer_handle_ = event_loop->AddTimer([this]() { PrintLatencyStats(); });
      timer_handle_->set_name("jitter");
      event_loop->OnRun([this, event_loop]() {
        timer_handle_->Schedule(event_loop->monotonic_now(),
                                std::chrono::milliseconds(1000));
      });
    }
  }

  void HandleMessage(const aos::Context &context, const void * /*message*/) {
    if (last_time_ != aos::monotonic_clock::min_time) {
      latency_.push_back(std::chrono::duration<double>(
                             context.monotonic_event_time - last_time_)
                             .count());
      if (context.monotonic_event_time >
          last_time_ + std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::duration<double>(
                               absl::GetFlag(FLAGS_max_jitter)))) {
        if (absl::GetFlag(FLAGS_enable_ftrace)) {
          ftrace_->FormatMessage(
              "Got high latency event on %s -> %.9f between messages",
              channel_name_.c_str(),
              std::chrono::duration<double>(context.monotonic_event_time -
                                            last_time_)
                  .count());
          ftrace_->TurnOffOrDie();
        }

        if (absl::GetFlag(FLAGS_print_jitter)) {
          // Printing isn't realtime, but if someone wants to run as RT, they
          // should know this.  Bypass the warning.
          ScopedNotRealtime nrt;

          LOG(INFO) << "Got a high latency event on "
                    << aos::configuration::StrippedChannelToString(channel_)
                    << " -> " << std::fixed << std::setprecision(9)
                    << std::chrono::duration<double>(
                           context.monotonic_event_time - last_time_)
                           .count()
                    << " between messages.";
        }
      }
    }

    last_time_ = context.monotonic_event_time;
  }

  void PrintLatencyStats() {
    std::sort(latency_.begin(), latency_.end());
    if (latency_.size() >= 100) {
      LOG(INFO) << "Percentiles 25th: " << latency_[latency_.size() * 0.25]
                << " 50th: " << latency_[latency_.size() * 0.5]
                << " 75th: " << latency_[latency_.size() * 0.75]
                << " 90th: " << latency_[latency_.size() * 0.9]
                << " 95th: " << latency_[latency_.size() * 0.95]
                << " 99th: " << latency_[latency_.size() * 0.99];
      LOG(INFO) << "Max: " << latency_.back() << " Min: " << latency_.front()
                << " Mean: "
                << std::accumulate(latency_.begin(), latency_.end(), 0.0) /
                       latency_.size();
    }
  }

 private:
  Ftrace *ftrace_;
  const Channel *channel_;

  std::string channel_name_;

  aos::monotonic_clock::time_point last_time_ = aos::monotonic_clock::min_time;

  std::vector<double> latency_;

  aos::TimerHandler *timer_handle_;
};

}  // namespace aos

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  aos::CliUtilInfo cli_info;
  if (cli_info.Initialize(
          &argc, &argv,
          [&cli_info](const aos::Channel *channel) {
            return aos::configuration::ChannelIsReadableOnNode(
                channel, cli_info.event_loop->node());
          },
          "channel is readeable on node", true)) {
    return 0;
  }

  aos::Ftrace ftrace;

  std::vector<std::unique_ptr<aos::State>> states;

  for (const aos::Channel *channel : cli_info.found_channels) {
    states.emplace_back(std::make_unique<aos::State>(
        &ftrace, &(cli_info.event_loop.value()), channel));
  }

  if (absl::GetFlag(FLAGS_priority) > 0) {
    cli_info.event_loop->SetRuntimeRealtimePriority(
        absl::GetFlag(FLAGS_priority));
  }

  cli_info.event_loop->Run();

  return 0;
}
