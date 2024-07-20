#include <unistd.h>

#include <iostream>

#include "absl/flags/flag.h"

#include "aos/aos_cli_utils.h"
#include "aos/configuration.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/realtime.h"

ABSL_FLAG(int32_t, priority, -1, "If set, the RT priority to run at.");
ABSL_FLAG(double, max_jitter, 10.00,
          "The max time in seconds between messages before considering the "
          "camera processes dead.");
ABSL_FLAG(double, grace_period, 10.00,
          "The grace period at startup before enforcing that messages must "
          "flow from the camera processes.");

namespace aos {

class State {
 public:
  State(aos::EventLoop *event_loop, const Channel *channel)
      : channel_(channel),
        channel_name_(aos::configuration::StrippedChannelToString(channel_)) {
    LOG(INFO) << "Watching for healthy message sends on " << channel_name_;

    event_loop->MakeRawNoArgWatcher(
        channel_,
        [this](const aos::Context &context) { HandleMessage(context); });

    timer_handle_ = event_loop->AddTimer(
        [this, event_loop]() { RunHealthCheck(event_loop); });
    timer_handle_->set_name("jitter");
    event_loop->OnRun([this, event_loop]() {
      timer_handle_->Schedule(
          event_loop->monotonic_now() +
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::duration<double>(
                      absl::GetFlag(FLAGS_grace_period))),
          std::chrono::milliseconds(1000));
    });
  }

  void HandleMessage(const aos::Context &context) {
    if (last_time_ == aos::monotonic_clock::min_time) {
      LOG(INFO) << "First message on " << channel_name_;
    }
    last_time_ = context.monotonic_event_time;
  }

  void RunHealthCheck(aos::EventLoop *event_loop) {
    if (last_time_ + std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::duration<double>(
                             absl::GetFlag(FLAGS_max_jitter))) <
        event_loop->monotonic_now()) {
      // Restart camera services
      LOG(INFO) << "Restarting camera services";
      LOG(INFO) << "Channel " << channel_name_ << " has not received a message "
                << absl::GetFlag(FLAGS_max_jitter) << " seconds";
      CHECK_EQ(std::system("aos_starter stop argus_camera0"), 0);
      CHECK_EQ(std::system("aos_starter stop argus_camera1"), 0);
      CHECK_EQ(std::system("sudo systemctl restart nvargus-daemon.service"), 0);
      CHECK_EQ(std::system("aos_starter start argus_camera0"), 0);
      CHECK_EQ(std::system("aos_starter start argus_camera1"), 0);

      std::exit(0);
      return;
    }
  }

 private:
  const Channel *channel_;

  std::string channel_name_;

  aos::monotonic_clock::time_point last_time_ = aos::monotonic_clock::min_time;

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

  std::vector<std::unique_ptr<aos::State>> states;

  for (const aos::Channel *channel : cli_info.found_channels) {
    states.emplace_back(
        std::make_unique<aos::State>(&(cli_info.event_loop.value()), channel));
  }

  if (absl::GetFlag(FLAGS_priority) > 0) {
    cli_info.event_loop->SetRuntimeRealtimePriority(
        absl::GetFlag(FLAGS_priority));
  }

  cli_info.event_loop->Run();

  return 0;
}
