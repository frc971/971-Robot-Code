#include <stdlib.h>

#include <iostream>
#include <optional>
#include <ostream>
#include <sstream>
#include <string_view>
#include <vector>

#include "aos/configuration_generated.h"
#include "aos/events/event_loop.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/log_reader_utils.h"
#include "aos/events/logging/log_replayer_config_generated.h"
#include "aos/events/logging/logfile_sorting.h"
#include "aos/events/logging/logfile_utils.h"
#include "aos/events/logging/replay_timing_generated.h"
#include "aos/events/logging/replay_timing_schema.h"
#include "aos/events/shm_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/util/file.h"
#include "flatbuffers/flatbuffers.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(config, "", "If specified, overrides logged configuration.");
DEFINE_bool(
    plot_timing, true,
    "If set, generates a plot of the replay timing--namely, the errors between "
    "when we "
    "should've sent messages and when we actually sent replayed messages.");
DEFINE_bool(skip_sender_channels, true,
            "If set, skips replay of the channels applications replay on");
DEFINE_bool(skip_replay, false,
            "If set, skips actually running the replay. Useful for writing a "
            "config without running replay");
DEFINE_bool(
    print_config, false,
    "If set, prints the config that will be used for replay to stdout as json");
DEFINE_string(
    replay_config, "",
    "Path to the configuration used for log replay which includes items such "
    "as channels to remap, and applications to target for replay. If not set, "
    "log_reader will run on shm event loop. ");
DEFINE_string(merge_with_config, "",
              "A valid json string to be merged with config. This is used to "
              "add extra applications needed to run only for log_replayer");

namespace aos::logger {

int Main(int argc, char *argv[]) {
  const std::vector<std::string> unsorted_logfiles =
      aos::logger::FindLogs(argc, argv);

  const std::vector<aos::logger::LogFile> logfiles =
      aos::logger::SortParts(unsorted_logfiles);

  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      FLAGS_config.empty()
          ? aos::FlatbufferDetachedBuffer<aos::Configuration>::Empty()
          : aos::configuration::ReadConfig(FLAGS_config);

  if (FLAGS_plot_timing) {
    aos::logger::LogReader config_reader(logfiles);

    // Go through the effort to add a ReplayTiming channel to ensure that we
    // can capture timing information from the replay.
    const aos::Configuration *raw_config = FLAGS_config.empty()
                                               ? config_reader.configuration()
                                               : &config.message();
    aos::ChannelT channel_overrides;
    channel_overrides.max_size = 10000;
    channel_overrides.frequency = 10000;
    config = aos::configuration::AddChannelToConfiguration(
        raw_config, "/timing",
        aos::FlatbufferSpan<reflection::Schema>(
            aos::timing::ReplayTimingSchema()),
        aos::configuration::GetMyNode(raw_config), channel_overrides);
  }

  if (!FLAGS_merge_with_config.empty()) {
    config = aos::configuration::MergeWithConfig(&config.message(),
                                                 FLAGS_merge_with_config);
  }

  std::optional<aos::FlatbufferDetachedBuffer<ReplayConfig>> replay_config =
      FLAGS_replay_config.empty()
          ? std::nullopt
          : std::make_optional(aos::JsonToFlatbuffer<ReplayConfig>(
                aos::util::ReadFileToStringOrDie(FLAGS_replay_config.data())));

  std::vector<std::pair<std::string_view, std::string_view>> message_filter;
  if (FLAGS_skip_sender_channels && replay_config.has_value()) {
    CHECK(replay_config.value().message().has_active_nodes());
    std::vector<const Node *> active_nodes;
    for (const auto &node : *replay_config.value().message().active_nodes()) {
      active_nodes.emplace_back(configuration::GetNode(
          &config.message(), node->name()->string_view()));
    }

    std::vector<std::string> applications;
    for (const auto &application :
         *replay_config.value().message().applications()) {
      if (application->name()->string_view() != "camera_message_interceptor") {
        applications.emplace_back(application->name()->string_view());
      }
    }

    aos::logger::ChannelsInLogResult channels =
        ChannelsInLog(logfiles, active_nodes, applications);
    for (auto const &channel :
         channels.watchers_and_fetchers_without_senders.value()) {
      message_filter.emplace_back(std::make_pair(channel.name, channel.type));
    }
  }

  aos::logger::LogReader reader(
      logfiles, &config.message(),
      message_filter.empty() ? nullptr : &message_filter);

  if (replay_config.has_value()) {
    for (auto const &remap_channel :
         *replay_config.value().message().remap_channels()) {
      auto const &channel = remap_channel->channel();
      std::string_view new_type = remap_channel->has_new_type()
                                      ? remap_channel->new_type()->string_view()
                                      : channel->type()->string_view();
      reader.RemapLoggedChannel(
          channel->name()->string_view(), channel->type()->string_view(),
          remap_channel->prefix()->string_view(), new_type);
    }
  }

  if (FLAGS_print_config) {
    // TODO(Naman): Replace with config writer if it will be cleaner
    std::cout << FlatbufferToJson(reader.configuration()) << std::endl;
  }

  if (!FLAGS_skip_replay) {
    aos::ShmEventLoop event_loop(reader.configuration());

    event_loop.SkipAosLog();
    event_loop.SkipTimingReport();

    reader.Register(&event_loop);
    reader.OnEnd(event_loop.node(), [&event_loop]() { event_loop.Exit(); });

    if (FLAGS_plot_timing) {
      aos::Sender<aos::timing::ReplayTiming> replay_timing_sender =
          event_loop.MakeSender<aos::timing::ReplayTiming>("/timing");
      reader.set_timing_accuracy_sender(event_loop.node(),
                                        std::move(replay_timing_sender));
    }

    event_loop.Run();

    reader.Deregister();
  }

  return EXIT_SUCCESS;
}

}  // namespace aos::logger

int main(int argc, char *argv[]) {
  gflags::SetUsageMessage(
      R"message(Binary to replay the full contents of a logfile into shared memory.
                #replay_config should be set in order to replay a set of nodes, applications and channels
                #print config and skip replay, if you only want to print the config and not do log replay
                Use case #1: log_replayer <log_dir> --print_config --replay_config=<path_to_config> --skip_replay
                Use case #2: log_replayer <log_dir> --nofatal_sent_too_fast --replay_config=<path_to_config>
      )message");

  aos::InitGoogle(&argc, &argv);
  return aos::logger::Main(argc, argv);
}
