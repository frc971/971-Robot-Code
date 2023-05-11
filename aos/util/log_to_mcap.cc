#include "aos/configuration.h"
#include "aos/events/event_loop_generated.h"
#include "aos/events/logging/log_reader.h"
#include "aos/init.h"
#include "aos/util/clock_publisher.h"
#include "aos/util/clock_timepoints_schema.h"
#include "aos/util/mcap_logger.h"

DEFINE_string(node, "", "Node to replay from the perspective of.");
DEFINE_string(output_path, "/tmp/log.mcap", "Log to output.");
DEFINE_string(mode, "flatbuffer", "json or flatbuffer serialization.");
DEFINE_bool(
    canonical_channel_names, false,
    "If set, use full channel names; by default, will shorten names to be the "
    "shortest possible version of the name (e.g., /aos instead of /pi/aos).");
DEFINE_bool(compress, true, "Whether to use LZ4 compression in MCAP file.");
DEFINE_bool(include_clocks, true,
            "Whether to add a /clocks channel that publishes all nodes' clock "
            "offsets.");

// Converts an AOS log to an MCAP log that can be fed into Foxglove. To try this
// out, run:
// bazel run -c opt //aos/util:log_to_mcap -- --node NODE_NAME /path/to/logfile
//
// Then navigate to http://studio.foxglove.dev (or spin up your own instance
// locally), and use it to open the file (this doesn't upload the file to
// foxglove's servers or anything).
int main(int argc, char *argv[]) {
  aos::InitGoogle(&argc, &argv);

  const std::vector<aos::logger::LogFile> logfiles =
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv));
  CHECK(!logfiles.empty());
  const std::string logger_node = logfiles.at(0).logger_node;
  bool all_logs_from_same_node = true;
  for (const aos::logger::LogFile &log : logfiles) {
    if (log.logger_node != logger_node) {
      all_logs_from_same_node = false;
      break;
    }
  }
  std::string replay_node = FLAGS_node;
  if (replay_node.empty() && all_logs_from_same_node) {
    LOG(INFO) << "Guessing \"" << logger_node
              << "\" as node given that --node was not specified.";
    replay_node = logger_node;
  }

  std::optional<aos::FlatbufferDetachedBuffer<aos::Configuration>> config;

  if (FLAGS_include_clocks) {
    aos::logger::LogReader config_reader(logfiles);

    const aos::Configuration *raw_config = config_reader.logged_configuration();
    config = aos::configuration::AddChannelToConfiguration(
        raw_config, "/clocks",
        aos::FlatbufferSpan<reflection::Schema>(aos::ClockTimepointsSchema()),
        replay_node.empty()
            ? nullptr
            : aos::configuration::GetNode(raw_config, replay_node));
  }

  aos::logger::LogReader reader(
      logfiles, config.has_value() ? &config.value().message() : nullptr);
  aos::SimulatedEventLoopFactory factory(reader.configuration());
  reader.RegisterWithoutStarting(&factory);

  const aos::Node *node =
      (replay_node.empty() ||
       !aos::configuration::MultiNode(reader.configuration()))
          ? nullptr
          : aos::configuration::GetNode(reader.configuration(), replay_node);

  std::unique_ptr<aos::EventLoop> clock_event_loop;
  std::unique_ptr<aos::ClockPublisher> clock_publisher;
  if (FLAGS_include_clocks) {
    reader.OnStart(
        node, [&clock_event_loop, &reader, &clock_publisher, &factory, node]() {
          clock_event_loop =
              reader.event_loop_factory()->MakeEventLoop("clock", node);
          clock_publisher = std::make_unique<aos::ClockPublisher>(
              &factory, clock_event_loop.get());
        });
  }

  std::unique_ptr<aos::EventLoop> mcap_event_loop;
  CHECK(!FLAGS_output_path.empty());
  std::unique_ptr<aos::McapLogger> relogger;
  factory.GetNodeEventLoopFactory(node)->OnStartup([&relogger, &mcap_event_loop,
                                                    &reader, node]() {
    mcap_event_loop = reader.event_loop_factory()->MakeEventLoop("mcap", node);
    relogger = std::make_unique<aos::McapLogger>(
        mcap_event_loop.get(), FLAGS_output_path,
        FLAGS_mode == "flatbuffer" ? aos::McapLogger::Serialization::kFlatbuffer
                                   : aos::McapLogger::Serialization::kJson,
        FLAGS_canonical_channel_names
            ? aos::McapLogger::CanonicalChannelNames::kCanonical
            : aos::McapLogger::CanonicalChannelNames::kShortened,
        FLAGS_compress ? aos::McapLogger::Compression::kLz4
                       : aos::McapLogger::Compression::kNone);
  });
  reader.event_loop_factory()->Run();
  reader.Deregister();
}
