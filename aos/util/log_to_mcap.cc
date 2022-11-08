#include "aos/events/event_loop_generated.h"
#include "aos/events/logging/log_reader.h"
#include "aos/init.h"
#include "aos/util/mcap_logger.h"

DEFINE_string(node, "", "Node to replay from the perspective of.");
DEFINE_string(output_path, "/tmp/log.mcap", "Log to output.");
DEFINE_string(mode, "flatbuffer", "json or flatbuffer serialization.");
DEFINE_bool(
    canonical_channel_names, false,
    "If set, use full channel names; by default, will shorten names to be the "
    "shortest possible version of the name (e.g., /aos instead of /pi/aos).");
DEFINE_bool(compress, true, "Whether to use LZ4 compression in MCAP file.");

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

  aos::logger::LogReader reader(logfiles);

  reader.Register();

  const aos::Node *node =
      (replay_node.empty() ||
       !aos::configuration::MultiNode(reader.configuration()))
          ? nullptr
          : aos::configuration::GetNode(reader.configuration(), replay_node);

  std::unique_ptr<aos::EventLoop> mcap_event_loop =
      reader.event_loop_factory()->MakeEventLoop("mcap", node);
  CHECK(!FLAGS_output_path.empty());
  aos::McapLogger relogger(
      mcap_event_loop.get(), FLAGS_output_path,
      FLAGS_mode == "flatbuffer" ? aos::McapLogger::Serialization::kFlatbuffer
                                 : aos::McapLogger::Serialization::kJson,
      FLAGS_canonical_channel_names
          ? aos::McapLogger::CanonicalChannelNames::kCanonical
          : aos::McapLogger::CanonicalChannelNames::kShortened,
      FLAGS_compress ? aos::McapLogger::Compression::kLz4
                     : aos::McapLogger::Compression::kNone);
  reader.event_loop_factory()->Run();
}
