#include "aos/events/event_loop_generated.h"
#include "aos/events/logging/log_reader.h"
#include "aos/init.h"
#include "aos/util/mcap_logger.h"

DEFINE_string(node, "", "Node to replay from the perspective of.");
DEFINE_string(output_path, "/tmp/log.mcap", "Log to output.");

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

  aos::logger::LogReader reader(logfiles);

  reader.Register();

  const aos::Node *node =
      aos::configuration::GetNode(reader.configuration(), FLAGS_node);
  CHECK_NOTNULL(node);
  std::unique_ptr<aos::EventLoop> mcap_event_loop =
      reader.event_loop_factory()->MakeEventLoop("mcap", node);
  CHECK(!FLAGS_output_path.empty());
  aos::McapLogger relogger(mcap_event_loop.get(), FLAGS_output_path);
  reader.event_loop_factory()->Run();
}
