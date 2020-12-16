// Sample binary for running the web server code against a logfile.
// This can be run by running
// bazel run -c opt //aos/network:log_web_proxy_main -- --node node_to_replay /path/to/logfile
// And then opening the plotting webpage at http://localhost:8080/graph.html

#include "aos/configuration.h"
#include "aos/events/logging/logger.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/init.h"
#include "aos/network/web_proxy.h"
#include "aos/seasocks/seasocks_logger.h"
#include "gflags/gflags.h"

#include "internal/Embedded.h"
#include "seasocks/Server.h"
#include "seasocks/WebSocket.h"

DEFINE_string(config, "./config.json", "File path of aos configuration");
DEFINE_string(data_dir, "www", "Directory to serve data files from");
DEFINE_string(node, "", "Directory to serve data files from");
DEFINE_int32(buffer_size, -1, "-1 if infinite, in # of messages / channel.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  // Make sure to reference this to force the linker to include it.
  findEmbeddedContent("");

  const std::vector<std::string> unsorted_logfiles =
      aos::logger::FindLogs(argc, argv);

  const std::vector<aos::logger::LogFile> logfiles =
      aos::logger::SortParts(unsorted_logfiles);

  aos::logger::LogReader reader(logfiles);

  reader.Register();

  std::unique_ptr<aos::EventLoop> event_loop;

  if (FLAGS_node.empty()) {
    CHECK(!aos::configuration::MultiNode(reader.configuration()))
        << "If using a multi-node logfile, please specify --node.";
    event_loop = reader.event_loop_factory()->MakeEventLoop("web_proxy");
  } else {
    event_loop = reader.event_loop_factory()->MakeEventLoop(
        "web_proxy",
        aos::configuration::GetNode(reader.configuration(), FLAGS_node));
  }

  event_loop->SkipTimingReport();

  aos::web_proxy::WebProxy web_proxy(event_loop.get(), FLAGS_buffer_size);

  web_proxy.SetDataPath(FLAGS_data_dir.c_str());

  // Keep the web proxy alive past when we finish reading the logfile.
  reader.set_exit_on_finish(false);

  reader.event_loop_factory()->Run();
}
