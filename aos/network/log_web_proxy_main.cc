// Sample binary for running the web server code against a logfile.
// This can be run by running
// bazel run -c opt //aos/network:log_web_proxy_main -- --node node_to_replay
// /path/to/logfile And then opening the plotting webpage at
// http://localhost:8080/graph.html

#include "absl/flags/flag.h"
#include "absl/log/check.h"

#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/init.h"
#include "aos/network/web_proxy.h"

ABSL_FLAG(std::string, data_dir, "www", "Directory to serve data files from");
ABSL_FLAG(std::string, node, "", "Directory to serve data files from");
ABSL_FLAG(int32_t, buffer_size, -1,
          "-1 if infinite, in # of messages / channel.");
ABSL_FLAG(double, monotonic_start_time, -1.0, "Start time (sec)");
ABSL_FLAG(double, monotonic_end_time, -1.0, "End time (sec)");
ABSL_FLAG(
    double, replay_rate, -1,
    "-1 to replay as fast as possible; 1.0 = realtime, 0.5 = half speed.");

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);

  const std::vector<aos::logger::LogFile> logfiles =
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv));

  aos::logger::LogReader reader(logfiles);

  reader.Register();

  // If going for "as fast as possible" don't actually use infinity, because we
  // don't want the log reading blocking our use of the epoll handlers.
  reader.SetRealtimeReplayRate(absl::GetFlag(FLAGS_replay_rate) == -1.0
                                   ? std::numeric_limits<double>::max()
                                   : absl::GetFlag(FLAGS_replay_rate));

  std::unique_ptr<aos::EventLoop> event_loop;

  if (absl::GetFlag(FLAGS_node).empty()) {
    CHECK(!aos::configuration::MultiNode(reader.configuration()))
        << "If using a multi-node logfile, please specify --node.";
    event_loop = reader.event_loop_factory()->MakeEventLoop("web_proxy");
  } else {
    event_loop = reader.event_loop_factory()->MakeEventLoop(
        "web_proxy", aos::configuration::GetNode(reader.configuration(),
                                                 absl::GetFlag(FLAGS_node)));
  }

  event_loop->SkipTimingReport();

  if (absl::GetFlag(FLAGS_monotonic_start_time) > 0) {
    event_loop->AddTimer([&reader]() { reader.event_loop_factory()->Exit(); })
        ->Schedule(aos::monotonic_clock::time_point(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(
                    absl::GetFlag(FLAGS_monotonic_start_time)))));

    reader.event_loop_factory()->Run();
  }

  aos::web_proxy::WebProxy web_proxy(
      event_loop.get(), reader.event_loop_factory()->scheduler_epoll(),
      aos::web_proxy::StoreHistory::kYes, absl::GetFlag(FLAGS_buffer_size));

  web_proxy.SetDataPath(absl::GetFlag(FLAGS_data_dir).c_str());

  if (absl::GetFlag(FLAGS_monotonic_end_time) > 0) {
    event_loop->AddTimer([&web_proxy]() { web_proxy.StopRecording(); })
        ->Schedule(aos::monotonic_clock::time_point(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(
                    absl::GetFlag(FLAGS_monotonic_end_time)))));
  }

  reader.event_loop_factory()->Run();

  // Keep the web proxy alive past when we finish reading the logfile, but crank
  // down the replay rate so that we don't peg our entire CPU just trying to
  // service timers in the web proxy code.
  reader.set_exit_on_finish(false);
  reader.SetRealtimeReplayRate(1.0);
  reader.event_loop_factory()->Run();
}
