#include <optional>

#include "gflags/gflags.h"

#include "aos/events/logging/log_reader.h"
#include "aos/init.h"
#include "aos/util/simulation_logger.h"
#include "frc971/input/joystick_state_generated.h"

DEFINE_string(output_folder, "/tmp/trimmed/",
              "Name of the folder to write the trimmed log to.");
DEFINE_double(pre_enable_time_sec, 10.0,
              "Amount of time to leave in the new log before the first enable "
              "signal happens.");
DEFINE_double(post_enable_time_sec, 1.0,
              "Amount of time to leave in the new log after the final enable "
              "signal ends.");

int main(int argc, char *argv[]) {
  gflags::SetUsageMessage(
      "Trims the sections at the start/end of a log where the robot is "
      "disabled.");
  aos::InitGoogle(&argc, &argv);
  const std::vector<aos::logger::LogFile> logfiles =
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv));
  std::optional<aos::monotonic_clock::time_point> start_time;
  std::optional<aos::monotonic_clock::time_point> end_time;
  bool printed_match = false;
  // We need to do two passes through the logfile; one to figure out when the
  // start/end times are, one to actually do the trimming.
  {
    aos::logger::LogReader reader(logfiles);
    const aos::Node *roborio =
        aos::configuration::GetNode(reader.configuration(), "roborio");
    reader.Register();
    std::unique_ptr<aos::EventLoop> event_loop =
        reader.event_loop_factory()->MakeEventLoop("roborio", roborio);
    event_loop->MakeWatcher(
        "/aos", [&start_time, &end_time, &printed_match,
                 &event_loop](const aos::JoystickState &msg) {
          if (!printed_match && msg.match_type() != aos::MatchType::kNone) {
            LOG(INFO) << "Match Type: "
                      << aos::EnumNameMatchType(msg.match_type());
            LOG(INFO) << "Match #: " << msg.match_number();
            printed_match = true;
          }

          if (msg.enabled()) {
            // Note that time is monotonic, so we don't need to e.g. do min's or
            // max's on the start/end time.
            if (!start_time.has_value()) {
              start_time = event_loop->context().monotonic_event_time;
            }
            end_time = event_loop->context().monotonic_event_time;
          }
        });

    reader.event_loop_factory()->Run();

    if (!printed_match) {
      LOG(INFO) << "No match info.";
    }
  }
  if (!start_time.has_value()) {
    LOG(WARNING) << "Log does not ontain any JoystickState messages.";
    return 1;
  }
  LOG(INFO) << "First enable at " << start_time.value();
  LOG(INFO) << "Final enable at " << end_time.value();
  start_time.value() -= std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(FLAGS_pre_enable_time_sec));
  end_time.value() += std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(FLAGS_post_enable_time_sec));

  {
    aos::logger::LogReader reader(logfiles);
    const aos::Node *roborio =
        aos::configuration::GetNode(reader.configuration(), "roborio");
    reader.Register();
    std::unique_ptr<aos::EventLoop> event_loop =
        reader.event_loop_factory()->MakeEventLoop("roborio", roborio);
    auto exit_timer = event_loop->AddTimer(
        [&reader]() { reader.event_loop_factory()->Exit(); });
    exit_timer->Schedule(start_time.value());
    reader.event_loop_factory()->Run();
    const std::set<std::string> logger_nodes =
        aos::logger::LoggerNodes(logfiles);
    // Only start up loggers that generated the original set of logfiles.
    // This mostly exists to make it so that utilities like log_to_mcap can
    // easily auto-detect which node to replay as when consuming the input logs.
    auto loggers = aos::util::MakeLoggersForNodes(
        reader.event_loop_factory(), {logger_nodes.begin(), logger_nodes.end()},
        FLAGS_output_folder);
    exit_timer->Schedule(end_time.value());

    reader.event_loop_factory()->Run();
  }

  LOG(INFO) << "Trimmed logs written to " << FLAGS_output_folder;

  return EXIT_SUCCESS;
}
