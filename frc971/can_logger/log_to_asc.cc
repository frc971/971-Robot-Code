#include "aos/configuration.h"
#include "aos/events/event_loop_generated.h"
#include "aos/events/logging/log_reader.h"
#include "aos/init.h"
#include "frc971/can_logger/asc_logger.h"
#include "frc971/can_logger/can_logging_generated.h"

DEFINE_string(node, "", "Node to replay from the perspective of.");
DEFINE_string(output_path, "/tmp/can_log.asc", "Log to output.");

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

  aos::logger::LogReader reader(
      logfiles, config.has_value() ? &config.value().message() : nullptr);
  aos::SimulatedEventLoopFactory factory(reader.configuration());
  reader.RegisterWithoutStarting(&factory);

  const aos::Node *node =
      (replay_node.empty() ||
       !aos::configuration::MultiNode(reader.configuration()))
          ? nullptr
          : aos::configuration::GetNode(reader.configuration(), replay_node);

  std::unique_ptr<aos::EventLoop> can_event_loop;
  CHECK(!FLAGS_output_path.empty());
  std::unique_ptr<frc971::can_logger::AscLogger> relogger;

  factory.GetNodeEventLoopFactory(node)->OnStartup([&relogger, &can_event_loop,
                                                    &reader, node]() {
    can_event_loop = reader.event_loop_factory()->MakeEventLoop("can", node);
    relogger = std::make_unique<frc971::can_logger::AscLogger>(
        can_event_loop.get(), FLAGS_output_path);
  });
  reader.event_loop_factory()->Run();
  reader.Deregister();
}
