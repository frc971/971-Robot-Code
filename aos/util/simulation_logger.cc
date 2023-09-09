#include "aos/util/simulation_logger.h"

#include "aos/events/logging/logfile_utils.h"

namespace aos::util {
LoggerState::LoggerState(aos::SimulatedEventLoopFactory *factory,
                         const aos::Node *node, std::string_view output_folder,
                         bool do_skip_timing_report)
    : event_loop_(factory->MakeEventLoop("logger", node)),
      namer_(std::make_unique<aos::logger::MultiNodeFilesLogNamer>(
          absl::StrCat(output_folder, "/", logger::MaybeNodeName(node), "/"),
          event_loop_.get())),
      logger_(std::make_unique<aos::logger::Logger>(event_loop_.get())) {
  if (do_skip_timing_report) {
    event_loop_->SkipTimingReport();
  }
  event_loop_->SkipAosLog();
  event_loop_->OnRun([this]() { logger_->StartLogging(std::move(namer_)); });
}

std::vector<std::unique_ptr<LoggerState>> MakeLoggersForNodes(
    aos::SimulatedEventLoopFactory *factory,
    const std::vector<std::string> &nodes_to_log,
    std::string_view output_folder, bool do_skip_timing_report) {
  std::vector<std::unique_ptr<LoggerState>> loggers;
  for (const std::string &node : nodes_to_log) {
    loggers.emplace_back(std::make_unique<LoggerState>(
        factory, aos::configuration::GetNode(factory->configuration(), node),
        output_folder, do_skip_timing_report));
  }
  return loggers;
}

std::vector<std::unique_ptr<LoggerState>> MakeLoggersForAllNodes(
    aos::SimulatedEventLoopFactory *factory, std::string_view output_folder,
    bool do_skip_timing_report) {
  std::vector<std::unique_ptr<LoggerState>> loggers;
  for (const aos::Node *node :
       configuration::GetNodes(factory->configuration())) {
    loggers.emplace_back(std::make_unique<LoggerState>(
        factory, node, output_folder, do_skip_timing_report));
  }
  return loggers;
}

}  // namespace aos::util
