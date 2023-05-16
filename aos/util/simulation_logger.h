#ifndef AOS_UTIL_SIMULATION_LOGGER_H_
#define AOS_UTIL_SIMULATION_LOGGER_H_
#include <string_view>

#include "aos/events/logging/log_writer.h"
#include "aos/events/simulated_event_loop.h"
namespace aos::util {

class LoggerState {
 public:
  LoggerState(aos::SimulatedEventLoopFactory *factory, const aos::Node *node,
              std::string_view output_folder);

 private:
  std::unique_ptr<aos::EventLoop> event_loop_;
  std::unique_ptr<aos::logger::LogNamer> namer_;
  std::unique_ptr<aos::logger::Logger> logger_;
};

// Creates a logger for each of the specified nodes. This makes it so that you
// can easily setup some number of loggers in simulation or log replay without
// needing to redo all the boilerplate every time.
std::vector<std::unique_ptr<LoggerState>> MakeLoggersForNodes(
    aos::SimulatedEventLoopFactory *factory,
    const std::vector<std::string> &nodes_to_log,
    std::string_view output_folder);

// Creates loggers for all of the nodes.
std::vector<std::unique_ptr<LoggerState>> MakeLoggersForAllNodes(
    aos::SimulatedEventLoopFactory *factory, std::string_view output_folder);

}  // namespace aos::util
#endif  // AOS_UTIL_SIMULATION_LOGGER_H_
