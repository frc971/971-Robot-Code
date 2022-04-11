#include <string>
#include "aos/events/event_loop.h"

#include "aos/logging/dynamic_log_command_generated.h"
#include "glog/logging.h"

// The purpose of this class is to listen for /aos aos.logging.DynamicLogCommand
// and make changes to the log level of the current application based on that
// message. Currently the only supported command is changing the global vlog
// level.
namespace aos {
namespace logging {

class DynamicLogging {
 public:
  DynamicLogging(aos::EventLoop *event_loop);
  ~DynamicLogging() {}

 private:
  void HandleDynamicLogCommand(const DynamicLogCommand &command);
  std::string application_name_;
  DISALLOW_COPY_AND_ASSIGN(DynamicLogging);
};

}  // namespace logging
}  // namespace aos
