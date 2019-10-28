#include "aos/seasocks/seasocks_logger.h"

#include "aos/logging/logging.h"
#include "seasocks/PrintfLogger.h"

namespace aos {
namespace seasocks {

void SeasocksLogger::log(::seasocks::Logger::Level level, const char *message) {
  // Convert Seasocks error codes to AOS.
  log_level aos_level;
  switch (level) {
    case ::seasocks::Logger::Level::Info:
      aos_level = INFO;
      break;
    case ::seasocks::Logger::Level::Warning:
      aos_level = WARNING;
      break;
    case ::seasocks::Logger::Level::Error:
    case ::seasocks::Logger::Level::Severe:
      aos_level = ERROR;
      break;
    case ::seasocks::Logger::Level::Debug:
    case ::seasocks::Logger::Level::Access:
    default:
      aos_level = DEBUG;
      break;
  }
  AOS_LOG(aos_level, "Seasocks: %s\n", message);
}

}  // namespace seasocks
}  // namespace aos
