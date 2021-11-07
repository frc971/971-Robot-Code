#include "aos/seasocks/seasocks_logger.h"

#include "glog/logging.h"
#include "seasocks/PrintfLogger.h"

namespace aos {
namespace seasocks {

void SeasocksLogger::log(::seasocks::Logger::Level level, const char *message) {
  // Convert Seasocks error codes to glog.
  int glog_level;
  switch (level) {
    case ::seasocks::Logger::Level::Info:
      glog_level = google::INFO;
      break;
    case ::seasocks::Logger::Level::Warning:
      glog_level = google::WARNING;
      break;
    case ::seasocks::Logger::Level::Error:
    case ::seasocks::Logger::Level::Severe:
      glog_level = google::ERROR;
      break;
    case ::seasocks::Logger::Level::Debug:
    case ::seasocks::Logger::Level::Access:
    default:
      if (!VLOG_IS_ON(1)) {
        return;
      }
      glog_level = google::INFO;
      break;
  }
  LOG_AT_LEVEL(glog_level) << "Seasocks: " << message;
}

}  // namespace seasocks
}  // namespace aos
