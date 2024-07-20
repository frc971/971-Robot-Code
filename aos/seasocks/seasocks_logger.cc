#include "aos/seasocks/seasocks_logger.h"

#include "absl/log/log.h"

#include "seasocks/PrintfLogger.h"

namespace aos::seasocks {

void SeasocksLogger::log(::seasocks::Logger::Level level, const char *message) {
  // Convert Seasocks error codes to glog.
  absl::LogSeverity log_level;
  switch (level) {
    case ::seasocks::Logger::Level::Info:
      log_level = absl::LogSeverity::kInfo;
      break;
    case ::seasocks::Logger::Level::Warning:
      log_level = absl::LogSeverity::kWarning;
      break;
    case ::seasocks::Logger::Level::Error:
    case ::seasocks::Logger::Level::Severe:
      log_level = absl::LogSeverity::kError;
      break;
    case ::seasocks::Logger::Level::Debug:
    case ::seasocks::Logger::Level::Access:
    default:
      if (!VLOG_IS_ON(1)) {
        return;
      }
      log_level = absl::LogSeverity::kInfo;
      break;
  }
  LOG(LEVEL(log_level)) << "Seasocks: " << message;
}

}  // namespace aos::seasocks
