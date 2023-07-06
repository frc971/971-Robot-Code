#ifndef FRC971_CAN_LOGGER_ASC_LOGGER_H_
#define FRC971_CAN_LOGGER_ASC_LOGGER_H_

#include <iomanip>
#include <iostream>

#include "gflags/gflags.h"
#include "glog/logging.h"

#include "aos/events/event_loop.h"
#include "frc971/can_logger/can_logging_generated.h"

namespace frc971 {
namespace can_logger {

class AscLogger {
 public:
  AscLogger(aos::EventLoop *event_loop, const std::string &filename);

 private:
  void HandleFrame(const CanFrame &frame);

  // This implementation attempts to duplicate the output of can-utils/log2asc
  void WriteFrame(std::ostream &file, const CanFrame &frame);

  static void WriteHeader(std::ostream &file,
                          aos::realtime_clock::time_point start_time);

  std::optional<aos::monotonic_clock::time_point> first_frame_monotonic_;

  std::ofstream output_;

  aos::EventLoop *event_loop_;
};

}  // namespace can_logger
}  // namespace frc971

#endif  // FRC971_CAN_LOGGER_ASC_LOGGER_H_
