#ifndef AOS_LOGGING_H_
#define AOS_LOGGING_H_

#include "aos/events/event_loop.h"
#include "aos/logging/implementations.h"
#include "aos/logging/log_message_generated.h"

namespace aos {

class AosLogToFbs {
 public:
  AosLogToFbs() {}

  // TODO(Tyler): Deregister logger on destruction to avoid memory leaks

  void Initialize(Sender<logging::LogMessageFbs> log_sender);

 private:
  Sender<logging::LogMessageFbs> log_sender_;
  logging::ScopedLogRestorer prev_logger_;
};

}  // namespace aos

#endif  // AOS_LOGGING_H_
