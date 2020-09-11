#ifndef AOS_LOGGING_H_
#define AOS_LOGGING_H_

#include "aos/events/event_loop.h"
#include "aos/logging/implementations.h"
#include "aos/logging/log_message_generated.h"

namespace aos {

class AosLogToFbs {
 public:
  AosLogToFbs() {}

  void Initialize(Sender<logging::LogMessageFbs> log_sender);
  std::shared_ptr<logging::LogImplementation> implementation() const {
    return implementation_;
  }

 private:
  Sender<logging::LogMessageFbs> log_sender_;
  std::shared_ptr<logging::LogImplementation> implementation_;
};

}  // namespace aos

#endif  // AOS_LOGGING_H_
