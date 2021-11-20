#ifndef AOS_LOGGING_H_
#define AOS_LOGGING_H_

#include "aos/events/event_loop.h"
#include "aos/logging/implementations.h"
#include "aos/logging/log_message_generated.h"

namespace aos {

class AosLogToFbs {
 public:
  AosLogToFbs() {}

  // Initializes a sender with the provided name and sender.  Note: the name
  // needs to be valid until this object is destroyed.
  void Initialize(const std::string *name,
                  Sender<logging::LogMessageFbs> log_sender);
  std::shared_ptr<logging::LogImplementation> implementation() const {
    return implementation_;
  }

 private:
  Sender<logging::LogMessageFbs> log_sender_;
  std::shared_ptr<logging::LogImplementation> implementation_;
};

}  // namespace aos

#endif  // AOS_LOGGING_H_
