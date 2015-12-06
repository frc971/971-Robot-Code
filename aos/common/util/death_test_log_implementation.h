#ifndef AOS_COMMON_UTIL_DEATH_TEST_LOG_IMPLEMENTATION_H_
#define AOS_COMMON_UTIL_DEATH_TEST_LOG_IMPLEMENTATION_H_

#include <stdlib.h>

#include "aos/common/logging/implementations.h"

namespace aos {
namespace util {

// Prints all FATAL messages to stderr and then abort(3)s before the regular
// stuff can print out anything else. Ignores all other messages.
// This is useful in death tests that expect a LOG(FATAL) to cause the death.
class DeathTestLogImplementation : public logging::HandleMessageLogImplementation {
 public:
  virtual void HandleMessage(const logging::LogMessage &message) override {
    if (message.level == FATAL) {
      logging::internal::PrintMessage(stderr, message);
      abort();
    }
  }
};

}  // namespace util
}  // namespace aos

#endif  // AOS_COMMON_UTIL_DEATH_TEST_LOG_IMPLEMENTATION_H_
