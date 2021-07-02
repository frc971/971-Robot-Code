#include "aos/events/logging/boot_timestamp.h"

#include <iostream>

#include "aos/time/time.h"

namespace aos::logger {
std::ostream &operator<<(std::ostream &os,
                         const struct BootTimestamp &timestamp) {
  return os << "{.boot=" << timestamp.boot << ", .time=" << timestamp.time
            << "}";
}

}  // namespace aos::logger
