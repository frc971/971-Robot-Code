#ifndef AOS_EVENTS_LOGGING_BOOT_TIMESTAMP_H_
#define AOS_EVENTS_LOGGING_BOOT_TIMESTAMP_H_

#include <iostream>

#include "aos/time/time.h"

namespace aos::logger {

// Simple class representing which boot and what monotonic time in that boot.
// Boots are assumed to be sequential, and the monotonic clock resets on reboot
// for all the compare operations.
struct BootTimestamp {
  // Boot number for this timestamp.
  size_t boot = 0u;
  // Monotonic time in that boot.
  monotonic_clock::time_point time = monotonic_clock::min_time;

  static constexpr BootTimestamp min_time() {
    return BootTimestamp{.boot = 0u, .time = monotonic_clock::min_time};
  }
  static constexpr BootTimestamp max_time() {
    return BootTimestamp{.boot = std::numeric_limits<size_t>::max(),
                         .time = monotonic_clock::max_time};
  }

  // Compare operators.  These are implemented such that earlier boots always
  // compare less than later boots, and the times are only compared in a single
  // boot.
  bool operator<(const BootTimestamp &m2) const;
  bool operator<=(const BootTimestamp &m2) const;
  bool operator>=(const BootTimestamp &m2) const;
  bool operator>(const BootTimestamp &m2) const;
  bool operator==(const BootTimestamp &m2) const;
  bool operator!=(const BootTimestamp &m2) const;
};

std::ostream &operator<<(std::ostream &os,
                         const struct BootTimestamp &timestamp);

inline bool BootTimestamp::operator<(const BootTimestamp &m2) const {
  if (boot != m2.boot) {
    return boot < m2.boot;
  }

  return time < m2.time;
}
inline bool BootTimestamp::operator<=(const BootTimestamp &m2) const {
  if (boot != m2.boot) {
    return boot <= m2.boot;
  }

  return time <= m2.time;
}
inline bool BootTimestamp::operator>=(const BootTimestamp &m2) const {
  if (boot != m2.boot) {
    return boot >= m2.boot;
  }

  return time >= m2.time;
}
inline bool BootTimestamp::operator>(const BootTimestamp &m2) const {
  if (boot != m2.boot) {
    return boot > m2.boot;
  }

  return time > m2.time;
}

inline bool BootTimestamp::operator==(const BootTimestamp &m2) const {
  return boot == m2.boot && time == m2.time;
}
inline bool BootTimestamp::operator!=(const BootTimestamp &m2) const {
  return boot != m2.boot || time != m2.time;
}

}  // namespace aos::logger

#endif  // AOS_EVENTS_LOGGING_BOOT_TIMESTAMP_H_
