#ifndef AOS_EVENTS_LOGGING_BOOT_TIMESTAMP_H_
#define AOS_EVENTS_LOGGING_BOOT_TIMESTAMP_H_

#include <iostream>

#include "aos/time/time.h"
#include "glog/logging.h"

namespace aos::logger {

// Simple class representing a duration in time and a boot it is from.  This
// gives us something to use for storing the time offset when filtering.
struct BootDuration {
  // Boot number for this timestamp.
  size_t boot = 0u;
  // Monotonic time in that boot.
  monotonic_clock::duration duration{0};

  BootDuration operator+(monotonic_clock::duration d) const {
    return {boot, duration + d};
  }

  BootDuration operator-() const { return {boot, -duration}; }
  BootDuration operator-(monotonic_clock::duration d) const {
    return {boot, duration - d};
  }

  BootDuration operator-(BootDuration d) const {
    CHECK_EQ(d.boot, boot);
    return {boot, duration - d.duration};
  }

  BootDuration operator+(BootDuration d) const {
    CHECK_EQ(d.boot, boot);
    return {boot, duration + d.duration};
  }

  BootDuration operator/(int x) const { return {boot, duration / x}; }

  bool operator==(const BootDuration &m2) const {
    return boot == m2.boot && duration == m2.duration;
  }
  bool operator!=(const BootDuration &m2) const {
    return boot != m2.boot || duration != m2.duration;
  }

  static constexpr BootDuration max_time() {
    return BootDuration{
        .boot = std::numeric_limits<size_t>::max(),
        .duration = monotonic_clock::duration(
            ::std::numeric_limits<monotonic_clock::duration::rep>::max())};
  }
};

// Simple class representing which boot and what monotonic time in that boot.
// Boots are assumed to be sequential, and the monotonic clock resets on reboot
// for all the compare operations.
struct BootTimestamp {
  // Boot number for this timestamp.
  size_t boot = 0u;
  // Monotonic time in that boot.
  monotonic_clock::time_point time = monotonic_clock::min_time;

  monotonic_clock::duration time_since_epoch() const {
    return time.time_since_epoch();
  }

  static constexpr BootTimestamp min_time() {
    return BootTimestamp{.boot = std::numeric_limits<size_t>::min(),
                         .time = monotonic_clock::min_time};
  }
  static constexpr BootTimestamp max_time() {
    return BootTimestamp{.boot = std::numeric_limits<size_t>::max(),
                         .time = monotonic_clock::max_time};
  }
  static constexpr BootTimestamp epoch() {
    return BootTimestamp{.boot = 0, .time = monotonic_clock::epoch()};
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

  BootTimestamp operator+(monotonic_clock::duration d) const {
    return {boot, time + d};
  }

  BootTimestamp operator+=(monotonic_clock::duration d) {
    time += d;
    return *this;
  }
  BootTimestamp operator-(monotonic_clock::duration d) const {
    return {boot, time - d};
  }
  BootTimestamp operator+(BootDuration d) const {
    return {boot, time + d.duration};
  }
};

// Structure to hold both a boot and queue index.  Queue indices reset after
// reboot, so we need to track them.
struct BootQueueIndex {
  // Boot number for this queue index.
  size_t boot = std::numeric_limits<size_t>::max();
  // Queue index.
  uint32_t index = std::numeric_limits<uint32_t>::max();

  // Returns a QueueIndex representing an invalid index.  Since
  // std::numeric_limits<uint32_t>::max() is never used in the QueueIndex code
  // and is reserved as an Invalid value, this will never collide.
  static BootQueueIndex Invalid() {
    return {.boot = std::numeric_limits<size_t>::max(),
            .index = std::numeric_limits<uint32_t>::max()};
  }

  bool operator==(const BootQueueIndex &b2) const {
    return index == b2.index && boot == b2.boot;
  }
  bool operator!=(const BootQueueIndex &b2) const {
    return index != b2.index || boot != b2.boot;
  }
  bool operator<(const BootQueueIndex &b2) const {
    if (boot == b2.boot) {
      return index < b2.index;
    }
    return boot < b2.boot;
  }
  bool operator>(const BootQueueIndex &b2) const {
    if (boot == b2.boot) {
      return index > b2.index;
    }
    return boot > b2.boot;
  }
};

std::ostream &operator<<(std::ostream &os,
                         const struct BootTimestamp &timestamp);
std::ostream &operator<<(std::ostream &os, const struct BootDuration &duration);
std::ostream &operator<<(std::ostream &os,
                         const struct BootQueueIndex &queue_index);

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
