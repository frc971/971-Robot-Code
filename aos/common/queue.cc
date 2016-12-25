#include "aos/common/queue.h"

#include <inttypes.h>
#include <chrono>

#include "aos/common/byteorder.h"

namespace aos {

namespace chrono = ::std::chrono;

void Message::Zero() { sent_time = monotonic_clock::min_time; }

size_t Message::Deserialize(const char *buffer) {
  int32_t sec;
  int32_t nsec;
  to_host(&buffer[0], &sec);
  to_host(&buffer[4], &nsec);
  sent_time = monotonic_clock::time_point(chrono::seconds(sec) +
                                          chrono::nanoseconds(nsec));
  return Size();
}
// Serializes the common fields into the buffer.
size_t Message::Serialize(char *buffer) const {
  // TODO(aschuh): to_network shouldn't need a pointer.
  int32_t sec =
      chrono::duration_cast<chrono::seconds>(sent_time.time_since_epoch())
          .count();
  int32_t nsec = chrono::duration_cast<chrono::nanoseconds>(
                     sent_time.time_since_epoch() - chrono::seconds(sec))
                     .count();
  to_network(&sec, &buffer[0]);
  to_network(&nsec, &buffer[4]);
  return Size();
}

size_t Message::Print(char *buffer, int length) const {
  int32_t sec =
      chrono::duration_cast<chrono::seconds>(sent_time.time_since_epoch())
          .count();
  int32_t nsec = chrono::duration_cast<chrono::nanoseconds>(
                     sent_time.time_since_epoch() - chrono::seconds(sec))
                     .count();
  return snprintf(buffer, length, "%" PRId32 ".%09" PRId32 "s", sec, nsec);
}

}  // namespace aos
