#include "aos/common/queue.h"

#include "aos/common/byteorder.h"
#include <inttypes.h>

namespace aos {

void Message::Zero() {
  sent_time.set_sec(0);
  sent_time.set_nsec(0);
}

size_t Message::Deserialize(const char *buffer) {
  int32_t sec;
  int32_t nsec;
  to_host(&buffer[0], &sec);
  to_host(&buffer[4], &nsec);
  sent_time.set_sec(sec);
  sent_time.set_nsec(nsec);
  return Size();
}
// Serializes the common fields into the buffer.
size_t Message::Serialize(char *buffer) const {
  // TODO(aschuh): to_network shouldn't need a pointer.
  int32_t sec = sent_time.sec();
  int32_t nsec = sent_time.nsec();
  to_network(&sec, &buffer[0]);
  to_network(&nsec, &buffer[4]);
  return Size();
}

size_t Message::Print(char *buffer, int length) const {
  return snprintf(buffer, length, "%" PRId32 ".%09" PRId32 "s",
                  sent_time.sec(), sent_time.nsec());
}

}  // namespace aos
