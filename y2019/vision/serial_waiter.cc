#include <unistd.h>

#include <chrono>

#include "y2019/jevois/serial.h"
#include "aos/time/time.h"

using ::aos::monotonic_clock;
using ::y2019::jevois::open_via_terminos;

namespace chrono = ::std::chrono;

int main(int /*argc*/, char ** /*argv*/) {
  int serial_fd = open_via_terminos("/dev/ttyS0");

  // Print out a startup message.  The Teensy will ignore it as a corrupt
  // packet, but it gives us some warm fuzzies that things are booting right.
  (void)write(
      serial_fd,
      "Starting target_sender in 3 seconds.  Press 10 a's to interrupt.\r\n",
      66);

  // Give the user 3 seconds to press 10 a's.  If they don't do it in time,
  // return 0 to signal that we should boot, or 1 that we are asked not to boot.
  constexpr int kACount = 10;
  int a_count = 0;
  const monotonic_clock::time_point end_time =
      monotonic_clock::now() + chrono::seconds(3);
  do {
    constexpr size_t kBufferSize = 16;
    char data[kBufferSize];
    ssize_t n = ::read(serial_fd, &data[0], kBufferSize);
    for (int i = 0; i < n; ++i) {
      if (data[i] == 'a') {
        ++a_count;
        if (a_count > kACount) {
          ::close(serial_fd);
          return 1;
        }
      } else {
        a_count = 0;
      }
    }
  } while (monotonic_clock::now() < end_time);

  ::close(serial_fd);
  return 0;
}
