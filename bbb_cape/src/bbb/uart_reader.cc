#include "bbb/uart_reader.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/select.h>
#include <termio.h>
#include <unistd.h>

#include "aos/common/logging/logging.h"

// This is the code for receiving data from the cape via UART.
// NOTE: In order for this to work, you MUST HAVE
// "capemgr.enable_partno=BB_UART1"
// in your BBB's /media/BEAGLEBONE/uEnv.txt file!
// `su -c "echo BB-UART1 > /sys/devices/bone_capemgr.*/slots"` works too, but
// you have to do it every time.

namespace bbb {
namespace {
// TODO(brians): Determine this in some way that allows easy switching for
// testing with /dev/ttyUSB0 for example.
const char *device = "/dev/ttyO1";
}  // namespace

extern "C" {

// Sets all of the weird, annoying TTY flags on fd. In a separate file because
// the structure it uses is so weird and annoying and carries so much baggage it
// messes up all the #includes in whatever file it's used in.
// Defined in uart_reader_termios2.c.
extern int aos_uart_reader_set_tty_options(int fd, int baud_rate);

}  // extern "C"

UartReader::UartReader(int32_t baud_rate)
    : fd_(open(device, O_RDWR | O_NOCTTY | O_NONBLOCK)) {
  
  if (fd_ < 0) {
    LOG(FATAL, "open(%s, O_RDWR | O_NOCTTY | O_NOBLOCK) failed with %d: %s."
               " Did you read my note in bbb/uart_reader.cc?\n",
        device, errno, strerror(errno));
  }

  if (aos_uart_reader_set_tty_options(fd_, baud_rate) != 0) {
    LOG(FATAL, "aos_uart_reader_set_tty_options(%d) failed with %d: %s\n",
        fd_, errno, strerror(errno));
  }

  FD_ZERO(&fd_set_);
  FD_SET(fd_, &fd_set_);
}

UartReader::~UartReader() {
  if (fd_ > 0) close(fd_);
}

ssize_t UartReader::ReadBytes(AlignedChar *dest, size_t max_bytes,
                              const ::aos::time::Time &timeout_time) {
  do {
    ::aos::time::Time timeout = timeout_time - ::aos::time::Time::Now();
    if (timeout < ::aos::time::Time(0, 0)) return -2;
    struct timeval timeout_timeval = timeout.ToTimeval();
    switch (select(fd_ + 1, &fd_set_, NULL, NULL, &timeout_timeval)) {
      case 0:
        return -2;
      case -1:
        continue;
      case 1:
        break;
      default:
        LOG(WARNING, "unknown select return value\n");
        return -1;
    }
    ssize_t r = read(fd_, dest, max_bytes);
    if (r != -1) return r;
  } while (errno == EINTR);
  return -1;
}

}  // namespace bbb
