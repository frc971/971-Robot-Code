#include "bbb/uart_reader.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <unistd.h>

#include "aos/common/logging/logging.h"

// This is the code for receiving data from the cape via UART.
// fragment active.
// `su -c "echo BB-UART1 > /sys/devices/bone_capemgr.*/slots"` works, but
// you have to do it every time. It is also possible to set it up to do that
// every time it boots.

namespace bbb {
namespace {

// TODO(brians): Determine this in some way that allows easy switching for
// testing with /dev/ttyUSB0 for example.
const char *device = "/dev/ttyO1";

bool easy_access(const char *path) {
  if (access(path, R_OK | W_OK) == 0) return true;
  if (errno == EACCES || errno == ENOENT) return false;
  LOG(FATAL, "access(%s, F_OK) failed with %d: %s\n", path, errno,
      strerror(errno));
}

int open_device() {
  if (easy_access(device)) {
    LOG(INFO, "unexporting BB-UART1\n");
    if (system("bash -c 'echo -$(cat /sys/devices/bone_capemgr.*/slots"
               " | fgrep BB-UART1"
               " | cut -d : -f 1 | tr -d \" \")"
               " > /sys/devices/bone_capemgr.*/slots'") == -1) {
      LOG(FATAL, "system([disable OMAP UART]) failed with %d: %s\n", errno,
          strerror(errno));
    }
    while (easy_access(device)) {
      LOG(DEBUG, "waiting for BB-UART1 to be unexported\n");
      ::aos::time::SleepFor(::aos::time::Time::InSeconds(0.1));
    }
  }

  LOG(INFO, "exporting BB-UART1\n");
  // 2 strings to work around a VIM bug where the indenter locks up when they're
  // combined as 1...
  if (system("bash -c 'echo BB-UART1 > /sys/devices/bone_capemgr.*"
             "/slots'") ==
      -1) {
    LOG(FATAL, "system([enable OMAP UART]) failed with %d: %s\n", errno,
        strerror(errno));
  }
  while (!easy_access(device)) {
    LOG(DEBUG, "waiting for BB-UART1 to be exported\n");
    ::aos::time::SleepFor(::aos::time::Time::InSeconds(0.1));
  }

  return open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
}

}  // namespace

extern "C" {

// Sets all of the weird, annoying TTY flags on fd. In a separate file because
// the structure it uses is so weird and annoying and carries so much baggage it
// messes up all the #includes in whatever file it's used in.
// Defined in uart_reader_termios2.c.
extern int aos_uart_reader_set_tty_options(int fd, int baud_rate);

}  // extern "C"

UartReader::UartReader(int32_t baud_rate)
    : fd_(open_device()) {
  
  if (fd_ < 0) {
    LOG(FATAL, "open(%s, O_RDWR | O_NOCTTY | O_NOBLOCK) failed with %d: %s\n",
        device, errno, strerror(errno));
  }

  if (aos_uart_reader_set_tty_options(fd_, baud_rate) != 0) {
    LOG(FATAL, "aos_uart_reader_set_tty_options(%d) failed with %d: %s\n",
        fd_, errno, strerror(errno));
  }
}

UartReader::~UartReader() {
  if (fd_ > 0) close(fd_);
}

ssize_t UartReader::ReadBytes(uint8_t *dest, size_t max_bytes,
                              const ::aos::time::Time &timeout_time) {
  do {
    ::aos::time::Time timeout = timeout_time - ::aos::time::Time::Now();
    if (timeout < ::aos::time::Time(0, 0)) return -2;
    struct timeval timeout_timeval = timeout.ToTimeval();
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(fd_, &fds);
    switch (select(fd_ + 1, &fds, NULL, NULL, &timeout_timeval)) {
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

bool UartReader::WriteBytes(uint8_t *bytes, size_t number_bytes) {
  size_t written = 0;
  while (written < number_bytes) {
    ssize_t r = write(fd_, &bytes[written], number_bytes - written);
    if (r == -1) return false;
    written += r;
  }
  return true;
}

}  // namespace bbb
