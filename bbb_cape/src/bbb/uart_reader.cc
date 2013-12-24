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

// Implementation for setting custom serial baud rates based on this code:
// <http://jim.sh/ftx/files/linux-custom-baudrate.c>

namespace bbb {
namespace {
// TODO(brians): Determine this in some way that allows easy switching for
// testing with /dev/ttyUSB0 for example.
// TODO(brians): Change back to /dev/ttyO1.
const char *device = "/dev/ttyUSB0";
}  // namespace

UartReader::UartReader(int32_t baud_rate)
    : fd_(open(device, O_RDWR | O_NDELAY | O_NOCTTY)) {
  
  if (fd_ < 0) {
    LOG(FATAL, "open(%s, O_RDWR | O_NOCTTY) failed with %d: %s."
               " Did you read my note in bbb/uart_reader.cc?\n",
        device, errno, strerror(errno));
  }

  {
    termios options;
    if (tcgetattr(fd_, &options) != 0) {
      LOG(FATAL, "tcgetattr(%d, %p) failed with %d: %s\n",
          fd_, &options, errno, strerror(errno));
    }
    if (cfsetispeed(&options, B38400) != 0) {
      LOG(FATAL, "cfsetispeed(%p, B38400) failed with %d: %s\n",
          &options, errno, strerror(errno));
    }
    if (cfsetospeed(&options, B38400) != 0) {
      LOG(FATAL, "cfsetospeed(%p, B38400) failed with %d: %s\n",
          &options, errno, strerror(errno));
    }
    cfmakeraw(&options);
    options.c_cflag |= CLOCAL;  // ignore modem flow control
    options.c_cflag |= CREAD;  // enable receiver
    options.c_cflag &= ~CRTSCTS;  // disable flow control
    //options.c_cflag |= PARENB;  // enable parity; defaults to even
    options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;  // 8 bits
    options.c_cflag &= ~CSTOPB;  // 1 stop bit
    options.c_iflag = 0;
    // Replace any received characters with parity or framing errors with 0s.
    options.c_iflag = (options.c_iflag & ~(IGNPAR | PARMRK)) | INPCK;
    //options.c_iflag |= IGNCR | PARMRK;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VMIN] = 20;
    options.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
      LOG(FATAL, "tcsetattr(%d, TCSANOW, %p) failed with %d: %s\n",
          fd_, &options, errno, strerror(errno));
    }
  }
 
  {
    serial_struct serinfo;
    if (ioctl(fd_, TIOCGSERIAL, &serinfo) != 0) {
      LOG(FATAL, "ioctl(%d, TIOCGSERIAL, %p) failed with %d: %s\n",
          fd_, &serinfo, errno, strerror(errno));
    }
    serinfo.reserved_char[0] = 0;
    serinfo.flags &= ~ASYNC_SPD_MASK;
    serinfo.flags |= ASYNC_SPD_CUST;
    //serinfo.flags |= ASYNC_LOW_LATENCY;
    serinfo.custom_divisor = static_cast<int>(
        static_cast<double>(serinfo.baud_base) / baud_rate + 0.5);
    if (serinfo.custom_divisor < 1) serinfo.custom_divisor = 1;
    if (ioctl(fd_, TIOCSSERIAL, &serinfo) < 0) {
      LOG(FATAL, "ioctl(%d, TIOCSSERIAL, %p) failed with %d: %s\n",
          fd_, &serinfo, errno, strerror(errno));
    }
    if (ioctl(fd_, TIOCGSERIAL, &serinfo) < 0) {
      LOG(FATAL, "ioctl(%d, TIOCGSERIAL, %p) failed with %d: %s\n",
          fd_, &serinfo, errno, strerror(errno));
    }
    if ((serinfo.flags & ASYNC_SPD_CUST) == 0) {
      LOG(FATAL, "Cannot set custom baud rate\n");
    }
    if (serinfo.custom_divisor * baud_rate != serinfo.baud_base) {
      LOG(WARNING, "actual baudrate is %d / %d = %f\n",
          serinfo.baud_base, serinfo.custom_divisor,
          static_cast<double>(serinfo.baud_base) / serinfo.custom_divisor);
    }
  }

  if (fcntl(fd_, F_SETFL, 0) != 0) {
    LOG(FATAL, "fcntl(%d, F_SETFL, 0) failed with %d: %s\n",
        fd_, errno, strerror(errno));
  }

  // Implement timeout.
  fd_set readfd;
  timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 500000;
  FD_ZERO(&readfd);
  FD_SET(fd_, &readfd);
  select(fd_ + 1, &readfd, NULL, NULL, &tv);
}

UartReader::~UartReader() {
  if (fd_ > 0) close(fd_);
}

ssize_t UartReader::ReadBytes(AlignedChar *dest, size_t max_bytes) {
  return read(fd_, dest, max_bytes);
}

} // namespace bbb
