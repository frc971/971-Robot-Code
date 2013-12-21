#include "bbb/uart_reader.h"

#include <fcntl.h>
#include <linux/serial.h>
#include <termio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <inttypes.h>

#include <algorithm>

#include "aos/common/logging/logging.h"
#include "bbb_cape/src/cape/cows.h"
#include "bbb/crc.h"

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
    : baud_rate_(baud_rate),
      buf_(new AlignedChar[DATA_STRUCT_SEND_SIZE]),
      fd_(open(device, O_RDWR | O_NOCTTY)) {
  static_assert((DATA_STRUCT_SEND_SIZE % 4) == 0,
                "We can't do checksums of lengths that aren't multiples of 4.");

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
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;
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
        static_cast<double>(serinfo.baud_base) / baud_rate_ + 0.5);
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
      LOG(FATAL, "can not set custom baud rate\n");
    }
    if (serinfo.custom_divisor * baud_rate_ != serinfo.baud_base) {
      LOG(WARNING, "actual baudrate is %d / %d = %f\n",
          serinfo.baud_base, serinfo.custom_divisor,
          static_cast<double>(serinfo.baud_base) / serinfo.custom_divisor);
    }
  }

  if (fcntl(fd_, F_SETFL, 0) != 0) {
    LOG(FATAL, "fcntl(%d, F_SETFL, 0) failed with %d: %s\n",
        fd_, errno, strerror(errno));
  }
}

UartReader::~UartReader() {
  delete buf_;
  if (fd_ > 0) close(fd_);
}

bool UartReader::FindPacket() {
  // How many 0 bytes we've found at the front so far.
  int zeros_found = 0;
  // How many bytes of the packet we've read in (or -1 if we don't know where
  // the packet is).
  int packet_bytes = -1;
  while (true) {
    size_t already_read = ::std::max(packet_bytes, 0);
    ssize_t new_bytes =
        read(fd_, buf_ + already_read, DATA_STRUCT_SEND_SIZE - already_read);
    LOG(DEBUG, "read %zd, wanted %d\n", new_bytes,
        DATA_STRUCT_SEND_SIZE - already_read);
    for (int i = 0; i < new_bytes; ++i) {
      LOG(DEBUG, "%x\n", buf_[i]);
    }
    if (new_bytes < 0) {
      if (errno == EINTR) continue;
      LOG(WARNING, "read(%d, %p, %zd) failed with %d: %s\n",
          fd_, buf_ + already_read, DATA_STRUCT_SEND_SIZE - already_read,
          errno, strerror(errno));
      return false;
    }

    if (packet_bytes != -1) {  // we think there's a packet at the beginning of
                               // our buffer
      for (int to_check = packet_bytes; packet_bytes + new_bytes; ++to_check) {
        // We shouldn't find any 0s in the middle of what should be a packet.
        if (buf_[to_check] == 0) {
          packet_bytes = -1;
          memmove(buf_, buf_ + to_check, new_bytes - to_check);
          new_bytes -= to_check;
          break;
        }
      }
      if (packet_bytes != -1) {
        packet_bytes += new_bytes;
        if (packet_bytes == DATA_STRUCT_SEND_SIZE) return true;
      }
    }
    // This can't just be an else because the above code might set it to -1 if
    // it finds 0s in the middle of a packet.
    if (packet_bytes == -1) {
      // Find the beginning of the packet (aka look for four zero bytes).
      for (ssize_t checked = 0; checked < new_bytes; ++checked) {
        if (buf_[checked] == 0) {
          ++zeros_found;
          if (zeros_found == 4) {
            packet_bytes = new_bytes - checked - 1;
            memmove(buf_, buf_ + checked + 1, packet_bytes);
            break;
          }
        } else {
          zeros_found = 0;
        }
      }
    }
  }
}

bool UartReader::GetPacket(DataStruct *packet) {
  if (!FindPacket()) return false;

  uint32_t unstuffed = cows_unstuff(reinterpret_cast<uint32_t *>(buf_),
                                    DATA_STRUCT_SEND_SIZE / 4,
                                    reinterpret_cast<uint32_t *>(packet));
  if (unstuffed == 0) {
    LOG(WARNING, "invalid packet\n");
  } else if (unstuffed != sizeof(packet)) {
    LOG(WARNING, "packet is %" PRIu32 " words instead of %" PRIu32 "\n",
        unstuffed, DATA_STRUCT_SEND_SIZE / 4);
  }

  // Make sure the checksum checks out.
  uint32_t checksum;
  memcpy(&checksum, buf_ + DATA_STRUCT_SEND_SIZE - 4, 4);
  if (cape::CalculateChecksum(reinterpret_cast<uint8_t *>(packet),
                              sizeof(DataStruct)) != checksum) {
    LOG(WARNING, "Rejecting packet due to checksum failure.\n");
    return false;
  }

  return true;
}

}  // namespace bbb
