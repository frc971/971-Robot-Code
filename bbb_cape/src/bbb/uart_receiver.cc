#include <fcntl.h>
#include <linux/serial.h>
#include <termio.h>
#include <unistd.h>

#include <cmath>
#include <cstring>

#include "aos/common/logging/logging_impl.h"
#include "bbb_cape/src/cape/cows.h"
#include "crc.h"
#include "uart_receiver.h"

// This is the code for receiving data from the cape via UART.
// NOTE: In order for this to work, you MUST HAVE "capemgr.enable_partno=BB_UART1"
// in your BBB's /media/BEAGLEBONE/uEnv.txt file!

// Implementation for setting custom serial baud rates based on this code:
// <http://jim.sh/ftx/files/linux-custom-baudrate.c>

namespace bbb {

UartReceiver::UartReceiver(uint32_t baud_rate, size_t packet_size) : 
  baud_rate_(baud_rate), packet_size_(packet_size) {
  //packet_size_ should be a multiple of four.
  int toadd = packet_size_ % 4;
  LOG(DEBUG, "Increasing packet size by %d bytes.\n", toadd);
  packet_size_ += toadd;
  
  // See cows.h for where this comes from.
  stuffed_size_ = ((packet_size_ - 1) / (pow(2, 32) - 1) + 1) * 4 + packet_size_;
  
  buf_ = static_cast<char *>(malloc(stuffed_size_));  
}

UartReceiver::~UartReceiver() {
  free(buf_);
}

int UartReceiver::SetUp() {
  termios options;
  serial_struct serinfo;

  if ((fd_ = open("/dev/ttyO1", O_RDWR | O_NOCTTY)) < 0) {
    LOG(FATAL, "Open() failed with error %d.\
      (Did you read my note in uart_receiver.cc?)\n", fd_);
  }
 
  // Must implement an ugly custom divisor.
  serinfo.reserved_char[0] = 0;
  if (ioctl(fd_, TIOCGSERIAL, &serinfo) < 0)
    return -1;
  serinfo.flags &= ~ASYNC_SPD_MASK;
  serinfo.flags |= ASYNC_SPD_CUST;
  serinfo.custom_divisor = (serinfo.baud_base + (baud_rate_ / 2)) / baud_rate_;
  if (serinfo.custom_divisor < 1) 
    serinfo.custom_divisor = 1;
  if (ioctl(fd_, TIOCSSERIAL, &serinfo) < 0)
    LOG(ERROR, "First ioctl failed.\n");
    return -1;
  if (ioctl(fd_, TIOCGSERIAL, &serinfo) < 0)
    LOG(ERROR, "Second ioctl failed.\n");
    return -1;
  if (serinfo.custom_divisor * static_cast<int>(baud_rate_) 
    != serinfo.baud_base) {
    LOG(WARNING, "actual baudrate is %d / %d = %f",
          serinfo.baud_base, serinfo.custom_divisor,
          (float)serinfo.baud_base / serinfo.custom_divisor);
  }

  fcntl(fd_, F_SETFL, 0);
  tcgetattr(fd_, &options);
  cfsetispeed(&options, B38400);
  cfsetospeed(&options, B38400);
  cfmakeraw(&options);
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~CRTSCTS;
  options.c_iflag = 0;
  options.c_oflag = 0;
  options.c_lflag = 0;
  // We know the minimum size for packets.
  // No use in having it do excessive syscalls.
  options.c_cc[VMIN] = packet_size_;
  options.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &options) != 0)
    LOG(ERROR, "Tcsetattr failed.\n");
    return -1;

  return 0;
}

int UartReceiver::GetPacket(DataStruct *packet) {
  int pstarti = 0, bread, cons_zeros = 0;
  uint32_t readi = 0;
  bool done = false, has_packet = false;
  uint32_t ptemp [(stuffed_size_ - 1) / 4 + 1];

  while (!done && buf_used_ < stuffed_size_) {
    if ((bread = read(fd_, buf_ + buf_used_, stuffed_size_ - buf_used_)) < 0) {
      LOG(WARNING, "Read() failed with error %d.\n", bread);
      return -1;
    }
    buf_used_ += bread;

    // Find the beginning of the packet.
    // Look for four bytes of zeros.
    while (readi < buf_used_) {
      if (buf_[readi] == 0) {
        if (cons_zeros == 4) {
          if (has_packet) {
            // We got to the end of a packet.
            done = true;
            break;
          } else {
            // We got to the start of a packet.
            has_packet = true;
            pstarti = readi - 3;
          }
        } else {
          ++cons_zeros;
        }
      } else {
        cons_zeros = 0;
      }
    }
    ++readi;
  }

  // Copy packet data to output.
  int filled = 0;
  readi -= 3;
  for (uint32_t i = pstarti; i < readi - 3; ++i) {
    ptemp[i] = buf_[i];
    ++filled;
  }
  // Move everything we didn't use to the beginning of the buffer for next time.
  uint32_t puti = 0;
  for (uint32_t i = readi; i < stuffed_size_; ++i) {
    buf_[puti++] = buf_[i];
  }
  buf_used_ = stuffed_size_ - readi;

  // Cows algorithm always outputs something 4-byte aligned.
  if (filled % 4) {
    LOG(WARNING, "Rejecting packet due to it not being possible\
      for cows to have created it.\n");
    return -1;
  }

  // Unstuff our packet.
  uint32_t ptemp_unstuffed [packet_size_];
  uint32_t bunstuffed;
  if ((bunstuffed = cows_unstuff(ptemp, sizeof(ptemp), ptemp_unstuffed)) == 0) {
    LOG(WARNING, "Rejecting packet due to failure to unstuff it.\n");
    return -1;
  }
  if (bunstuffed != packet_size_) {
    LOG(WARNING, "Rejecting packet of wrong size.\
      Expected packet of size %d, got packet of size %d.\n",
      packet_size_, bunstuffed);
    return -1;
  }

  // Make sure the checksum checks out.
  uint32_t checksum = static_cast<uint32_t>(ptemp_unstuffed[packet_size_ - 4]);
  memcpy(packet, ptemp_unstuffed, sizeof(DataStruct));
  if (cape::CalculateChecksum((uint8_t *)packet, sizeof(DataStruct)) != checksum) {
    LOG(WARNING, "Rejecting packet due to checksum failure.\n");
    return -1;
  }

  return 0;
}

} //bbb

