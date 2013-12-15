#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "aos/common/logging/logging_impl.h"
#include "bbb_cape/src/cape/cows.h"
#include "crc.h"
#include "uart_receiver.h"

// This is the code for receiving data from the cape via UART.
// NOTE: In order for this to work, you MUST HAVE "capemgr.enable_partno=BB_UART1"
// in your BBB's /media/BEAGLEBONE/uEnv.txt file!

namespace bbb {

UartReceiver::UartReceiver(speed_t baud_rate, size_t packet_size) : 
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
  termios cape;

  if ((fd_ = open("/dev/ttyO1", O_RDWR | O_NOCTTY)) < 0) {
    LOG(FATAL, "Open() failed with error %d.\
      (Did you read my note in uart_receiver.cc?)\n", fd_);
  }
  if (int ret = tcgetattr(fd_, &cape)) {
    LOG(ERROR, "Tcgetattr() failed with error %d.\n", ret);
    return -1;
  }
  if (int ret = cfsetispeed(&cape, baud_rate_) < 0) {
    LOG(ERROR, "Cfsetispeed() failed with error %d.\n", ret);
    return -1;
  }

  cape.c_iflag = 0;
  cape.c_oflag = 0;
  cape.c_lflag = 0;
  cape.c_cc[VMIN] = 0; 
  cape.c_cc[VTIME] = 0;
  
  tcsetattr(fd_, TCSANOW, &cape);

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

