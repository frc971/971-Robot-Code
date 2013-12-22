#ifndef BBB_CAPE_SRC_BBB_UART_READER_H_
#define BBB_CAPE_SRC_BBB_UART_READER_H_

#include <stdint.h>
#include <string.h>

#include "bbb/packet_finder.h"

namespace bbb {

class UartReader : public PacketFinder {
 public:
  UartReader(int32_t baud_rate);
  virtual ~UartReader();

  int ReadBytes(AlignedChar *dest, size_t max_bytes);

 private:
  const int fd_;
};

}  // namespace bbb

#endif
