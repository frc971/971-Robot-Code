#ifndef FCR971_INPUT_UART_RECEIVER_H_
#define FRC971_INPUT_UART_RECEIVER_H_

#include <stdint.h>

#define DATA_STRUCT_NAME DataStruct
#include "cape/data_struct.h"
#undef DATA_STRUCT_NAME

namespace bbb {

class UartReceiver {
 public:
  UartReceiver(int32_t baud_rate);
  ~UartReceiver();

  // Returns true if it finds one or false if it gets an I/O error first.
  // packet must be aligned to 4 bytes.
  bool GetPacket(DataStruct *packet);

 private:
  // Reads bytes until there are 4 zeros and then fills up buf_.
  // Returns true if it finds one or false if it gets an I/O error first or the
  // packet is invalid in some way.
  bool FindPacket();

  typedef char __attribute__((aligned(8))) AlignedChar;

  const int32_t baud_rate_;
  AlignedChar *const buf_;
  const int fd_;
};

}  // namespace bbb

#endif
