#ifndef FCR971_INPUT_UART_RECEIVER_H_
#define FRC971_INPUT_UART_RECEIVER_H_

#include <stdint.h>

#include <memory>

#define DATA_STRUCT_NAME DataStruct
#include "cape/data_struct.h"
#undef DATA_STRUCT_NAME

namespace bbb {

class UartReader {
 public:
  UartReader(int32_t baud_rate);
  ~UartReader();

  // Returns true if it finds one or false if it gets an I/O error first.
  // packet must be aligned to 4 bytes.
  bool GetPacket(DataStruct *packet);

 private:
  // Reads bytes until there are 4 zeros and then fills up buf_.
  // Returns true if it finds one or false if it gets an I/O error first or the
  // packet is invalid in some way.
  bool FindPacket();

  // Processes a packet currently in buf_ and leaves the result in
  // unstuffed_data_.
  // Returns true if it succeeds or false if there was something wrong with the
  // data.
  bool ProcessPacket();

  typedef char __attribute__((aligned(4))) AlignedChar;

  const int32_t baud_rate_;
  AlignedChar *const buf_;
  AlignedChar *const unstuffed_data_;
  const int fd_;

  // How many bytes of the packet we've read in (or -1 if we don't know where
  // the packet is).
  int packet_bytes_ = -1;
};

}  // namespace bbb

#endif
