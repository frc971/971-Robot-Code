#ifndef BBB_CAPE_SRC_BBB_PACKET_FINDER_H_
#define BBB_CAPE_SRC_BBB_PACKET_FINDER_H_

#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#define DATA_STRUCT_NAME DataStruct
#include "cape/data_struct.h"
#undef DATA_STRUCT_NAME

namespace bbb {

class PacketFinder {
 public:
  typedef char __attribute__((aligned(8))) AlignedChar;
  
  PacketFinder();
  virtual ~PacketFinder();

  // Returns true if it finds one or false if it gets an I/O error first.
  // Packet must be aligned to 4 bytes.
  bool ReadPacket();

  // Gets a reference to the received packet.
  // The most recent call to ReadPacket() must have returned true or the data
  // pointed to is undefined.
  template <typename T>
  const T *get_packet() {
    static_assert(alignof(T) <= alignof(*unstuffed_data_),
                  "We need to align our data better.");
    /*static_assert(sizeof(T) <= PACKET_SIZE - 8,
                  "We aren't getting that much data.");*/
    return reinterpret_cast<const T *>(unstuffed_data_);
  }

 private:
  // Implemented by subclasses to provide a data source 
  // for these algorithms.
  // Returns the number of bytes read or -1 if there is an error in errno.
  virtual ssize_t ReadBytes(AlignedChar *dest, size_t max_bytes) = 0;

 protected:
  const uint32_t kPacketSize = DATA_STRUCT_SEND_SIZE - 4; 

  // Reads bytes until there are 4 zeros and then fills up buf_.
  // Returns true if it finds one or false if it gets an I/O error first or the
  // packet is invalid in some way.
  bool FindPacket();
  // Processes a packet currently in buf_ and leaves the result in
  // unstuffed_data_.
  // Returns true if it succeeds or false if there was something wrong with the
  // data.
  bool ProcessPacket();

  AlignedChar *const buf_;
  AlignedChar *const unstuffed_data_;

  // How many bytes of the packet we've read in (or -1 if we don't know where
  // the packet is).
  int packet_bytes_ = -1;
};

}  // namespace bbb

#endif
