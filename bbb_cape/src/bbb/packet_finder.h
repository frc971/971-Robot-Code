#ifndef BBB_CAPE_SRC_BBB_PACKET_FINDER_H_
#define BBB_CAPE_SRC_BBB_PACKET_FINDER_H_

#include <stdint.h>
#include <string.h>

#include "aos/common/logging/logging.h"

#include "bbb/byte_reader.h"

#define DATA_STRUCT_NAME DataStruct
#include "cape/data_struct.h"
#undef DATA_STRUCT_NAME

namespace bbb {

class PacketFinder {
 public:
  // *reader has to stay alive for the entire lifetime of this object but this
  // object does not take ownership.
  explicit PacketFinder(ByteReader *reader, size_t packet_size);
  ~PacketFinder();

  // Returns true if it succeeds or false if it gets an I/O error first.
  bool ReadPacket();

  // Gets a reference to the received packet.
  // The most recent call to ReadPacket() must have returned true or the data
  // pointed to is undefined.
  template <typename T>
  const T *get_packet() {
    static_assert(alignof(T) <= alignof(*unstuffed_data_),
                  "We need to align our data better.");
    CHECK(sizeof(T) <= packet_size_ - 8);
    return reinterpret_cast<const T *>(unstuffed_data_);
  }

 private:
  typedef ByteReader::AlignedChar AlignedChar;

  // Reads bytes until there are 4 zeros and then fills up buf_.
  // Returns true if it finds one or false if it gets an I/O error first or the
  // packet is invalid in some way.
  bool FindPacket();

  // Processes a packet currently in buf_ and leaves the result in
  // unstuffed_data_.
  // Returns true if it succeeds or false if there was something wrong with the
  // data.
  bool ProcessPacket();

  ByteReader *const reader_;
  const size_t packet_size_;

  AlignedChar *const buf_;
  AlignedChar *const unstuffed_data_;

  // How many bytes of the packet we've read in (or -1 if we don't know where
  // the packet is).
  int packet_bytes_ = -1;
};

}  // namespace bbb

#endif
