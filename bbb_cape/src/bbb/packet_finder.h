#ifndef BBB_CAPE_SRC_BBB_PACKET_FINDER_H_
#define BBB_CAPE_SRC_BBB_PACKET_FINDER_H_

#include <stdint.h>
#include <string.h>

#define DATA_STRUCT_NAME DataStruct
#include "cape/data_struct.h"
#undef DATA_STRUCT_NAME

namespace bbb {

class PacketFinder {
 protected:
  typedef char __attribute__((aligned(4))) AlignedChar;

 public:
  PacketFinder();
  virtual ~PacketFinder();

  // Returns true if it finds one or false if it gets an I/O error first.
  // packet must be aligned to 4 bytes.
  bool GetPacket(DataStruct *packet);
  // Implemented by subclasses to provide a data source 
  // for these algorithms.
  virtual int ReadBytes(AlignedChar *dest, size_t max_bytes) = 0;
 
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


  AlignedChar *const buf_;
  AlignedChar *const unstuffed_data_;

  // How many bytes of the packet we've read in (or -1 if we don't know where
  // the packet is).
  int packet_bytes_ = -1;
};

}  // namespace bbb

#endif
