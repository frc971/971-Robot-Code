#include "bbb/packet_finder.h"

#include <errno.h>
#include <inttypes.h>

#include <algorithm>

#include "aos/common/logging/logging.h"
#include "bbb_cape/src/cape/cows.h"
#include "bbb/crc.h"

#define PACKET_SIZE (DATA_STRUCT_SEND_SIZE - 4)

namespace bbb {

PacketFinder::PacketFinder()
    : buf_(new AlignedChar[PACKET_SIZE]),
      unstuffed_data_(new AlignedChar[PACKET_SIZE - 4]) {
  static_assert((PACKET_SIZE % 4) == 0,
                "We can't do checksums of lengths that aren't multiples of 4.");
}

PacketFinder::~PacketFinder() {
  delete buf_;
  delete unstuffed_data_;
}

// TODO(brians): Figure out why this (sometimes) gets confused right after
// flashing the cape.
bool PacketFinder::FindPacket() {
  // How many 0 bytes we've found at the front so far.
  int zeros_found = 0;
  while (true) {
    size_t already_read = ::std::max(0, packet_bytes_);
    ssize_t new_bytes =
        ReadBytes(buf_ + already_read, PACKET_SIZE - already_read);
    if (new_bytes < 0) {
      if (errno == EINTR) continue;
      LOG(ERROR, "ReadBytes(%p, %zd) failed with %d: %s\n",
          buf_ + already_read, PACKET_SIZE - already_read,
          errno, strerror(errno));
      return false;
    }

    if (packet_bytes_ == -1) {
      for (size_t to_check = already_read; to_check < already_read + new_bytes;
           ++to_check) {
        if (buf_[to_check] == 0) {
          ++zeros_found;
          if (zeros_found == 4) {
            packet_bytes_ = 0;
            zeros_found = 0;
            new_bytes -= to_check + 1;
            memmove(buf_, buf_ + to_check + 1, new_bytes);
            to_check = 0;
          }
        } else {
          zeros_found = 0;
        }
      }
    }
    if (packet_bytes_ != -1) {  // if we decided that these are good bytes
      packet_bytes_ += new_bytes;
      if (packet_bytes_ == PACKET_SIZE) return true;
    }
  }
}

bool PacketFinder::ProcessPacket() {
  uint32_t unstuffed = cows_unstuff(
      reinterpret_cast<uint32_t *>(buf_), PACKET_SIZE,
      reinterpret_cast<uint32_t *>(unstuffed_data_), PACKET_SIZE - 4);
  if (unstuffed == 0) {
    LOG(WARNING, "invalid packet\n");
    return false;
  } else if (unstuffed != (PACKET_SIZE - 4) / 4) {
    LOG(WARNING, "packet is %" PRIu32 " words instead of %" PRIu32 "\n",
        unstuffed, (PACKET_SIZE - 4) / 4);
    return false;
  }

  // Make sure the checksum checks out.
  uint32_t sent_checksum;
  memcpy(&sent_checksum, unstuffed_data_ + PACKET_SIZE - 8, 4);
  uint32_t calculated_checksum = cape::CalculateChecksum(
      reinterpret_cast<uint8_t *>(unstuffed_data_), PACKET_SIZE - 8);
  if (sent_checksum != calculated_checksum) {
    LOG(WARNING, "sent checksum: %" PRIx32 " vs calculated: %" PRIx32"\n",
        sent_checksum, calculated_checksum);
    return false;
  }

  return true;
}

bool PacketFinder::ReadPacket() {
  if (!FindPacket()) return false;

  if (!ProcessPacket()) {
    packet_bytes_ = -1;
    int zeros = 0;
    for (int i = 0; i < PACKET_SIZE; ++i) {
      if (buf_[i] == 0) {
        ++zeros;
        if (zeros == 4) {
          LOG(INFO, "found another packet start at %d\n", i);
          packet_bytes_ = PACKET_SIZE - (i + 1);
          memmove(buf_, buf_ + i + 1, packet_bytes_);
          return false;
        }
      } else {
        zeros = 0;
      }
    }
    return false;
  } else {
    packet_bytes_ = -1;
  }

  return true;
}

}  // namespace bbb
