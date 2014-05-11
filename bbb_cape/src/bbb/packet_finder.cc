#include "bbb/packet_finder.h"

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <algorithm>

#include "aos/common/logging/logging.h"

#include "cape/cows.h"
#include "bbb/crc.h"
#include "bbb/byte_io.h"

using ::aos::time::Time;

namespace bbb {
namespace {

// Enabling all of the debugging logs during normal operation on the BBB causes
// it to use most of the CPU when it runs into trouble.
const bool kDebugLogs = false;

}  // namespace

constexpr ::aos::time::Time PacketFinder::kDebugLogInterval;

PacketFinder::PacketFinder(ByteReaderInterface *reader, size_t packet_size)
    : reader_(reader),
      packet_size_(packet_size),
      buf_(new AlignedChar[packet_size_ + kZeros]),
      unstuffed_data_(new AlignedChar[packet_size_ - 4]) {
  CHECK((packet_size_ % 4) == 0);
}

PacketFinder::~PacketFinder() {
  delete[] buf_;
  delete[] unstuffed_data_;
}

bool PacketFinder::FindPacket(const ::Time &timeout_time) {
  // How many 0 bytes we've found at the front so far.
  int zeros_found = 0;
  while (true) {
    size_t already_read = ::std::max(0, packet_bytes_);
    size_t to_read = packet_size_ - already_read;
    if (packet_bytes_ == -1) to_read += kZeros;
    ssize_t new_bytes =
        reader_->ReadBytes((uint8_t *)(buf_ + already_read),
                           to_read, timeout_time);
    if (new_bytes < 0) {
      if (new_bytes == -1) {
        PLOG(ERROR, "ReadBytes(%p, %zd) failed",
             buf_ + already_read, to_read);
      } else if (new_bytes == -2) {
        LOG(WARNING, "timed out\n");
      } else {
        LOG(WARNING, "bad ByteReader %p returned %zd\n", reader_, new_bytes);
      }
      return false;
    }

    if (!irq_priority_increased_) {
      // Iff we're root.
      if (getuid() == 0) {
        // TODO(brians): Do this cleanly.
        int chrt_result =
            system("chrt -o 0 bash -c 'chrt -r -p 55"
                   " $(pgrep irq/89)'");
        if (chrt_result == -1) {
          LOG(FATAL, "system(chrt -r -p 55 the_irq) failed\n");
        } else if (!WIFEXITED(chrt_result) || WEXITSTATUS(chrt_result) != 0) {
          LOG(FATAL, "$(chrt -r -p 55 the_irq) failed, return value = %d\n",
              WEXITSTATUS(chrt_result));
        }
      } else {
        LOG(WARNING, "not root, so not increasing priority of the IRQ\n");
      }

      irq_priority_increased_ = true;
    }

    if (packet_bytes_ == -1) {
      for (size_t to_check = already_read; to_check < already_read + new_bytes;
           ++to_check) {
        if (buf(to_check) == 0) {
          ++zeros_found;
          if (zeros_found == kZeros) {
            packet_bytes_ = 0;
            zeros_found = 0;
            new_bytes -= to_check + 1;
            memmove(buf_, buf_ + to_check + 1, new_bytes);
            to_check = 0;
            break;
          }
        } else {
          zeros_found = 0;
        }
      }
    }
    if (packet_bytes_ != -1) {  // if we decided that these are good bytes
      packet_bytes_ += new_bytes;
      CHECK_LE(packet_bytes_, static_cast<ssize_t>(packet_size_));
      if (packet_bytes_ == static_cast<ssize_t>(packet_size_)) return true;
    }
  }
}

bool PacketFinder::ProcessPacket() {
  uint32_t unstuffed = cows_unstuff(
      reinterpret_cast<uint32_t *>(buf_), packet_size_,
      reinterpret_cast<uint32_t *>(unstuffed_data_), packet_size_ - 4);
  invalid_packet_.Print();
  bad_checksum_.Print();

  if (unstuffed == 0) {
    if (kDebugLogs) LOG(WARNING, "invalid\n");
    LOG_INTERVAL(invalid_packet_);
    return false;
  } else if (unstuffed != (packet_size_ - 4) / 4) {
    LOG(WARNING, "packet is %" PRIu32 " words instead of %zu\n",
        unstuffed, (packet_size_ - 4) / 4);
    return false;
  }

  // Make sure the checksum checks out.
  uint32_t sent_checksum;
  memcpy(&sent_checksum, unstuffed_data_ + packet_size_ - 8, 4);
  uint32_t calculated_checksum = cape::CalculateChecksum(
      reinterpret_cast<uint8_t *>(unstuffed_data_), packet_size_ - 8);
  if (sent_checksum != calculated_checksum) {
    if (kDebugLogs) {
      LOG(WARNING, "sent %" PRIx32 " not %" PRIx32 "\n", sent_checksum,
          calculated_checksum);
    }
    LOG_INTERVAL(bad_checksum_);
    return false;
  }

  return true;
}

bool PacketFinder::ReadPacket(const ::Time &timeout_time) {
  if (!FindPacket(timeout_time)) return false;

  if (!ProcessPacket()) {
    packet_bytes_ = -1;
    int zeros = 0;
    for (size_t i = 0; i < packet_size_; ++i) {
      if (buf(i) == 0) {
        ++zeros;
        if (zeros == kZeros) {
          if (kDebugLogs) LOG(INFO, "start at %zd\n", i);
          packet_bytes_ = packet_size_ - (i + 1);
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
