#ifndef BBB_CRC_H_
#define BBB_CRC_H_

#include <string.h>
#include <stdint.h>

namespace bbb {

class ByteReaderInterface;

}  // namespace bbb
namespace cape {

// Calculates a CRC32 checksum for data. This is definitely the same one as the
// cape MCU does in hardware which seems to be the same one as Ethernet etc use.
// length is the number of bytes of data to read. It must be a multiple of 4.
// initial can be a previous return value to continue the same checksum over
// more data.
uint32_t CalculateChecksum(const uint8_t *data, size_t length,
                           uint32_t initial = 0xFFFFFFFF);
// Reads all data out of reader and does a checksum over all of it in reasonably
// sized pieces. Does all of the reads with a timeout of 0. Stops on the first
// timeout.
uint32_t CalculateChecksum(::bbb::ByteReaderInterface *reader);

}  // namespace cape

#endif  // BBB_CRC_H_
