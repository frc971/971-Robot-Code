#ifndef BBB_CRC_H_
#define BBB_CRC_H_

#include <stdint.h>

namespace cape {

// Calculates a CRC32 checksum for data. This is definitely the same one as the
// cape MCU does in hardware which seems to be the same one as Ethernet etc use.
uint32_t CalculateChecksum(uint8_t *data, size_t length);

}  // namespace cape

#endif  // BBB_CRC_H_
