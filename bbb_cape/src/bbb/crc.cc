#include "bbb/crc.h"

#include <assert.h>
#include <errno.h>
#include <string.h>

#include "aos/common/once.h"
#include "aos/common/logging/logging.h"

#include "bbb/byte_io.h"

// There are various implementations that look a lot like this scattered around
// the internet.

namespace cape {
namespace {

const uint32_t kPolynomial = 0x04c11db7;

const uint32_t *GenerateTable() {
  static uint32_t table[256];

  for (int i = 0; i < 256; ++i) {
    uint32_t c = i << 24;
    for (int j = 8; j > 0; --j) {
      c = c & 0x80000000 ? ((c << 1) ^ kPolynomial) : (c << 1);
    }
    table[i] = c;
  }

  return table;
}

}  // namespace

uint32_t CalculateChecksum(const uint8_t *data, size_t length,
                           uint32_t initial) {
  assert((length % 4) == 0);

  static ::aos::Once<const uint32_t> table_once(GenerateTable);
  const uint32_t *const table = table_once.Get();

  uint32_t r = initial;

  for (size_t i = 0; i < (length / 4); ++i) {
    for (int ii = 3; ii >= 0; --ii) {
      r = (r << 8) ^ table[(r >> 24) ^ data[i * 4 + ii]];
    }
  }

  return r;
}

uint32_t CalculateChecksum(::bbb::ByteReaderInterface *reader) {
  uint8_t buffer[256];
  int remainder_bytes = 0;
  uint32_t checksum = 0xFFFFFFFF;
  while (true) {
    ssize_t read = reader->ReadBytes(&buffer[remainder_bytes],
                                     sizeof(buffer) - remainder_bytes);
    if (read == -2) return checksum;
    if (read == -1) {
      LOG(FATAL, "reader %p failed to read with %d: %s\n",
          reader, errno, strerror(errno));
    }
    size_t checksum_bytes = (read / 4) * 4;
    checksum = CalculateChecksum(buffer, checksum_bytes, checksum);
    remainder_bytes = read - checksum_bytes;
    memmove(buffer, &buffer[checksum_bytes], remainder_bytes);
  }
}

}  // namespace cape
