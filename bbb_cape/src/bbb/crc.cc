#include "bbb/crc.h"

#include "aos/common/once.h"

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

uint32_t CalculateChecksum(uint8_t *data, size_t length) {
  static ::aos::Once<const uint32_t> table_once(GenerateTable);
  const uint32_t *const table = table_once.Get();

  uint32_t r = 0xFFFFFFFF;

  for (size_t i = 0; i < length; ++i) {
    r = (r << 8) ^ table[(r >> 24) ^ data[i]];
  }

  return ~r;
}

}  // namespace cape
