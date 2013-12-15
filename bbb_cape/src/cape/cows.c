#include "cows.h"

#include <limits.h>

// This implementation is based on
// <http://www.jacquesf.com/2011/03/consistent-overhead-byte-stuffing/>.

uint32_t cows_stuff(const void *restrict source_in, size_t source_length,
                    void *restrict destination_in) {
  const uint32_t *restrict source = (const uint32_t *)source_in;
  uint32_t *restrict destination = (uint32_t *)destination_in;
  size_t source_index = 0;
  size_t destination_index = 1;
  size_t code_index = 0;
  uint32_t code = 1;

  while (source_index < ((source_length - 1) / 4) + 1) {
    if (source[source_index] == 0) {
      destination[code_index] = code;
      code = 1;
      code_index = destination_index++;
      ++source_index;
    } else {
      destination[destination_index++] = source[source_index++];
      ++code;
      if (code == UINT32_MAX) {
        destination[code_index] = code;
        code = 1;
        code_index = destination_index++;
      }
    }
  }
  destination[code_index] = code;
  return destination_index;
}

uint32_t cows_unstuff(const uint32_t *restrict source, size_t source_length,
                      uint32_t *restrict destination) {
  size_t source_index = 0;
  size_t destination_index = 0;
  uint32_t code;

  while (source_index < ((source_length - 1) / 4) + 1) {
    code = source[source_index];
    if (source_index + code > source_length && code != 1) {
      return 0;
    }

    ++source_index;

    for (uint32_t i = 1; i < code; ++i) {
      destination[destination_index++] = source[source_index++];
    }
    if (code != UINT32_MAX && source_index != source_length) {
      destination[destination_index++] = 0;
    }
  }
  return destination_index;
}
