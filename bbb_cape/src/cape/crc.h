#ifndef CAPE_CRC_H_
#define CAPE_CRC_H_

#include <stdint.h>
#include <sys/types.h>

void crc_init(void);

// The second argument is the number of words to checksum, NOT the number of
// bytes.
uint32_t crc_calculate(uint32_t *restrict data, size_t words);

#endif  // CAPE_CRC_H_
