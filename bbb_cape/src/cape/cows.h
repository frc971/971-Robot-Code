#ifndef CAPE_COWS_H_
#define CAPE_COWS_H_

#include <sys/types.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// This file implements something very similar to Consistent Overhead Byte
// Stuffing <http://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing>. It
// uses that algorithm except with 4-byte chunks instead of individual bytes
// because that's more efficient on 32-bit processors. I'm calling it Consistent
// Overhead Word Stuffing.

// source_length will be rounded up a multiple of 4. That many bytes of source
// will be read.
// destination must have at least
// ([source_length rounded up to a multiple of 4] / (2^32 - 1) rounded up) * 4
// more bytes than source_length available.
// source and destination both have to be 4-byte aligned.
// Returns the total number of words written (not necessarily the maximum given
// in the above description of destination).
uint32_t cows_stuff(const void *__restrict__ source, size_t source_length,
                    void *__restrict__ destination);

// source_length will be rounded up a multiple of 4. That many bytes of source
// will be read.
// source and destination both have to be 4-byte aligned.
// Returns the total number of words written to destination or 0 for error.
// Possible errors are trying to unstuff more data than is available in source
// or trying to write more than destination_length bytes out.
uint32_t cows_unstuff(const uint32_t *__restrict__ source, size_t source_length,
                      uint32_t *__restrict__ destination,
                      size_t destination_length);

#ifdef __cplusplus
} // extern C
#endif

#endif  // CAPE_COWS_H_
