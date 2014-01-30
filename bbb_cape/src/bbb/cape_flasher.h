#ifndef BBB_CAPE_SRC_BBB_CAPE_FLASHER_H_
#define BBB_CAPE_SRC_BBB_CAPE_FLASHER_H_

#include <string>

#include "aos/common/macros.h"

namespace bbb {

class ByteReaderInterface;
class ByteReaderWriterInterface;

// Talks to the custom bootloader on the cape MCU. Expects for the cape to be
// freshly booted into the bootloader when initialized.
class CapeFlasher {
 public:
  explicit CapeFlasher(ByteReaderWriterInterface *uart);

  // Downloads a file.
  // file should give the raw bytes to download, starting at the start of the
  // main code. All reads will be done with a timeout of 0. Stops on the first
  // timeout.
  void Download(ByteReaderInterface *file);

 private:
  void WriteBuffer(uint8_t *buffer, size_t size);

  ByteReaderWriterInterface *const uart_;

  DISALLOW_COPY_AND_ASSIGN(CapeFlasher);
};

}  // namespace bbb

#endif  // BBB_CAPE_SRC_BBB_CAPE_FLASHER_H_
