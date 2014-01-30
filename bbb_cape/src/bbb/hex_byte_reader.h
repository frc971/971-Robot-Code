#ifndef BBB_CAPE_SRC_BBB_HEX_BYTE_READER_H_
#define BBB_CAPE_SRC_BBB_HEX_BYTE_READER_H_

#include <string>

#include "aos/common/time.h"

#include "bbb/byte_io.h"

namespace bbb {

// Reads bytes from a .hex file.
class HexByteReader : public ByteReaderInterface {
 public:
  explicit HexByteReader(const ::std::string &filename);
  virtual ~HexByteReader() {}

  virtual ssize_t ReadBytes(uint8_t *dest, size_t max_bytes,
                            const ::aos::time::Time &timeout) override;

  // Returns the total number of bytes that we will eventually read out.
  unsigned int GetSize();

 private:
  void *const parser_status_;
};

}  // namespace bbb

#endif  // BBB_CAPE_SRC_BBB_HEX_BYTE_READER_H_
