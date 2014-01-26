#ifndef BBB_BYTE_READER_H_
#define BBB_BYTE_READER_H_

#include <sys/types.h>

#include "aos/common/time.h"

namespace bbb {

class ByteReader {
public:
  // We have 64-bit ints in some of our data.
  typedef char __attribute__((aligned(8))) AlignedChar;

  // Implemented by subclasses to provide a data source
  // for these algorithms.
  // Returns the number of bytes read, -1 if there is an error in errno, or -2
  // if reading takes longer than timeout.
  virtual ssize_t ReadBytes(AlignedChar *dest, size_t max_bytes,
                            const ::aos::time::Time &timeout_time) = 0;
};

}  // namespace bbb

#endif  // BBB_BYTE_READER_H_
