#ifndef BBB_BYTE_READER_H_
#define BBB_BYTE_READER_H_

#include <sys/types.h>

#include "aos/common/time.h"

namespace bbb {

class ByteReaderInterface {
 public:
  virtual ~ByteReaderInterface() {}

  // Implemented by subclasses to provide a data source
  // for these algorithms.
  // Returns the number of bytes read, -1 if there is an error in errno, or -2
  // if reading takes longer than timeout.
  virtual ssize_t ReadBytes(
      uint8_t *dest, size_t max_bytes,
      const ::aos::time::Time &timeout = ::aos::time::Time(0, 0)) = 0;
};

class ByteWriterInterface {
 public:
  virtual ~ByteWriterInterface() {}

  // Implemented by subclasses to actually write the data somewhere.
  // Returns true if it succeeds or false if it fails and there is an error in
  // errno.
  virtual bool WriteBytes(uint8_t *bytes, size_t number_bytes) = 0;
};

class ByteReaderWriterInterface : public ByteReaderInterface,
                                  public ByteWriterInterface {};

class ByteReaderAndWriter : public ByteReaderWriterInterface {
 public:
  ByteReaderAndWriter(ByteReaderInterface *reader, ByteWriterInterface *writer)
      : reader_(reader), writer_(writer) {}

  ByteReaderInterface *reader() { return reader_; }
  ByteWriterInterface *writer() { return writer_; }

 private:
  ByteReaderInterface *const reader_;
  ByteWriterInterface *const writer_;
};

}  // namespace bbb

#endif  // BBB_BYTE_READER_H_
