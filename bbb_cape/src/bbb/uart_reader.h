#ifndef BBB_CAPE_SRC_BBB_UART_READER_H_
#define BBB_CAPE_SRC_BBB_UART_READER_H_

#include <stdint.h>
#include <string.h>
#include <sys/select.h>

#include "aos/common/time.h"

#include "bbb/byte_reader.h"

namespace bbb {

class UartReader : public ByteReader {
 public:
  UartReader(int32_t baud_rate);
  virtual ~UartReader();

  virtual ssize_t ReadBytes(AlignedChar *dest, size_t max_bytes,
                            const ::aos::time::Time &timeout) override;

 private:
  const int fd_;
  // Gets initialized to only contain fd_.
  fd_set fd_set_;
};

}  // namespace bbb

#endif
