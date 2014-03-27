#ifndef BBB_CAPE_SRC_BBB_UART_READER_H_
#define BBB_CAPE_SRC_BBB_UART_READER_H_

#include <stdint.h>
#include <string.h>
#include <sys/select.h>

#include "aos/common/time.h"
#include "aos/common/macros.h"

#include "bbb/byte_io.h"

namespace bbb {

class UartReader : public ByteReaderWriterInterface {
 public:
  explicit UartReader(int32_t baud_rate);
  virtual ~UartReader();

  virtual ssize_t ReadBytes(
      uint8_t *dest, size_t max_bytes,
      const ::aos::time::Time &timeout_time = ::aos::time::Time(0, 0)) override;
  virtual bool WriteBytes(uint8_t *bytes, size_t number_bytes) override;

 private:
  const int fd_;

  DISALLOW_COPY_AND_ASSIGN(UartReader);
};

}  // namespace bbb

#endif
