#include "frc971/wpilib/spi_rx_clearer.h"

#include <fcntl.h>
#include <inttypes.h>
#include <sys/mman.h>
#include <unistd.h>

#include "aos/logging/logging.h"

namespace frc971 {
namespace wpilib {

SpiRxClearer::SpiRxClearer() {
  const int fd = AOS_PCHECK(open("/dev/mem", O_RDWR));
  void *const mmap_result = mmap(nullptr, kMappingSize, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, fd, spi_peripheral_base_);
  if (mmap_result == MAP_FAILED) {
    AOS_PLOG(FATAL, "mmap the SPI peripheral from /dev/mem failed\n");
  }
  AOS_PCHECK(close(fd));
  mapping_ = static_cast<volatile uint32_t *>(mmap_result);
}

SpiRxClearer::~SpiRxClearer() {
  AOS_PCHECK(munmap(const_cast<uint32_t *>(mapping_), kMappingSize));
}

void SpiRxClearer::ClearRxFifo() {
  // The FIFO has 128 entries, so if we do that many reads and don't get it all,
  // something weird is going on.
  for (int i = 0; i < 128; ++i) {
    if (RxFifoIsEmpty()) {
      // If there's nothing more, we're done.
      return;
    }
    // Read the next byte.
    AOS_LOG(DEBUG, "Read from RX FIFO: %" PRIx32 "\n", ReadRegister(0x20));
  }
  AOS_LOG(FATAL, "Failed to clear the RX FIFO\n");
}

}  // namespace wpilib
}  // namespace frc971
