#ifndef FRC971_WPILIB_SPI_RX_CLEARER_H_
#define FRC971_WPILIB_SPI_RX_CLEARER_H_

#include <stdint.h>

namespace frc971 {
namespace wpilib {

// Allows clearing the RX FIFO of the roboRIO's SPI peripheral on demand. This
// is necessary to work around a driver bug. See
// https://docs.google.com/document/d/1ANV4LtnVcku2fk84Y31pIIqrxUD_x_dYyCQgp10NB0k/edit
// for details.
class SpiRxClearer {
 public:
  SpiRxClearer();
  ~SpiRxClearer();

  // Actually clears the RX FIFO. This should be very fast. Don't do this while
  // any operations via the kernel driver are ongoing.
  void ClearRxFifo();

 private:
  // How big of a mapping we do.
  static constexpr uint32_t kMappingSize = 0x1000;

  // The physical base address of the SPI instance we're going to mess with.
  const uint32_t spi_peripheral_base_ = 0xe0006000;
  volatile uint32_t *mapping_;

  uint32_t ReadRegister(uint32_t offset) { return mapping_[offset / 4]; }
  void WriteRegister(uint32_t offset, uint32_t value) {
    mapping_[offset / 4] = value;
  }

  bool RxFifoIsEmpty() { return !(ReadRegister(4) & (1 << 4)); }
};

}  // namespace wpilib
}  // namespace frc971

#endif // FRC971_WPILIB_SPI_RX_CLEARER_H_
