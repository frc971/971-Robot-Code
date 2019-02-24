#ifndef MOTORS_PERIPHERAL_SPI_H_
#define MOTORS_PERIPHERAL_SPI_H_

#include "motors/core/kinetis.h"
#include "motors/peripheral/uart_buffer.h"
#include "motors/util.h"
#include "third_party/GSL/include/gsl/gsl"

namespace frc971 {
namespace teensy {

// Simple synchronous interface to a SPI peripheral.
class Spi {
 public:
  Spi(KINETISK_SPI_t *module, int module_clock_frequency)
      : module_(module), module_clock_frequency_(module_clock_frequency) {}
  Spi(const Spi &) = delete;
  ~Spi();
  Spi &operator=(const Spi &) = delete;

  // Currently hard-coded for slave mode with the parameters we want. In the
  // future, we should add more setters to configure things in more detail.
  void Initialize();

  // Cleras all the hardware queues.
  void ClearQueues();

  bool SpaceAvailable() const { return module_->SR & M_SPI_TFFF; }
  // Only call this if SpaceAvailable() has just returned true.
  void WriteFrame(uint32_t frame_flags) {
    module_->PUSHR = frame_flags;
    module_->SR = M_SPI_TFFF;
  }

  bool DataAvailable() const { return module_->SR & M_SPI_RFDF; }
  // Only call this if DataAvailable() has just returned true.
  uint16_t ReadFrame() {
    const uint16_t result = module_->POPR;
    module_->SR = M_SPI_RFDF;
    return result;
  }

  // Calling code must synchronize all of these.
  void EnableTransmitInterrupt() {
    rser_value_ |= M_SPI_TFFF_RE;
  }
  void DisableTransmitInterrupt() {
    rser_value_ &= ~M_SPI_TFFF_RE;
  }
  void EnableReceiveInterrupt() {
    rser_value_ |= M_SPI_RFDF_RE;
  }
  void DisableReceiveInterrupt() {
    rser_value_ &= ~M_SPI_RFDF_RE;
  }
  void FlushInterruptRequests() {
    module_->RSER = rser_value_;
  }

 private:
  KINETISK_SPI_t *const module_;
  const int module_clock_frequency_;

  // What we put in RSER.
  uint32_t rser_value_ = 0;

  // What we put in MCR.
  uint32_t mcr_value_ = 0;
};

// Interrupt-based buffered interface to a SPI peripheral.
class InterruptBufferedSpi {
 public:
  InterruptBufferedSpi(KINETISK_SPI_t *module, int module_clock_frequency)
      : spi_(module, module_clock_frequency) {}
  ~InterruptBufferedSpi();

  // Provides access to the underlying Spi wrapper for configuration. Don't do
  // anything else with this object besides configure it.
  Spi *spi() { return &spi_; }

  void Initialize();

  // Clears all of the queues, in both hardware and software.
  //
  // Note that this still leaves a to-be-transmitted byte queued.
  void ClearQueues(const DisableInterrupts &);

  // Queues up the given data for immediate writing. Blocks only if the queue
  // fills up before all of data is enqueued.
  void Write(gsl::span<const char> data, DisableInterrupts *disable_interrupts);

  // Reads currently available data.
  // Returns all the data which is currently available (possibly none);
  // buffer is where to store the result. The return value will be a subspan of
  // this.
  gsl::span<char> Read(gsl::span<char> buffer,
                       DisableInterrupts *disable_interrupts);

  // Should be called as the body of the interrupt handler.
  void HandleInterrupt(const DisableInterrupts &disable_interrupts) {
    while (ReadAndWriteFrame(true, disable_interrupts)) {}
    spi_.FlushInterruptRequests();
  }

 private:
  bool WriteFrame(bool disable_empty, const DisableInterrupts &);
  bool ReadFrame(const DisableInterrupts &);
  bool ReadAndWriteFrame(bool disable_empty,
                         const DisableInterrupts &disable_interrupts) {
    const bool read = ReadFrame(disable_interrupts);
    const bool written = WriteFrame(disable_empty, disable_interrupts);
    return read || written;
  }

  Spi spi_;
  UartBuffer<1024> transmit_buffer_, receive_buffer_;
  // The number of frames we should receive before stopping.
  int frames_to_receive_ = 0;
};

}  // namespace teensy
}  // namespace frc971

#endif  // MOTORS_PERIPHERAL_SPI_H_
