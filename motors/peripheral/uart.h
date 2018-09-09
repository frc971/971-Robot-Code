#ifndef MOTORS_PERIPHERAL_UART_H_
#define MOTORS_PERIPHERAL_UART_H_

#include "motors/core/kinetis.h"
#include "motors/peripheral/uart_buffer.h"
#include "motors/util.h"
#include "third_party/GSL/include/gsl/gsl"

namespace frc971 {
namespace teensy {

// Simple synchronous interface to a UART.
class Uart {
 public:
  Uart(KINETISK_UART_t *module, int module_clock_frequency)
      : module_(module), module_clock_frequency_(module_clock_frequency) {}
  Uart(const Uart &) = delete;
  ~Uart();
  Uart &operator=(const Uart &) = delete;

  void Initialize(int baud_rate);

  // Blocks until all of the data is at least queued.
  void Write(gsl::span<char> data, const DisableInterrupts &) { DoWrite(data); }

  bool SpaceAvailable() const { return module_->S1 & M_UART_TDRE; }
  // Only call this if SpaceAvailable() has just returned true.
  void WriteCharacter(char c) { module_->D = c; }

  void EnableTransmitInterrupt() {
    c2_value_ |= M_UART_TIE;
    module_->C2 = c2_value_;
  }

  void DisableTransmitInterrupt() {
    c2_value_ &= ~M_UART_TIE;
    module_->C2 = c2_value_;
  }

 private:
  void DoWrite(gsl::span<char> data);

  KINETISK_UART_t *const module_;
  const int module_clock_frequency_;
  // What we put in C2 except TE.
  uint8_t c2_value_;

  int tx_fifo_size_, rx_fifo_size_;
};

// Interrupt-based buffered interface to a UART.
class InterruptBufferedUart {
 public:
  InterruptBufferedUart(KINETISK_UART_t *module, int module_clock_frequency)
      : uart_(module, module_clock_frequency) {}

  void Initialize(int baud_rate);

  void Write(gsl::span<char> data);

  // Should be called as the body of the interrupt handler.
  void HandleInterrupt(const DisableInterrupts &disable_interrupts) {
    WriteCharacters(true, disable_interrupts);
  }

 private:
  void WriteCharacters(bool disable_empty, const DisableInterrupts &);

  Uart uart_;
  UartBuffer<1024> buffer_;

  bool interrupt_enabled_ = false;
};

}  // namespace teensy
}  // namespace frc971

#endif  // MOTORS_PERIPHERAL_UART_H_
