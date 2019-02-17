#ifndef MOTORS_PERIPHERAL_UART_H_
#define MOTORS_PERIPHERAL_UART_H_

#include "aos/containers/sized_array.h"
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
  void Write(gsl::span<const char> data, const DisableInterrupts &) {
    DoWrite(data);
  }

  // Returns all the data which is currently available.
  aos::SizedArray<char, 4> Read(const DisableInterrupts &) {
    return DoRead();
  }

  bool SpaceAvailable() const { return module_->S1 & M_UART_TDRE; }
  // Only call this if SpaceAvailable() has just returned true.
  void WriteCharacter(char c) { module_->D = c; }

  bool DataAvailable() const { return module_->S1 & M_UART_RDRF; }
  // Only call this if DataAvailable() has just returned true.
  char ReadCharacter() { return module_->D; }

  // TODO(Brian): These APIs for enabling/disabling interrupts aren't quite
  // right. Redo them some time. Some issues:
  //   * They get called during initialization/destruction time, which means
  //     interrupts don't really need to be disabled because everything is
  //     singlethreaded.
  //   * Often, several C2 modifications are made in a single
  //     interrupts-disabled section. These could be batched to reduce
  //     peripheral writes. Sometimes, no modifications are made at all, in
  //     which case there doesn't even need to be a single write.

  void EnableTransmitInterrupt(const DisableInterrupts &) {
    c2_value_ |= M_UART_TIE;
    module_->C2 = c2_value_;
  }

  void DisableTransmitInterrupt(const DisableInterrupts &) {
    DoDisableTransmitInterrupt();
  }

  void EnableReceiveInterrupt(const DisableInterrupts &) {
    c2_value_ |= M_UART_RIE;
    module_->C2 = c2_value_;
  }

  void DisableReceiveInterrupt(const DisableInterrupts &) {
    DoDisableReceiveInterrupt();
  }

 private:
  void DoDisableTransmitInterrupt() {
    c2_value_ &= ~M_UART_TIE;
    module_->C2 = c2_value_;
  }
  void DoDisableReceiveInterrupt() {
    c2_value_ &= ~M_UART_RIE;
    module_->C2 = c2_value_;
  }

  void DoWrite(gsl::span<const char> data);
  aos::SizedArray<char, 4> DoRead();

  KINETISK_UART_t *const module_;
  const int module_clock_frequency_;
  // What we put in C2 except TE.
  uint8_t c2_value_;

  int tx_fifo_size_, rx_fifo_size_;
};

// Interrupt-based buffered interface to a UART.
// TODO(Brian): Move DisableInterrupts calls up to the caller of this.
class InterruptBufferedUart {
 public:
  InterruptBufferedUart(KINETISK_UART_t *module, int module_clock_frequency)
      : uart_(module, module_clock_frequency) {}
  ~InterruptBufferedUart();

  void Initialize(int baud_rate);

  // Queues up the given data for immediate writing. Blocks only if the queue
  // fills up before all of data is enqueued.
  void Write(gsl::span<const char> data);

  // Reads currently available data.
  // Returns all the data which is currently available (possibly none);
  // buffer is where to store the result. The return value will be a subspan of
  // this.
  gsl::span<char> Read(gsl::span<char> buffer);

  // Should be called as the body of the interrupt handler.
  void HandleInterrupt(const DisableInterrupts &disable_interrupts) {
    WriteCharacters(true, disable_interrupts);
    ReadCharacters(disable_interrupts);
  }

 private:
  void WriteCharacters(bool disable_empty, const DisableInterrupts &);
  void ReadCharacters(const DisableInterrupts &);

  Uart uart_;
  UartBuffer<1024> transmit_buffer_, receive_buffer_;
};

}  // namespace teensy
}  // namespace frc971

#endif  // MOTORS_PERIPHERAL_UART_H_
