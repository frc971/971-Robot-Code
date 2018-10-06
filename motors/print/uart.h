#ifndef MOTORS_PRINT_UART_H_
#define MOTORS_PRINT_UART_H_

#include "motors/peripheral/uart.h"
#include "motors/print/print.h"

namespace frc971 {
namespace motors {

// A printing implementation using a hardware UART. This has a reasonably sized
// buffer in memory and uses interrupts to keep the hardware busy. It could
// support DMA too in the future.
class UartPrinting : public PrintingImplementation {
 public:
  // All required parameters must be filled out.
  UartPrinting(const PrintingParameters &parameters);
  ~UartPrinting() override;

  void Initialize() override;

  int WriteStdout(gsl::span<const char> buffer) override;

 private:
  teensy::InterruptBufferedUart stdout_uart_;
  const int stdout_status_interrupt_;
};

// Could easily create a subclass of UartPrinting that also implements
// WriteDebug on a second UART, and conditionally instantiate that.

}  // namespace motors
}  // namespace frc971

#endif  // MOTORS_PRINT_UART_H_
