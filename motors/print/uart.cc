#include "motors/print/uart.h"

#include "motors/core/kinetis.h"

#include <atomic>

namespace frc971 {
namespace motors {
namespace {

::std::atomic<teensy::InterruptBufferedUart *> global_stdout{nullptr};

}  // namespace

::std::unique_ptr<PrintingImplementation> CreatePrinting(
    const PrintingParameters &parameters) {
  if (parameters.stdout_uart_module == nullptr) {
    return ::std::unique_ptr<PrintingImplementation>(new NopPrinting());
  }
  if (parameters.stdout_uart_module_clock_frequency == 0) {
    return ::std::unique_ptr<PrintingImplementation>(new NopPrinting());
  }
  if (parameters.stdout_uart_status_interrupt < 0) {
    return ::std::unique_ptr<PrintingImplementation>(new NopPrinting());
  }
  return ::std::unique_ptr<PrintingImplementation>(
      new UartPrinting(parameters));
}

extern "C" void uart0_status_isr(void) {
  teensy::InterruptBufferedUart *const tty =
      global_stdout.load(::std::memory_order_relaxed);
  DisableInterrupts disable_interrupts;
  tty->HandleInterrupt(disable_interrupts);
}

UartPrinting::UartPrinting(const PrintingParameters &parameters)
    : stdout_uart_{parameters.stdout_uart_module,
                   parameters.stdout_uart_module_clock_frequency},
      stdout_status_interrupt_(parameters.stdout_uart_status_interrupt) {
  stdout_uart_.Initialize(parameters.stdout_uart_baud_rate);
}

UartPrinting::~UartPrinting() {
  NVIC_DISABLE_IRQ(stdout_status_interrupt_);
  global_stdout.store(nullptr, ::std::memory_order_release);
}

void UartPrinting::Initialize() {
  global_stdout.store(&stdout_uart_, ::std::memory_order_release);
  NVIC_ENABLE_IRQ(stdout_status_interrupt_);
}

int UartPrinting::WriteStdout(gsl::span<const char> buffer) {
  stdout_uart_.Write(buffer);
  return buffer.size();
}

extern "C" int _write(const int /*file*/, char *const ptr, const int len) {
  teensy::InterruptBufferedUart *const tty =
      global_stdout.load(::std::memory_order_acquire);
  if (tty != nullptr) {
    tty->Write(gsl::make_span(ptr, len));
  }
  return len;
}

}  // namespace motors
}  // namespace frc971
