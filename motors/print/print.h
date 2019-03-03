#ifndef MOTORS_PRINT_PRINT_H_
#define MOTORS_PRINT_PRINT_H_

#include <memory>

#include "aos/containers/sized_array.h"
#include "motors/core/kinetis.h"
#include "third_party/GSL/include/gsl/gsl"

namespace frc971 {
namespace teensy {

class AcmTty;

}  // namespace teensy
namespace motors {

class PrintingImplementation {
 public:
  PrintingImplementation() = default;
  virtual ~PrintingImplementation() = default;

  PrintingImplementation(const PrintingImplementation &) = delete;
  PrintingImplementation &operator=(const PrintingImplementation &) = delete;

  virtual void Initialize() = 0;

  // Writes something directly to stdout/stderr (they are treated as the same).
  virtual int WriteStdout(gsl::span<const char> buffer) = 0;
  // Writes something to a separate debug stream. Some implementations will
  // always ignore this, and others will ignore it under some conditions.
  virtual int WriteDebug(gsl::span<const char> buffer) { return buffer.size(); }

  // Reads any characters which are available (never blocks).
  //
  // The default never returns any data.
  virtual aos::SizedArray<char, 4> ReadStdin() { return {}; }
};

// A trivial printing "implementation" which simply does nothing. This is used
// when a real implementation can't be created by CreatePrinting due to missing
// parameters.
class NopPrinting : public PrintingImplementation {
 public:
  NopPrinting() = default;
  ~NopPrinting() override = default;

  void Initialize() override {}
  int WriteStdout(gsl::span<const char> buffer) override {
    return buffer.size();
  }
};

// Contains various parameters for controlling how the printing implementation
// is initialized. Some of these are optional depending on which implementation
// is selected, while others being missing will result in some implementations
// turning into NOPs.
struct PrintingParameters {
  // This module must have its clock enabled and pinmuxing set up before calling
  // CreatePrinting.
  KINETISK_UART_t *stdout_uart_module = nullptr;
  int stdout_uart_module_clock_frequency = 0;
  int stdout_uart_baud_rate = 115200;
  int stdout_uart_status_interrupt = -1;

  // Setting this to true indicates the implementation should manage its own
  // UsbDevice. If there are other USB functions around, set stdout_tty (and
  // optionally debug_tty) instead.
  bool dedicated_usb = false;

  // If these are used, Initialize() must be called on the UsbDevice before the
  // PrintingImplementation.
  teensy::AcmTty *stdout_tty = nullptr;
  teensy::AcmTty *debug_tty = nullptr;
};

// Creates an implementation of the linked-in type. Exactly one printing
// implementation must be linked in. If all the necessary parameters aren't
// filled out, this will return a NopPrinting instance.
//
// Some implementations will work even before calling this, or calling
// Initialize() on the result. Others do require calling this before they will
// work. This must be called before enabling any interrupts or some
// implementations may deadlock.
//
// This should only be called once per program lifetime. Many implementations
// manage global resources in the returned object. The resulting object may be
// destroyed, but not while interrupts might be running. Destroying the object
// may or may not stop printing.
//
// This will not enable any interrupts. When applicable, that is deferred until
// Initialize() is called on the result.
::std::unique_ptr<PrintingImplementation> CreatePrinting(
    const PrintingParameters &parameters);

}  // namespace motors
}  // namespace frc971

#endif  // MOTORS_PRINT_PRINT_H_
