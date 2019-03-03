#ifndef MOTORS_PRINT_USB_H_
#define MOTORS_PRINT_USB_H_

#include "motors/print/print.h"
#include "motors/usb/cdc.h"
#include "motors/usb/usb.h"

namespace frc971 {
namespace motors {

// A printing implementation which uses externally-created functions. The stdout
// one is required, while the debug one is optional.
class UsbPrinting final : public PrintingImplementation {
 public:
  UsbPrinting(teensy::AcmTty *stdout_tty, teensy::AcmTty *debug_tty);
  ~UsbPrinting() override;

  void Initialize() override;

  int WriteStdout(gsl::span<const char> buffer) override {
    return stdout_tty_->Write(buffer.data(), buffer.size());
  }

  int WriteDebug(gsl::span<const char> buffer) override {
    if (debug_tty_ == nullptr) {
      return buffer.size();
    }
    return debug_tty_->Write(buffer.data(), buffer.size());
  }

  aos::SizedArray<char, 4> ReadStdin() override {
    aos::SizedArray<char, 4> result;
    result.set_size(stdout_tty_->Read(result.data(), result.max_size()));
    return result;
  }

 private:
  teensy::AcmTty *const stdout_tty_;
  teensy::AcmTty *const debug_tty_;
};

// A printing implementation which creates its own UsbDevice and functions, and
// manages their lifecycle.
class DedicatedUsbPrinting final : public PrintingImplementation {
 public:
  DedicatedUsbPrinting();
  ~DedicatedUsbPrinting() override;

  void Initialize() override;

  int WriteStdout(gsl::span<const char> buffer) override {
    return stdout_tty_.Write(buffer.data(), buffer.size());
  }

  int WriteDebug(gsl::span<const char> buffer) override {
    return debug_tty_.Write(buffer.data(), buffer.size());
  }

  aos::SizedArray<char, 4> ReadStdin() override {
    aos::SizedArray<char, 4> result;
    result.set_size(stdout_tty_.Read(result.data(), result.max_size()));
    return result;
  }

 private:
  teensy::UsbDevice usb_device_;
  teensy::AcmTty stdout_tty_;
  teensy::AcmTty debug_tty_;
};

}  // namespace motors
}  // namespace frc971

#endif  // MOTORS_PRINT_USB_H_
