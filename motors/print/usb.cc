#include "motors/print/usb.h"

#include <atomic>

#include "motors/core/kinetis.h"

namespace frc971 {
namespace motors {
namespace {

::std::atomic<teensy::AcmTty *> global_stdout{nullptr};

}  // namespace

::std::unique_ptr<PrintingImplementation> CreatePrinting(
    const PrintingParameters &parameters) {
  if (parameters.dedicated_usb) {
    return ::std::unique_ptr<PrintingImplementation>(
        new DedicatedUsbPrinting());
  }
  if (parameters.stdout_tty != nullptr) {
    return ::std::unique_ptr<PrintingImplementation>(
        new UsbPrinting(parameters.stdout_tty, parameters.debug_tty));
  }
  return ::std::unique_ptr<PrintingImplementation>(new NopPrinting());
}

extern "C" int _write(const int /*file*/, char *const ptr, const int len) {
  teensy::AcmTty *const tty = global_stdout.load(::std::memory_order_acquire);
  if (tty != nullptr) {
    return tty->Write(ptr, len);
  }
  return len;
}

UsbPrinting::UsbPrinting(teensy::AcmTty *stdout_tty, teensy::AcmTty *debug_tty)
    : stdout_tty_(stdout_tty), debug_tty_(debug_tty) {}

UsbPrinting::~UsbPrinting() {
  global_stdout.store(nullptr, ::std::memory_order_release);
}

void UsbPrinting::Initialize() {
  global_stdout.store(stdout_tty_, ::std::memory_order_release);
}

DedicatedUsbPrinting::DedicatedUsbPrinting()
    : usb_device_{0, 0x16c0, 0x0490},
      stdout_tty_{&usb_device_},
      debug_tty_{&usb_device_} {
  usb_device_.SetManufacturer("FRC 971 Spartan Robotics");
  usb_device_.SetProduct("FET12v2");
  NVIC_SET_SANE_PRIORITY(IRQ_USBOTG, 0x7);
}

DedicatedUsbPrinting::~DedicatedUsbPrinting() {
  global_stdout.store(nullptr, ::std::memory_order_release);
}

void DedicatedUsbPrinting::Initialize() {
  usb_device_.Initialize();
  global_stdout.store(&stdout_tty_, ::std::memory_order_release);
}

}  // namespace motors
}  // namespace frc971
