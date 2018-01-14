#ifndef MOTORS_USB_INTERRUPT_OUT_H_
#define MOTORS_USB_INTERRUPT_OUT_H_

#include "motors/usb/usb.h"
#include "motors/util.h"

#include <array>
#include <string>

namespace frc971 {
namespace teensy {

// A simple function that just has an interrupt out endpoint and exposes the
// data received.
class InterruptOut final : public UsbFunction {
 public:
  static constexpr size_t kSize = 64;

  InterruptOut(UsbDevice *device, const ::std::string &name)
      : UsbFunction(device), name_(name) {}
  ~InterruptOut() override = default;

  // Copies the next packet into buffer.
  // buffer must have kSize of space available.
  // Returns the size of the packet, or -1 if there isn't one.
  int ReceiveData(char *buffer);

 private:
  void Initialize() override;

  void HandleOutFinished(int endpoint, BdtEntry *bdt_entry) override;
  void HandleConfigured(int endpoint) override;
  void HandleReset() override {
    device()->SetBdtEntry(endpoint_, Direction::kRx, EvenOdd::kEven,
                          {0, nullptr});
    device()->SetBdtEntry(endpoint_, Direction::kRx, EvenOdd::kOdd,
                          {0, nullptr});
  }

  ::std::string MicrosoftExtendedCompatibleId() override {
    ::std::string result = "WINUSB";
    result.resize(16);
    return result;
  }

  ::std::array<::std::array<char, kSize>, 2> buffers_;

  int interface_;
  int endpoint_;

  // These are BdtEntries which we're holding onto until the data is copied out.
  // This also has the advantage of avoiding any more data being sent until
  // we're ready.
  // They are only manipulated with interrupts disabled.
  BdtEntry *first_rx_held_ = nullptr, *second_rx_held_ = nullptr;
  Data01 next_rx_toggle_;

  const ::std::string name_;
};

}  // namespace teensy
}  // namespace frc971

#endif  // MOTORS_USB_INTERRUPT_OUT_H_
