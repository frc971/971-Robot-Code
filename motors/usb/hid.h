#ifndef MOTORS_USB_HID_H_
#define MOTORS_USB_HID_H_

#include <stdint.h>
#include <string.h>
#include <array>

#include "motors/usb/usb.h"
#include "motors/util.h"

namespace frc971 {
namespace teensy {

// Implements an HID class device.
// TODO(Brian): Make this way more generic.
class HidFunction final : public UsbFunction {
 public:
  HidFunction(UsbDevice *device, int report_max_size)
      : UsbFunction(device), report_max_size_(report_max_size) {
    if (report_max_size_ > kMaxReportSize) {
      __builtin_trap();
    }
  }
  ~HidFunction() override = default;

  // Sets the report descriptor. Must be called at least once.
  //
  // May only be called during setup.
  void set_report_descriptor(const ::std::string &report_descriptor) {
    report_descriptor_ = report_descriptor;
  }

  void UpdateReport(const void *data, int length,
                    const DisableInterrupts &disable_interrupts) {
    memcpy(report_tx_buffer_to_fill(disable_interrupts), data, length);
  }

 private:
  // Choose 64 for now so it always fits into a single packet.
  static constexpr int kMaxReportSize = 64;

  uint8_t *report_tx_buffer_for(EvenOdd odd) {
    return report_tx_buffers_[EvenOddIndex(odd)].data();
  }
  uint8_t *report_tx_buffer_being_sent(const DisableInterrupts &) {
    return report_tx_buffer_for(BufferStateToEmpty(tx_state_));
  }
  uint8_t *report_tx_buffer_to_fill(const DisableInterrupts &) {
    return report_tx_buffer_for(BufferStateToFill(tx_state_));
  }

  int in_endpoint_max_size() const { return report_max_size_; }

  void Initialize() override;

  SetupResponse HandleEndpoint0SetupPacket(
      const UsbDevice::SetupPacket &setup_packet) override;
  SetupResponse HandleGetDescriptor(
      const UsbDevice::SetupPacket &setup_packet) override;

  void HandleInFinished(int endpoint, BdtEntry *bdt_entry,
                        EvenOdd odd) override;

  void HandleConfigured(int endpoint) override;
  void HandleReset() override {
    tx_state_ = EndpointBufferState::kBothEmptyEvenFirst;
    device()->SetBdtEntry(in_endpoint_, Direction::kTx, EvenOdd::kEven,
                          {0, nullptr});
    device()->SetBdtEntry(in_endpoint_, Direction::kTx, EvenOdd::kOdd,
                          {0, nullptr});
    {
      DisableInterrupts disable_interrupts;
      memset(report_tx_buffers_[0].data(), 0, kMaxReportSize);
      memset(report_tx_buffers_[1].data(), 0, kMaxReportSize);
    }
  }

  ::std::array<::std::array<uint8_t, kMaxReportSize>, 2> report_tx_buffers_;
  ::std::array<uint8_t, kMaxReportSize> get_report_response_buffer_;

  // This is only manipulated with interrupts disabled.
  EndpointBufferState tx_state_;
  Data01 next_tx_toggle_;

  // Our interface number.
  int interface_;

  // The IN endpoint we send reports on.
  int in_endpoint_;

  const int report_max_size_;

  ::std::string report_descriptor_;
  UsbDescriptorList hid_descriptor_list_;
};

}  // namespace teensy
}  // namespace frc971

#endif  // MOTORS_USB_HID_H_
