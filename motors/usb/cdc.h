#ifndef MOTORS_USB_CDC_H_
#define MOTORS_USB_CDC_H_

#include <array>

#include "motors/usb/usb.h"
#include "motors/usb/queue.h"
#include "motors/util.h"

// CDC (Communications Device Class) is a "class standard" which takes 30 pages
// to explain that a communications device has communications and data, and they
// can come via separate interfaces or the same ones, and the data can be in a
// lot of formats. The only commonality across the "class" seems to be some
// constants and a few descriptors.

namespace frc971 {
namespace teensy {

struct CdcLineCoding {
  static constexpr uint8_t stop_bits_1() { return 0; }
  static constexpr uint8_t stop_bits_1_5() { return 1; }
  static constexpr uint8_t stop_bits_2() { return 2; }

  static constexpr uint8_t parity_none() { return 0; }
  static constexpr uint8_t parity_odd() { return 1; }
  static constexpr uint8_t parity_even() { return 2; }
  static constexpr uint8_t parity_mark() { return 3; }
  static constexpr uint8_t parity_space() { return 4; }

  // The baud rate in bits/second.
  uint32_t rate;  // dwDTERate

  uint8_t stop_bits;  // bCharFormat

  uint8_t parity;  // bParityType

  // 5, 6, 7, 8, or 16 according to the standard.
  uint8_t data_bits;  // bDataBits
} __attribute__((packed));
static_assert(sizeof(CdcLineCoding) == 7, "wrong size");

// Implements a pretty dumb serial port via CDC's ACM (Abstract Control
// Management) and Call Management functions.
class AcmTty final : public UsbFunction {
 public:
  AcmTty(UsbDevice *device) : UsbFunction(device) {}
  ~AcmTty() override = default;

  size_t Read(void *buffer, size_t buffer_size);
  size_t Write(const void *buffer, size_t buffer_size);

  bool write_queue_empty() const { return tx_queue_.empty(); }

 private:
  enum class NextEndpoint0Out {
    kNone,
    kLineCoding,
  };

  // We're going with the largest allowable sizes for full speed devices for
  // the data endpoints to maximize throughput.
  static constexpr uint16_t kDataMaxPacketSize = 64;

  // We have no information to send here, so no point allocating bus bandwidth
  // for it.
  static constexpr uint16_t kStatusMaxPacketSize = 1;

  void Initialize() override;

  SetupResponse HandleEndpoint0SetupPacket(
      const UsbDevice::SetupPacket &setup_packet) override;
  SetupResponse HandleEndpoint0OutPacket(void *data, int data_length) override;
  void HandleOutFinished(int endpoint, BdtEntry *bdt_entry) override;
  void HandleInFinished(int endpoint, BdtEntry *bdt_entry,
                        EvenOdd odd) override;
  void HandleConfigured(int endpoint) override;
  void HandleReset() override {
    // TODO(Brian): Handle data already in the buffers correctly.
    DisableInterrupts disable_interrupts;
    tx_state_ = EndpointBufferState::kBothEmptyEvenFirst;
    device()->SetBdtEntry(status_endpoint_, Direction::kTx, EvenOdd::kEven,
                          {0, nullptr});
    device()->SetBdtEntry(status_endpoint_, Direction::kTx, EvenOdd::kOdd,
                          {0, nullptr});
    device()->SetBdtEntry(data_tx_endpoint_, Direction::kTx, EvenOdd::kEven,
                          {0, nullptr});
    device()->SetBdtEntry(data_tx_endpoint_, Direction::kTx, EvenOdd::kOdd,
                          {0, nullptr});
    device()->SetBdtEntry(data_rx_endpoint_, Direction::kRx, EvenOdd::kEven,
                          {0, nullptr});
    device()->SetBdtEntry(data_rx_endpoint_, Direction::kRx, EvenOdd::kOdd,
                          {0, nullptr});
  }

  char *tx_buffer_for(EvenOdd odd) {
    return tx_buffers_[EvenOddIndex(odd)].data();
  }

  void EnqueueTxData(const DisableInterrupts &);

  ::std::array<::std::array<char, kDataMaxPacketSize>, 2> tx_buffers_,
      rx_buffers_;

  // In theory, we could sent notifications over this about things like the line
  // state, but we don't have anything to report so we pretty much just ignore
  // this.
  int status_interface_;
  int data_interface_;
  int status_endpoint_, data_tx_endpoint_, data_rx_endpoint_;

  Queue tx_queue_{1024}, rx_queue_{1024};

  NextEndpoint0Out next_endpoint0_out_ = NextEndpoint0Out::kNone;

  // This is only manipulated with interrupts disabled.
  EndpointBufferState tx_state_;
  Data01 next_tx_toggle_;

  // These are BdtEntries which we're holding onto without releasing back to the
  // hardware so that we won't receive any more data until we have space for it.
  // They are only manipulated with interrupts disabled.
  BdtEntry *first_rx_held_ = nullptr, *second_rx_held_ = nullptr;
  Data01 next_rx_toggle_;

  CdcLineCoding line_coding_{0, CdcLineCoding::stop_bits_1(),
                             CdcLineCoding::parity_none(), 8};
  CdcLineCoding line_coding_to_send_;

  uint16_t control_line_state_ = 0;
};

}  // namespace teensy
}  // namespace frc971

#endif  // MOTORS_USB_CDC_H_
