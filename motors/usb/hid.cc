#include "motors/usb/hid.h"

namespace frc971 {
namespace teensy {
namespace {

constexpr uint8_t hid_class() { return 0x03; }

namespace hid_class_requests {
constexpr uint8_t get_report() { return 0x01; }
constexpr uint8_t get_idle() { return 0x02; }
constexpr uint8_t get_protocol() { return 0x03; }
constexpr uint8_t set_report() { return 0x09; }
constexpr uint8_t set_idle() { return 0x0a; }
constexpr uint8_t set_protcol() { return 0x0b; }
}  // namespace hid_class_requests

}  // namespace

void HidFunction::Initialize() {
  interface_ = AddInterface();
  in_endpoint_ = AddEndpoint();

  CreateIadDescriptor(
      /*first_interface=*/interface_,
      /*interface_count=*/1,
      /*function_class=*/hid_class(),
      /*function_subclass=*/0,
      /*function_protocol=*/0, "HidIad");

  {
    const auto interface_descriptor = CreateDescriptor(
        interface_descriptor_length(), UsbDescriptorType::kInterface);
    interface_descriptor->AddByte(interface_);   // bInterfaceNumber
    interface_descriptor->AddByte(0);            // bAlternateSetting
    interface_descriptor->AddByte(1);            // bNumEndpoints
    interface_descriptor->AddByte(hid_class());  // bInterfaceClass
    interface_descriptor->AddByte(0);            // bInterfaceSubClass
    interface_descriptor->AddByte(0);            // bInterfaceProtocol
    interface_descriptor->AddByte(device()->AddString("Hid"));  // iInterface
  }

  {
    const auto hid_descriptor = hid_descriptor_list_.CreateDescriptor(
        9, UsbClassDescriptorType::kHidHid);
    hid_descriptor->AddUint16(0x0110);  // bcdHID
    hid_descriptor->AddByte(0);         // bCountryCode
    hid_descriptor->AddByte(1);         // bNumDescriptors
    hid_descriptor->AddByte(static_cast<uint8_t>(
        UsbClassDescriptorType::kHidReport));              // bDescriptorType
    hid_descriptor->AddUint16(report_descriptor_.size());  // wDescriptorLength
  }
  AddPremadeDescriptor(hid_descriptor_list_);

  {
    const auto endpoint_descriptor = CreateDescriptor(
        endpoint_descriptor_length(), UsbDescriptorType::kEndpoint);
    endpoint_descriptor->AddByte(in_endpoint_ |
                                 m_endpoint_address_in());  // bEndpointAddress
    endpoint_descriptor->AddByte(
        m_endpoint_attributes_interrupt());                  // bmAttributes
    endpoint_descriptor->AddUint16(in_endpoint_max_size());  // wMaxPacketSize
    endpoint_descriptor->AddByte(0x8);                        // bInterval
  }
}

UsbFunction::SetupResponse HidFunction::HandleEndpoint0SetupPacket(
    const UsbDevice::SetupPacket &setup_packet) {
  if (G_SETUP_REQUEST_TYPE_TYPE(setup_packet.request_type) !=
      SetupRequestType::kClass) {
    return SetupResponse::kIgnored;
  }
  if (G_SETUP_REQUEST_TYPE_RECIPIENT(setup_packet.request_type) !=
      standard_setup_recipients::kInterface) {
    return SetupResponse::kIgnored;
  }
  if (setup_packet.index != interface_) {
    return SetupResponse::kIgnored;
  }
  const bool in = setup_packet.request_type & M_SETUP_REQUEST_TYPE_IN;
  switch (setup_packet.request) {
    case hid_class_requests::get_report():
      if (!in) {
        return SetupResponse::kStall;
      }
      // If it's not requesting the only Input report, no idea what the host
      // wants so stall.
      if (setup_packet.value != 0x0100) {
        return SetupResponse::kStall;
      }
      {
        DisableInterrupts disable_interrupts;
        memcpy(get_report_response_buffer_.data(),
               report_tx_buffer_being_sent(disable_interrupts), kMaxReportSize);
      }
      device()->QueueEndpoint0Data(
          reinterpret_cast<const char *>(get_report_response_buffer_.data()),
          ::std::min<uint16_t>(setup_packet.length, report_max_size_));
      return SetupResponse::kHandled;

    case hid_class_requests::set_idle():
      // Minimum implementation to make the host stack happy.
      if (in) {
        return SetupResponse::kStall;
      }
      device()->SendEmptyEndpoint0Packet();
      return SetupResponse::kHandled;

      // TODO(Brian): Should we actually implement the idle stuff?

    default:
      return SetupResponse::kStall;
  }
}

UsbFunction::SetupResponse HidFunction::HandleGetDescriptor(
    const UsbDevice::SetupPacket &setup_packet) {
  const uint8_t recipient =
      G_SETUP_REQUEST_TYPE_RECIPIENT(setup_packet.request_type);
  if (recipient != standard_setup_recipients::kInterface) {
    return SetupResponse::kIgnored;
  }

  const uint8_t descriptor_type = (setup_packet.value >> 8) & 0xFF;
  if (G_DESCRIPTOR_TYPE_TYPE(descriptor_type) !=
      standard_descriptor_type_types::kClass) {
    return SetupResponse::kIgnored;
  }
  if (setup_packet.index != interface_) {
    return SetupResponse::kIgnored;
  }

  const uint8_t descriptor_index = setup_packet.value & 0xFF;
  switch (descriptor_type) {
    case static_cast<uint8_t>(UsbClassDescriptorType::kHidHid):
      if (descriptor_index != 0) {
        return SetupResponse::kStall;
      }
      device()->QueueEndpoint0Data(
          hid_descriptor_list_.GetData(),
          ::std::min<int>(setup_packet.length,
                          hid_descriptor_list_.CurrentSize()));
      return SetupResponse::kHandled;

    case static_cast<uint8_t>(UsbClassDescriptorType::kHidReport):
      if (descriptor_index != 0) {
        return SetupResponse::kStall;
      }
      device()->QueueEndpoint0Data(
          report_descriptor_.data(),
          ::std::min<int>(setup_packet.length, report_descriptor_.size()));
      return SetupResponse::kHandled;

    case static_cast<uint8_t>(UsbClassDescriptorType::kHidPhysical):
      static constexpr char kNoPhysicalDescriptors[] = {0, 0, 0};
      device()->QueueEndpoint0Data(
          kNoPhysicalDescriptors,
          ::std::min<int>(setup_packet.length, sizeof(kNoPhysicalDescriptors)));
      return SetupResponse::kHandled;
  }
  return SetupResponse::kStall;
}

void HidFunction::HandleInFinished(int endpoint, BdtEntry * /*bdt_entry*/,
                                   EvenOdd odd) {
  if (endpoint == in_endpoint_) {
    DisableInterrupts disable_interrupts;
    if (odd != BufferStateToEmpty(tx_state_)) {
      __builtin_trap();
    }

    // Copy the current one into the just-sent buffer.
    memcpy(report_tx_buffer_being_sent(disable_interrupts),
           report_tx_buffer_to_fill(disable_interrupts), kMaxReportSize);

    dma_memory_barrier();
    device()->SetBdtEntry(
        in_endpoint_, Direction::kTx, BufferStateToFill(tx_state_),
        {M_USB_BD_OWN | M_USB_BD_DTS | V_USB_BD_BC(report_max_size_) |
             static_cast<uint32_t>(next_tx_toggle_),
         report_tx_buffer_to_fill(disable_interrupts)});

    // Advance the state to indicate we've swapped buffers.
    tx_state_ = BufferStateAfterFill(BufferStateAfterEmpty(tx_state_));
    next_tx_toggle_ = Data01Inverse(next_tx_toggle_);
  }
}

void HidFunction::HandleConfigured(int endpoint) {
  if (endpoint == in_endpoint_) {
    device()->ConfigureEndpointFor(in_endpoint_, false, true, true);
    DisableInterrupts disable_interrupts;
    next_tx_toggle_ = Data01::kData0;

    EvenOdd to_fill;
    if (BufferStateHasFull(tx_state_)) {
      to_fill = BufferStateToEmpty(tx_state_);
    } else {
      to_fill = BufferStateToFill(tx_state_);
      tx_state_ = BufferStateAfterFill(tx_state_);
    }

    dma_memory_barrier();
    device()->SetBdtEntry(
        in_endpoint_, Direction::kTx, to_fill,
        {M_USB_BD_OWN | M_USB_BD_DTS | V_USB_BD_BC(report_max_size_) |
             static_cast<uint32_t>(next_tx_toggle_),
         report_tx_buffer_to_fill(disable_interrupts)});
    next_tx_toggle_ = Data01Inverse(next_tx_toggle_);
  }
}

}  // namespace teensy
}  // namespace frc971
