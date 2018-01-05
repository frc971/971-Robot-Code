#include "motors/usb/interrupt_out.h"

namespace frc971 {
namespace teensy {

void InterruptOut::Initialize() {
  interface_ = AddInterface();
  endpoint_ = AddEndpoint();

  SetMicrosoftDeviceInterfaceGuids("{D4FA286B-C60D-4B99-B49B-9656139F5771}");

  CreateIadDescriptor(
      /*first_interface=*/interface_,
      /*interface_count=*/1,
      /*function_class=*/vendor_specific_class(),
      /*function_subclass=*/0,
      /*function_protocol=*/0, name_);

  {
    const auto interface_descriptor = CreateDescriptor(
        interface_descriptor_length(), UsbDescriptorType::kInterface);
    interface_descriptor->AddByte(interface_);  // bInterfaceNumber
    interface_descriptor->AddByte(0);           // bAlternateSetting
    interface_descriptor->AddByte(1);           // bNumEndpoints
    interface_descriptor->AddByte(vendor_specific_class());  // bInterfaceClass
    interface_descriptor->AddByte(0x97);  // bInterfaceSubClass
    interface_descriptor->AddByte(0x97);  // bInterfaceProtocol
    interface_descriptor->AddByte(device()->AddString(name_));  // iInterface
  }

  {
    const auto endpoint_descriptor = CreateDescriptor(
        endpoint_descriptor_length(), UsbDescriptorType::kEndpoint);
    endpoint_descriptor->AddByte(endpoint_);  // bEndpointAddress
    endpoint_descriptor->AddByte(
        m_endpoint_attributes_interrupt());  // bmAttributes
    endpoint_descriptor->AddUint16(kSize);   // wMaxPacketSize
    endpoint_descriptor->AddByte(1);  // bInterval
  }
}

void InterruptOut::HandleOutFinished(int endpoint, BdtEntry *bdt_entry) {
  if (endpoint == endpoint_) {

    DisableInterrupts disable_interrupts;
    if (first_rx_held_ == nullptr) {
      first_rx_held_ = bdt_entry;
    } else {
      second_rx_held_ = bdt_entry;
    }
  }
}

int InterruptOut::ReceiveData(char *buffer) {
  DisableInterrupts disable_interrupts;
  if (first_rx_held_ == nullptr) {
    return -1;
  }

  BdtEntry *const bdt_entry = first_rx_held_;
  const size_t data_size = G_USB_BD_BC(bdt_entry->buffer_descriptor);
  memcpy(buffer, bdt_entry->address, kSize);
  dma_memory_barrier();

  first_rx_held_->buffer_descriptor = M_USB_BD_OWN | M_USB_BD_DTS |
                                      V_USB_BD_BC(kSize) |
                                      static_cast<uint32_t>(next_rx_toggle_);
  next_rx_toggle_ = Data01Inverse(next_rx_toggle_);

  first_rx_held_ = second_rx_held_;
  second_rx_held_ = nullptr;
  return data_size;
}

void InterruptOut::HandleConfigured(int endpoint) {
  if (endpoint == endpoint_) {
    device()->ConfigureEndpointFor(endpoint_, true, false, true);
    next_rx_toggle_ = Data01::kData0;
    device()->SetBdtEntry(
        endpoint_, Direction::kRx, EvenOdd::kEven,
        {M_USB_BD_OWN | M_USB_BD_DTS | V_USB_BD_BC(buffers_[0].size()),
         buffers_[0].data()});
    device()->SetBdtEntry(endpoint_, Direction::kRx, EvenOdd::kOdd,
                          {M_USB_BD_OWN | M_USB_BD_DTS |
                               V_USB_BD_BC(buffers_[1].size()) | M_USB_BD_DATA1,
                           buffers_[1].data()});
  }
}

}  // namespace teensy
}  // namespace frc971
