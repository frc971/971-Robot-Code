#include "motors/usb/cdc.h"

#include <string.h>
#include <stdint.h>

#include "motors/core/time.h"

#define CHECK(c)                             \
  do {                                       \
    if (!(c)) {                              \
      while (true) {                         \
        for (int i = 0; i < 10000000; ++i) { \
          GPIOC_PSOR = 1 << 5;               \
        }                                    \
        for (int i = 0; i < 10000000; ++i) { \
          GPIOC_PCOR = 1 << 5;               \
        }                                    \
      }                                      \
    }                                        \
  } while (false)

namespace frc971 {
namespace teensy {
namespace {

// Aka the Communications Device Class code, the Communications Class code,
// and the Communications Interface Class code.
constexpr uint8_t communications_class() { return 0x02; }
constexpr uint8_t data_interface_class() { return 0x0A; }

namespace cdc_descriptor_subtype {
constexpr uint8_t header() { return 0x00; }
constexpr uint8_t header_length() { return 5; }
constexpr uint8_t call_management() { return 0x01; }
constexpr uint8_t call_management_length() { return 5; }
constexpr uint8_t abstract_control_management() { return 0x02; }
constexpr uint8_t abstract_control_management_length() { return 4; }
constexpr uint8_t direct_line_management() { return 0x03; }
constexpr uint8_t telephone_ringer() { return 0x04; }
constexpr uint8_t telephone_call_etc() { return 0x05; }
// Can't just call this "union" because that's a keyword...
constexpr uint8_t union_function() { return 0x06; }
constexpr uint8_t union_length(int number_subordinates) {
  return 4 + number_subordinates;
}
constexpr uint8_t country_selection() { return 0x07; }
constexpr uint8_t telephone_operational_modes() { return 0x08; }
constexpr uint8_t usb_terminal() { return 0x09; }
constexpr uint8_t network_channel() { return 0x0A; }
constexpr uint8_t protocol_unit() { return 0x0B; }
constexpr uint8_t extension_unit() { return 0x0C; }
constexpr uint8_t multichannel_management() { return 0x0D; }
constexpr uint8_t capi_control() { return 0x0E; }
constexpr uint8_t ethernet_networking() { return 0x0F; }
constexpr uint8_t atm_networking() { return 0x10; }
constexpr uint8_t wireless_handset_control() { return 0x11; }
constexpr uint8_t mobile_direct_line() { return 0x12; }
constexpr uint8_t mdlm_detail() { return 0x13; }
constexpr uint8_t device_management() { return 0x14; }
constexpr uint8_t obex() { return 0x15; }
constexpr uint8_t command_set() { return 0x16; }
constexpr uint8_t command_set_detail() { return 0x17; }
constexpr uint8_t telephone_control() { return 0x18; }
constexpr uint8_t obex_service() { return 0x19; }
constexpr uint8_t ncm() { return 0x1A; }
}  // namespace cdc_descriptor_subtype

namespace communications_subclass {
constexpr uint8_t direct_line_control_model() { return 0x01; }
constexpr uint8_t abstract_control_model() { return 0x02; }
constexpr uint8_t telephone_control_model() { return 0x03; }
constexpr uint8_t multichannel_control_model() { return 0x04; }
constexpr uint8_t capi_control_model() { return 0x05; }
constexpr uint8_t ethernet_networking_control_model() { return 0x06; }
constexpr uint8_t atm_networking_control_model() { return 0x07; }
constexpr uint8_t wireless_handset_control_model() { return 0x08; }
constexpr uint8_t device_management() { return 0x09; }
constexpr uint8_t mobile_direct_line_model() { return 0x0A; }
constexpr uint8_t obex() { return 0x0B; }
constexpr uint8_t ethernet_emulation_model() { return 0x0C; }
constexpr uint8_t network_control_model() { return 0x0D; }
}  // namespace communications_subclass

namespace cdc_class_requests {
constexpr uint8_t send_encapsulated_command() { return 0x00; }
constexpr uint8_t get_encapsulated_response() { return 0x01; }
constexpr uint8_t set_comm_feature() { return 0x02; }
constexpr uint8_t get_comm_feature() { return 0x03; }
constexpr uint8_t clear_comm_feature() { return 0x04; }
constexpr uint8_t set_aux_line_state() { return 0x10; }
constexpr uint8_t set_hook_state() { return 0x11; }
constexpr uint8_t pulse_setup() { return 0x12; }
constexpr uint8_t send_pulse() { return 0x13; }
constexpr uint8_t set_pulse_time() { return 0x14; }
constexpr uint8_t ring_aux_jack() { return 0x15; }
constexpr uint8_t set_line_coding() { return 0x20; }
constexpr uint8_t get_line_coding() { return 0x21; }
constexpr uint8_t set_control_line_state() { return 0x22; }
constexpr uint8_t send_break() { return 0x23; }
constexpr uint8_t set_ringer_parms() { return 0x30; }
constexpr uint8_t get_ringer_parms() { return 0x31; }
constexpr uint8_t set_operation_parms() { return 0x32; }
constexpr uint8_t get_operation_parms() { return 0x33; }
constexpr uint8_t set_line_parms() { return 0x34; }
constexpr uint8_t get_line_parms() { return 0x35; }
constexpr uint8_t dial_digits() { return 0x36; }
constexpr uint8_t set_unit_parameter() { return 0x37; }
constexpr uint8_t get_unit_parameter() { return 0x38; }
constexpr uint8_t clear_unit_parameter() { return 0x39; }
constexpr uint8_t get_profile() { return 0x3A; }
}  // namespace cdc_class_requests

}  // namespace

void AcmTty::Initialize() {
  status_interface_ = AddInterface();
  data_interface_ = AddInterface();
  status_endpoint_ = AddEndpoint();
  data_tx_endpoint_ = AddEndpoint();
  data_rx_endpoint_ = AddEndpoint();

  CreateIadDescriptor(
      /*first_interface=*/status_interface_,
      /*interface_count=*/2,
      /*function_class=*/communications_class(),
      /*function_subclass=*/communications_subclass::abstract_control_model(),
      /*function_protocol=*/0, "UsbTty");

  {
    const auto interface_descriptor = CreateDescriptor(
        interface_descriptor_length(), UsbDescriptorType::kInterface);
    interface_descriptor->AddByte(status_interface_);  // bInterfaceNumber
    interface_descriptor->AddByte(0);  // bAlternateSetting
    interface_descriptor->AddByte(1);                       // bNumEndpoints
    interface_descriptor->AddByte(communications_class());  // bInterfaceClass
    interface_descriptor->AddByte(
        communications_subclass::
            abstract_control_model());  // bInterfaceSubClass
    interface_descriptor->AddByte(0);   // bInterfaceProtocol
    interface_descriptor->AddByte(
        device()->AddString("UsbTty.status"));  // iInterface
  }

  {
    const auto cdc_header =
        CreateDescriptor(cdc_descriptor_subtype::header_length(),
                         UsbClassDescriptorType::kInterface);
    cdc_header->AddByte(
        cdc_descriptor_subtype::header());  // bDescriptorSubtype
    cdc_header->AddUint16(0x0110);          // bcdCDC
  }

  {
    const auto call_management =
        CreateDescriptor(cdc_descriptor_subtype::call_management_length(),
                         UsbClassDescriptorType::kInterface);
    call_management->AddByte(
        cdc_descriptor_subtype::call_management());  // bDescriptorSubtype
    // We don't do call management.
    call_management->AddByte(0);  // bmCapabilities
    call_management->AddByte(data_interface_);  // bDataInterface
  }

  {
    const auto abstract_control_management = CreateDescriptor(
        cdc_descriptor_subtype::abstract_control_management_length(),
        UsbClassDescriptorType::kInterface);
    abstract_control_management->AddByte(
        cdc_descriptor_subtype::
            abstract_control_management());  // bDescriptorSubtype
    // We support:
    //   line_coding and serial_state
    //   send_break
    // We don't support:
    //   comm_feature
    //   network_notification
    abstract_control_management->AddByte(6);  // bmCapabilities
  }

  {
    const auto cdc_union_descriptor =
        CreateDescriptor(cdc_descriptor_subtype::union_length(1),
                         UsbClassDescriptorType::kInterface);
    cdc_union_descriptor->AddByte(
        cdc_descriptor_subtype::union_function());     // bDescriptorSubtype
    cdc_union_descriptor->AddByte(status_interface_);  // bMasterInterface
    cdc_union_descriptor->AddByte(data_interface_);  // bSlaveInterface
  }

  {
    const auto endpoint_descriptor = CreateDescriptor(
        endpoint_descriptor_length(), UsbDescriptorType::kEndpoint);
    endpoint_descriptor->AddByte(status_endpoint_ |
                                 m_endpoint_address_in());  // bEndpointAddress
    endpoint_descriptor->AddByte(
        m_endpoint_attributes_interrupt());                // bmAttributes
    endpoint_descriptor->AddUint16(kStatusMaxPacketSize);  // wMaxPacketSize
    // Set it to the max because we have nothing to send, so no point using bus
    // bandwidth asking.
    endpoint_descriptor->AddByte(255);  // bInterval
  }

  {
    const auto interface_descriptor = CreateDescriptor(
        interface_descriptor_length(), UsbDescriptorType::kInterface);
    interface_descriptor->AddByte(data_interface_);         // bInterfaceNumber
    interface_descriptor->AddByte(0);                       // bAlternateSetting
    interface_descriptor->AddByte(2);                       // bNumEndpoints
    interface_descriptor->AddByte(data_interface_class());  // bInterfaceClass
    interface_descriptor->AddByte(0);  // bInterfaceSubClass
    interface_descriptor->AddByte(0);  // bInterfaceProtocol
    interface_descriptor->AddByte(
        device()->AddString("UsbTty.data"));  // iInterface
  }

  // Kernel seems to think tx and rx belong in the other order, but it deals
  // with either one. Everybody's examples seem to use this order, and the
  // kernel deals with it, so going to go with this.
  {
    const auto endpoint_descriptor = CreateDescriptor(
        endpoint_descriptor_length(), UsbDescriptorType::kEndpoint);
    endpoint_descriptor->AddByte(data_rx_endpoint_);  // bEndpointAddress
    endpoint_descriptor->AddByte(m_endpoint_attributes_bulk());  // bmAttributes
    endpoint_descriptor->AddUint16(kDataMaxPacketSize);  // wMaxPacketSize
    endpoint_descriptor->AddByte(1);                     // bInterval
  }

  {
    const auto endpoint_descriptor = CreateDescriptor(
        endpoint_descriptor_length(), UsbDescriptorType::kEndpoint);
    endpoint_descriptor->AddByte(data_tx_endpoint_ |
                                 m_endpoint_address_in());  // bEndpointAddress
    endpoint_descriptor->AddByte(m_endpoint_attributes_bulk());  // bmAttributes
    endpoint_descriptor->AddUint16(kDataMaxPacketSize);  // wMaxPacketSize
    endpoint_descriptor->AddByte(1);                     // bInterval
  }
}

// We deliberately don't implement SendEncapsulatedCommand and
// GetEncapsulatedResponse because there doesn't seem to be any reason for
// anybody to send those to us, despite them being "required".
UsbFunction::SetupResponse AcmTty::HandleEndpoint0SetupPacket(
    const UsbDevice::SetupPacket &setup_packet) {
  if (G_SETUP_REQUEST_TYPE_TYPE(setup_packet.request_type) !=
      SetupRequestType::kClass) {
    return SetupResponse::kIgnored;
  }
  if (G_SETUP_REQUEST_TYPE_RECIPIENT(setup_packet.request_type) !=
      standard_setup_recipients::kInterface) {
    return SetupResponse::kIgnored;
  }
  if (setup_packet.index != status_interface_) {
    return SetupResponse::kIgnored;
  }
  const bool in = setup_packet.request_type & M_SETUP_REQUEST_TYPE_IN;
  switch (setup_packet.request) {
    case cdc_class_requests::set_line_coding():
      if (in || setup_packet.value != 0 ||
          setup_packet.length != sizeof(line_coding_)) {
        return SetupResponse::kStall;
      }
      next_endpoint0_out_ = NextEndpoint0Out::kLineCoding;
      return SetupResponse::kHandled;

    case cdc_class_requests::get_line_coding():
      if (!in || setup_packet.value != 0 ||
          setup_packet.length != sizeof(line_coding_)) {
        return SetupResponse::kStall;
      }
      line_coding_to_send_ = line_coding_;
      device()->QueueEndpoint0Data(
          reinterpret_cast<const char *>(&line_coding_to_send_),
          setup_packet.length);
      return SetupResponse::kHandled;

    case cdc_class_requests::set_control_line_state():
      if (in || setup_packet.length != 0) {
        return SetupResponse::kStall;
      }
      control_line_state_ = setup_packet.value;
      device()->SendEmptyEndpoint0Packet();
      return SetupResponse::kHandled;

    case cdc_class_requests::send_break():
      if (in || setup_packet.length != 0) {
        return SetupResponse::kStall;
      }
      // TODO(Brian): setup_packet.value is the length of the break in ms.
      // 0xFFFF means keep sending break until receiving another one.
      device()->SendEmptyEndpoint0Packet();
      return SetupResponse::kHandled;
  }
  return SetupResponse::kStall;
}

UsbFunction::SetupResponse AcmTty::HandleEndpoint0OutPacket(void *data,
                                                            int data_length) {
  switch (next_endpoint0_out_) {
    case NextEndpoint0Out::kNone:
      return SetupResponse::kIgnored;

    case NextEndpoint0Out::kLineCoding:
      next_endpoint0_out_ = NextEndpoint0Out::kNone;
      if (data_length != sizeof(line_coding_)) {
        return SetupResponse::kStall;
      }
      memcpy(&line_coding_, data, data_length);
      device()->SendEmptyEndpoint0Packet();
      // If we're supposed to reboot, then do it.
      if (line_coding_.rate == UINT32_C(0x97101678)) {
        // Delay for a bit so the empty IN packet gets back to the host.
        delay(5);
        __asm__ __volatile__("bkpt");
      }
      return SetupResponse::kHandled;

    default:
      return SetupResponse::kIgnored;
  }
}

void AcmTty::HandleOutFinished(int endpoint, BdtEntry *bdt_entry) {
  if (endpoint == data_rx_endpoint_) {
    const size_t data_size = G_USB_BD_BC(bdt_entry->buffer_descriptor);
    CHECK(rx_queue_.Write(static_cast<char *>(bdt_entry->address), data_size) ==
          data_size);
    dma_memory_barrier();

    DisableInterrupts disable_interrupts;
    // If we don't have space to handle it, don't return the entry so we'll
    // ask the host to wait to transmit more data.
    if (rx_queue_.space_available() >= kDataMaxPacketSize * 2) {
      bdt_entry->buffer_descriptor = M_USB_BD_OWN | M_USB_BD_DTS |
                                     V_USB_BD_BC(kDataMaxPacketSize) |
                                     static_cast<uint32_t>(next_rx_toggle_);
      next_rx_toggle_ = Data01Inverse(next_rx_toggle_);
    } else {
      if (first_rx_held_ == nullptr) {
        first_rx_held_ = bdt_entry;
      } else {
        CHECK(second_rx_held_ == nullptr);
        second_rx_held_ = bdt_entry;
      }
    }
  }
}

void AcmTty::HandleInFinished(int endpoint, BdtEntry * /*bdt_entry*/,
                              EvenOdd odd) {
  if (endpoint == data_tx_endpoint_) {
    DisableInterrupts disable_interrupts;
    CHECK(odd == BufferStateToEmpty(tx_state_));
    tx_state_ = BufferStateAfterEmpty(tx_state_);
    EnqueueTxData(disable_interrupts);
  }
}

void AcmTty::HandleConfigured(int endpoint) {
  // TODO(Brian): Handle data already in the buffers correctly.
  if (endpoint == status_endpoint_) {
    device()->ConfigureEndpointFor(status_endpoint_, false, true, true);
  } else if (endpoint == data_tx_endpoint_) {
    device()->ConfigureEndpointFor(data_tx_endpoint_, false, true, true);
    DisableInterrupts disable_interrupts;
    next_tx_toggle_ = Data01::kData0;
    EnqueueTxData(disable_interrupts);
  } else if (endpoint == data_rx_endpoint_) {
    device()->ConfigureEndpointFor(data_rx_endpoint_, true, false, true);
    next_rx_toggle_ = Data01::kData0;
    device()->SetBdtEntry(
        data_rx_endpoint_, Direction::kRx, EvenOdd::kEven,
        {M_USB_BD_OWN | M_USB_BD_DTS | V_USB_BD_BC(rx_buffers_[0].size()),
         rx_buffers_[0].data()});
    device()->SetBdtEntry(
        data_rx_endpoint_, Direction::kRx, EvenOdd::kOdd,
        {M_USB_BD_OWN | M_USB_BD_DTS | V_USB_BD_BC(rx_buffers_[1].size()) |
             M_USB_BD_DATA1,
         rx_buffers_[1].data()});
  }
}

size_t AcmTty::Read(void *buffer, size_t buffer_size) {
  const size_t r = rx_queue_.Read(static_cast<char *>(buffer), buffer_size);

  DisableInterrupts disable_interrupts;
  if (rx_queue_.space_available() >= kDataMaxPacketSize * 2) {
    if (first_rx_held_ != nullptr) {
      first_rx_held_->buffer_descriptor =
          M_USB_BD_OWN | M_USB_BD_DTS | V_USB_BD_BC(kDataMaxPacketSize) |
          static_cast<uint32_t>(next_rx_toggle_);
      next_rx_toggle_ = Data01Inverse(next_rx_toggle_);
    }
    if (second_rx_held_ != nullptr) {
      second_rx_held_->buffer_descriptor =
          M_USB_BD_OWN | M_USB_BD_DTS | V_USB_BD_BC(kDataMaxPacketSize) |
          static_cast<uint32_t>(next_rx_toggle_);
      next_rx_toggle_ = Data01Inverse(next_rx_toggle_);
    }
    first_rx_held_ = second_rx_held_ = nullptr;
  }

  return r;
}

size_t AcmTty::Write(const void *buffer, size_t buffer_size) {
  const size_t r =
      tx_queue_.Write(static_cast<const char *>(buffer), buffer_size);
  DisableInterrupts disable_interrupts;
  EnqueueTxData(disable_interrupts);
  return r;
}

// TODO(Brian): Could this critical section be broken up per buffer we fill?
void AcmTty::EnqueueTxData(const DisableInterrupts &) {
  while (BufferStateHasEmpty(tx_state_) && !tx_queue_.empty()) {
    const EvenOdd next_tx_odd = BufferStateToFill(tx_state_);
    const size_t buffer_size =
        tx_queue_.Read(tx_buffer_for(next_tx_odd), kDataMaxPacketSize);
    CHECK(buffer_size > 0);
    dma_memory_barrier();
    device()->SetBdtEntry(
        data_tx_endpoint_, Direction::kTx, next_tx_odd,
        {M_USB_BD_OWN | M_USB_BD_DTS | V_USB_BD_BC(buffer_size) |
             static_cast<uint32_t>(next_tx_toggle_),
         tx_buffer_for(next_tx_odd)});
    tx_state_ = BufferStateAfterFill(tx_state_);
    next_tx_toggle_ = Data01Inverse(next_tx_toggle_);
  }
}

}  // namespace teensy
}  // namespace frc971
