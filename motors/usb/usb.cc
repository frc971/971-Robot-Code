#include "motors/usb/usb.h"

#include <string.h>

#include <map>

#include "motors/util.h"

namespace frc971 {
namespace teensy {
namespace {

// The mask of interrupts we care about.
constexpr uint32_t usb_enabled_interrupts() {
  // Deliberately not turning the sleep interrupt on here because we just
  // want to ignore that anyways.
  return USB_INTEN_TOKDNEEN | USB_INTEN_SOFTOKEN | USB_INTEN_ERROREN |
         USB_INTEN_USBRSTEN;
}

// The names of all the standard setup requests which come in on endpoint 0.
namespace standard_setup_requests {
constexpr int kGetStatus = 0;
constexpr int kClearFeature = 1;
constexpr int kSetFeature = 3;
constexpr int kSetAddress = 5;
constexpr int kGetDescriptor = 6;
constexpr int kSetDescriptor = 7;
constexpr int kGetConfiguration = 8;
constexpr int kSetConfiguration = 9;
constexpr int kGetInterface = 10;
constexpr int kSetInterface = 11;
constexpr int kSynchFrame = 12;
}  // namespace standard_setup_requests

// The names of the standard feature selectors.
namespace standard_feature_selectors {
constexpr int kDeviceRemoteWakeup = 1;
constexpr int kEndpointHalt = 0;
constexpr int kTestMode = 2;
}  // namespace standard_feature_selectors

// The names of all the PIDs (Packet IDs) from the USB standard. Note that this
// USB hardware doesn't expose most of them, especially in device mode.
enum class UsbPid {
  kOut = 0x1,
  kIn = 0x9,
  kSof = 0x5,
  kSetup = 0xD,
  kData0 = 0x3,
  kData1 = 0xB,
  kData2 = 0x7,
  kMData = 0xF,
  kAck = 0x2,
  kNak = 0xA,
  kStall = 0xE,
  kNYet = 0x6,
  kPre = 0xC,
  kErr = 0xC,
  kSplit = 0x8,
  kPing = 0x4,
  kReserved = 0x0,
};

// The device class for using IADs.
constexpr uint8_t iad_device_class() { return 0xEF; }
// The device subclass for using IADs.
constexpr uint8_t iad_device_subclass() { return 0x02; }
// The device protocol for using IADs.
constexpr uint8_t iad_device_protocol() { return 0x01; }

// The Microsoft "vendor code" we're going to use. It's pretty arbitrary (just
// has to not be some other request we want to use).
constexpr uint8_t microsoft_vendor_code() { return 0x67; }

// The total number of endpoints supported by this hardware.
constexpr int number_endpoints() { return 16; }

__attribute__((aligned(512))) BdtEntry
    usb0_buffer_descriptor_table[number_endpoints() * 2 /* rx/tx */ *
                                 2 /* even/odd */];

// Returns the specified BDT entry.
BdtEntry *MutableBdtEntry(int endpoint, Direction direction, EvenOdd odd) {
  return &usb0_buffer_descriptor_table[static_cast<uint32_t>(endpoint << 2) |
                                       static_cast<uint32_t>(direction) |
                                       static_cast<uint32_t>(odd)];
}

// Returns the BDT entry corresponding to a USBx_STAT value.
BdtEntry *MutableBdtEntryFromStat(uint8_t stat) {
  return &usb0_buffer_descriptor_table[static_cast<uint32_t>(stat) >> 2];
}

// A pointer to the object we're going to ask to handle interrupts.
UsbDevice *volatile global_usb0_device = nullptr;

}  // namespace

constexpr int UsbDevice::kEndpoint0MaxSize;

UsbDevice::UsbDevice(int index, uint16_t vendor_id, uint16_t product_id)
    : index_(index) {
  // TODO(Brian): Pass index_ into all the register access macros. Also sort out
  // how to deal with it for the interrupts.
  assert(index == 0);

  assert(global_usb0_device == nullptr);
  global_usb0_device = this;

  // Endpoint 0 isn't a normal endpoint, so it doesn't show up in here.
  endpoint_mapping_.push_back(nullptr);

  // Set up the "String Descriptor Zero, Specifying Languages Supported by the
  // Device" (aka english_us_code() only).
  strings_.emplace_back(4, '\0');
  strings_.back()[0] = 4;
  strings_.back()[1] = static_cast<uint8_t>(UsbDescriptorType::kString);
  strings_.back()[2] = english_us_code() & 0xFF;
  strings_.back()[3] = (english_us_code() >> 8) & 0xFF;

  device_descriptor_ =
      device_descriptor_list_.CreateDescriptor(18, UsbDescriptorType::kDevice);
  device_descriptor_->AddUint16(0x0200);  // bcdUSB
  device_descriptor_->AddByte(iad_device_class());  // bDeviceClass
  device_descriptor_->AddByte(iad_device_subclass());  // bDeviceSubClass
  device_descriptor_->AddByte(iad_device_protocol());  // bDeviceProtocol
  device_descriptor_->AddByte(kEndpoint0MaxSize);  // bMaxPacketSize0
  device_descriptor_->AddUint16(vendor_id);  // idVendor
  device_descriptor_->AddUint16(product_id);  // idProduct
  // Increment this whenever you need Windows boxes to actually pay attention to
  // changes.
  device_descriptor_->AddUint16(25);  // bcdDevice
  // We might overwrite these string descriptor indices later if we get strings
  // to put there.
  device_descriptor_->AddByte(0);  // iManufacturer
  device_descriptor_->AddByte(0);  // iProduct
  device_descriptor_->AddByte(0);  // iSerialNumber
  device_descriptor_->AddByte(1);  // bNumConfigurations

  config_descriptor_ = config_descriptor_list_.CreateDescriptor(
      9, UsbDescriptorType::kConfiguration);
}

UsbDevice::~UsbDevice() {
  NVIC_DISABLE_IRQ(IRQ_USBOTG);
  dma_memory_barrier();
  assert(global_usb0_device == this);
  global_usb0_device = nullptr;
}

void UsbDevice::Initialize() {
  assert(!is_set_up_);

  for (UsbFunction *function : functions_) {
    function->Initialize();
  }

  {
    const uint32_t length = 16 + 24 * functions_.size();
    microsoft_extended_id_descriptor_.resize(length);
    int index = 0;

    // dwLength
    microsoft_extended_id_descriptor_[index++] = length & 0xFF;
    microsoft_extended_id_descriptor_[index++] = (length >> UINT32_C(8)) & 0xFF;
    microsoft_extended_id_descriptor_[index++] =
        (length >> UINT32_C(16)) & 0xFF;
    microsoft_extended_id_descriptor_[index++] =
        (length >> UINT32_C(24)) & 0xFF;

    // bcdVersion
    microsoft_extended_id_descriptor_[index++] = 0x00;
    microsoft_extended_id_descriptor_[index++] = 0x01;

    // wIndex
    microsoft_extended_id_descriptor_[index++] =
        microsoft_feature_descriptors::kExtendedCompatibilityId;
    microsoft_extended_id_descriptor_[index++] = 0;

    // bCount
    microsoft_extended_id_descriptor_[index++] = functions_.size();

    // Reserved
    index += 7;

    for (UsbFunction *function : functions_) {
      // bFirstInterfaceNumber
      microsoft_extended_id_descriptor_[index++] = function->first_interface_;

      // Reserved
      index++;

      // compatibleID and subCompatibleID
      microsoft_extended_id_descriptor_.replace(
          index, 16, function->MicrosoftExtendedCompatibleId());
      index += 16;

      // Reserved
      index += 6;
    }

    assert(index == length);
  }

  config_descriptor_->AddUint16(
      config_descriptor_list_.CurrentSize());              // wTotalLength
  config_descriptor_->AddByte(interface_mapping_.size());  // bNumInterfaces
  config_descriptor_->AddByte(1);  // bConfigurationValue
  // Doesn't seem to be much point naming our one and only configuration.
  config_descriptor_->AddByte(0);  // iConfiguration
  config_descriptor_->AddByte((1 << 7) /* Reserved */ |
                              (1 << 6) /* Self-powered */);  // bmAttribute
  config_descriptor_->AddByte(2 /* 4mA */);  // bMaxPower

  device_descriptor_.reset();
  config_descriptor_.reset();
  device_descriptor_list_.CheckFinished();
  config_descriptor_list_.CheckFinished();
  is_set_up_ = true;

  // Make sure all the buffer descriptors are clear.
  for (int i = 0; i < number_endpoints(); ++i) {
    for (Direction direction : {Direction::kTx, Direction::kRx}) {
      for (EvenOdd odd : {EvenOdd::kOdd, EvenOdd::kEven}) {
        MutableBdtEntry(i, direction, odd)->buffer_descriptor = 0;
        MutableBdtEntry(i, direction, odd)->address = nullptr;
      }
    }
  }
  dma_memory_barrier();

  // The other startup code handles getting the incoming 48MHz clock running.
  SIM_SCGC4 |= SIM_SCGC4_USBOTG;
  MPU_RGDAAC0 |= 0x03000000;

  // Reset it.
  USB0_USBTRC0 = USB_USBTRC_USBRESET;
  // TRM says to wait "two USB clock cycles", so assume that's at 48MHz and then
  // round up, being pessimistic in assuming each read from the peripheral is
  // only a single core clock. This wildly overapproximates how long we need to
  // wait, but whatever.
  for (int i = 0; i < ((F_CPU / 48000000) + 1) * 2; ++i) {
    while ((USB0_USBTRC0 & USB_USBTRC_USBRESET) != 0) {
    }
  }

  USB0_BDTPAGE1 =
      reinterpret_cast<uintptr_t>(&usb0_buffer_descriptor_table[0]) >> 8;
  USB0_BDTPAGE2 =
      reinterpret_cast<uintptr_t>(&usb0_buffer_descriptor_table[0]) >> 16;
  USB0_BDTPAGE3 =
      reinterpret_cast<uintptr_t>(&usb0_buffer_descriptor_table[0]) >> 24;

  // The Quick Reference User Guide says to clear all the interrupts.
  ClearInterrupts();
  USB0_OTGISTAT = USB_OTGISTAT_ONEMSEC | USB_OTGISTAT_LINE_STATE_CHG;

  // Now enable the module.
  USB0_CTL = USB_CTL_USBENSOFEN;

  // Un-suspend the transceiver and disable weak pulldowns.
  USB0_USBCTRL = 0;
  // And enable the D+ pullup which indicates we're a full-speed device.
  USB0_CONTROL = USB_CONTROL_DPPULLUPNONOTG;

  // Enable the reset interrupt (which is the first one we care about).
  USB0_INTEN = USB_INTEN_USBRSTEN;

  dma_memory_barrier();
  NVIC_ENABLE_IRQ(IRQ_USBOTG);
}

void usb_isr(void) {
  UsbDevice *const usb0_device = global_usb0_device;
  if (usb0_device == nullptr) {
    NVIC_DISABLE_IRQ(IRQ_USBOTG);
  } else {
    usb0_device->HandleInterrupt();
  }
}

void UsbDevice::ClearInterrupts() {
  USB0_ISTAT = USB_ISTAT_ATTACH | USB_ISTAT_RESUME | USB_ISTAT_SLEEP |
               USB_ISTAT_TOKDNE | USB_ISTAT_SOFTOK | USB_ISTAT_ERROR |
               USB_ISTAT_USBRST;
  USB0_ERRSTAT = USB_ERRSTAT_BTSERR | USB_ERRSTAT_DMAERR | USB_ERRSTAT_BTOERR |
                 USB_ERRSTAT_DFN8 | USB_ERRSTAT_CRC16 | USB_ERRSTAT_CRC5EOF |
                 USB_ERRSTAT_PIDERR;
}

void UsbDevice::HandleInterrupt() {
  while (true) {
    const uint32_t status = USB0_ISTAT;
    if ((status & usb_enabled_interrupts()) == 0) {
      return;
    }

    // If we just got a start-of-frame token, then ask all the functions what to
    // do.
    if (status & USB_ISTAT_SOFTOK) {
      // TODO(Brian): Actually ask the functions, maybe only if we're
      // configured.
      USB0_ISTAT = USB_ISTAT_SOFTOK;
    }

    // If we just finished processing a token.
    if (status & USB_ISTAT_TOKDNE) {
      const uint8_t stat = USB0_STAT;
      USB0_ISTAT = USB_ISTAT_TOKDNE;
      const int endpoint = G_USB_STAT_ENDP(stat);

      if (endpoint == 0) {
        HandleEndpoint0Token(stat);
      } else {
        BdtEntry *const bdt_entry = MutableBdtEntryFromStat(stat);
        const UsbPid pid = G_USB_BD_PID(bdt_entry->buffer_descriptor);
        UsbFunction *const function = endpoint_mapping_[endpoint];
        if (function == nullptr) {
          // Should never happen, so stall if we do get here somehow.
          StallEndpoint(endpoint);
        } else {
          switch (pid) {
            case UsbPid::kOut:
              function->HandleOutFinished(endpoint, bdt_entry);
              break;

            case UsbPid::kIn:
              function->HandleInFinished(
                  endpoint, bdt_entry,
                  (stat & M_USB_STAT_ODD) ? EvenOdd::kOdd : EvenOdd::kEven);
              break;

            case UsbPid::kSetup:
            default:
              // Should never happen, so stall if we do get here somehow.
              StallEndpoint(endpoint);
              break;
          }
        }
      }
    }

    if (status & USB_ISTAT_USBRST) {
      // Use DATA0 for all endpoints.
      USB0_CTL = USB_CTL_ODDRST;
      endpoint0_tx_odd_ = EvenOdd::kEven;
      endpoint0_tx_toggle_ = Data01::kData0;

      for (UsbFunction *function : functions_) {
        function->HandleReset();
      }

      MutableBdtEntry(0, Direction::kRx, EvenOdd::kEven)->buffer_descriptor =
          M_USB_BD_OWN | M_USB_BD_DTS | V_USB_BD_BC(kEndpoint0MaxSize);
      MutableBdtEntry(0, Direction::kRx, EvenOdd::kEven)->address =
          &endpoint0_receive_buffer_[0][0];

      MutableBdtEntry(0, Direction::kRx, EvenOdd::kOdd)->buffer_descriptor =
          M_USB_BD_OWN | M_USB_BD_DTS | V_USB_BD_BC(kEndpoint0MaxSize);
      MutableBdtEntry(0, Direction::kRx, EvenOdd::kOdd)->address =
          &endpoint0_receive_buffer_[1][0];

      MutableBdtEntry(0, Direction::kTx, EvenOdd::kEven)->buffer_descriptor = 0;
      MutableBdtEntry(0, Direction::kTx, EvenOdd::kOdd)->buffer_descriptor = 0;

      USB0_ENDPT0 = USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN | USB_ENDPT_EPHSHK;

      ClearInterrupts();

      // Set the address to 0 for enumeration.
      USB0_ADDR = 0;
      new_address_ = 0;

      endpoint0_data_ = nullptr;
      endpoint0_data_left_ = 0;

      USB0_INTEN = usb_enabled_interrupts();
      USB0_ERREN = USB_ERREN_BTSERREN | USB_ERREN_DMAERREN |
                   USB_ERREN_BTOERREN | USB_ERREN_DFN8EN | USB_ERREN_CRC16EN |
                   USB_ERREN_CRC5EOFEN | USB_ERREN_PIDERREN;

      // Start the peripheral going.
      dma_memory_barrier();
      USB0_CTL = USB_CTL_USBENSOFEN;

      continue;
    }

    // TODO(Brian): Handle errors more intelligently.
    if (status & USB_ISTAT_ERROR) {
      const uint8_t error = USB0_ERRSTAT;
      USB0_ERRSTAT = error;
      USB0_ISTAT = USB_ISTAT_ERROR;
    }
  }
}

void UsbDevice::HandleEndpoint0Token(const uint8_t stat) {
  BdtEntry *const bdt_entry = MutableBdtEntryFromStat(stat);
  const UsbPid pid = G_USB_BD_PID(bdt_entry->buffer_descriptor);
  switch (pid) {
    case UsbPid::kSetup:
      // Unstall it if it was previously stalled.
      USB0_ENDPT0 = USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN | USB_ENDPT_EPHSHK;

      SetupPacket setup_packet;
      memcpy(&setup_packet, bdt_entry->address, sizeof(setup_packet));

      // Give the buffer back now.
      dma_memory_barrier();
      // Next IN and OUT packet for this endpoint (data stage/status stage)
      // should both be DATA1.
      MutableBdtEntryFromStat(stat ^ M_USB_STAT_ODD)->buffer_descriptor =
          M_USB_BD_OWN | M_USB_BD_DTS | V_USB_BD_BC(kEndpoint0MaxSize) |
          M_USB_BD_DATA1;
      endpoint0_tx_toggle_ = Data01::kData1;

      // Give this buffer back. It should be DATA0 because it'll be the second
      // received packet.
      bdt_entry->buffer_descriptor =
          M_USB_BD_OWN | M_USB_BD_DTS | V_USB_BD_BC(kEndpoint0MaxSize);

      // TODO(Brian): Tell the functions a new setup packet is starting.
      // CdcTty: next_endpoint0_out_ = NextEndpoint0Out::kNone;

      // Forget about any pending transactions on this endpoint. There shouldn't
      // be any, so if we think there are something's out of sync and we should
      // just drop it. Important to do this before clearing TXD_SUSPEND in
      // USBx_CTL. Standard says "If a Setup transaction is received by an
      // endpoint before a previously initiated control transfer is completed,
      // the device must abort the current transfer/operation".
      endpoint0_data_ = nullptr;
      endpoint0_data_left_ = 0;
      MutableBdtEntry(0, Direction::kTx, EvenOdd::kEven)->buffer_descriptor = 0;
      MutableBdtEntry(0, Direction::kTx, EvenOdd::kOdd)->buffer_descriptor = 0;

      HandleEndpoint0SetupPacket(setup_packet);

      break;

    case UsbPid::kOut:
     for (UsbFunction *function : functions_) {
       switch (function->HandleEndpoint0OutPacket(
           bdt_entry->address, G_USB_BD_BC(bdt_entry->buffer_descriptor))) {
          case SetupResponse::kIgnored:
            break;
          case SetupResponse::kHandled:
            dma_memory_barrier();
            bdt_entry->buffer_descriptor =
                M_USB_BD_OWN | M_USB_BD_DTS | V_USB_BD_BC(kEndpoint0MaxSize);
            return;
          case SetupResponse::kStall:
            bdt_entry->buffer_descriptor =
                M_USB_BD_OWN | M_USB_BD_DTS | V_USB_BD_BC(kEndpoint0MaxSize);
            StallEndpoint0();
            return;
        }
      }
      bdt_entry->buffer_descriptor =
          M_USB_BD_OWN | M_USB_BD_DTS | V_USB_BD_BC(kEndpoint0MaxSize);
      StallEndpoint0();
      return;

    case UsbPid::kIn:
      // The functions are allowed to queue data in {endpoint0_data_,
      // endpoint0_data_left_}, so this case deals with sending their data too.

      // An IN transaction completed, so set up for the next one if appropriate.
      if (!BufferEndpoint0TxPacket()) {
        // After we're done, any further requests from the host should result in
        // stalls (until the next setup token).
        // TODO(Brian): Keep track of which direction it is and how much we've
        // finished so we actually know when to stall it, both here and for
        // kOut tokens.
        //StallEndpoint0();
      }

      // If we have a new address, there is nothing left in the setup request
      // besides a single IN packet forming the status stage, so we know the
      // changes must be done now.
      if (new_address_ != 0) {
        USB0_ADDR = new_address_;
        new_address_ = 0;
      }

      break;

    default:
      // Should never happen, but give the buffer back anyways if necessary.
      if (!(bdt_entry->buffer_descriptor & M_USB_BD_OWN)) {
        bdt_entry->buffer_descriptor =
            M_USB_BD_OWN | M_USB_BD_DTS | V_USB_BD_BC(kEndpoint0MaxSize);
      }
      break;
  }

  // Clear the TXD_SUSPEND flag.
  dma_memory_barrier();
  USB0_CTL = USB_CTL_USBENSOFEN;
}

void UsbDevice::HandleEndpoint0SetupPacket(const SetupPacket &setup_packet) {
  const bool in = setup_packet.request_type & M_SETUP_REQUEST_TYPE_IN;
  const uint8_t recipient =
      G_SETUP_REQUEST_TYPE_RECIPIENT(setup_packet.request_type);
  switch (G_SETUP_REQUEST_TYPE_TYPE(setup_packet.request_type)) {
    case SetupRequestType::kStandard:
      switch (setup_packet.request) {
        case standard_setup_requests::kSetAddress:
          if (in || recipient != standard_setup_recipients::kDevice ||
              setup_packet.index != 0 || setup_packet.length != 0) {
            break;
          }
          new_address_ = setup_packet.value;
          SendEmptyEndpoint0Packet();
          return;

        case standard_setup_requests::kSetConfiguration:
          if (in || recipient != standard_setup_recipients::kDevice ||
              setup_packet.index != 0 || setup_packet.length != 0) {
            break;
          }
          configuration_ = setup_packet.value;

          // No need to mess with endpoint0_tx_toggle_ because we reset it with
          // each setup packet anyways.

          for (int endpoint = 0;
               endpoint < static_cast<int>(endpoint_mapping_.size());
               ++endpoint) {
            if (endpoint_mapping_[endpoint]) {
              endpoint_mapping_[endpoint]->HandleConfigured(endpoint);
            }
          }

          SendEmptyEndpoint0Packet();
          return;

        case standard_setup_requests::kClearFeature:
          if (in || setup_packet.length != 0) {
            break;
          }
          if (recipient == standard_setup_recipients::kEndpoint &&
              setup_packet.value == standard_feature_selectors::kEndpointHalt) {
            const int endpoint =
                G_SETUP_REQUEST_INDEX_ENDPOINT(setup_packet.index);
            // Our endpoint 0 doesn't support the halt feature because that's
            // weird and not recommended by the standard.
            if (endpoint == 0) {
              break;
            }
            if (endpoint >= number_endpoints()) {
              break;
            }
            USB0_ENDPTn(endpoint) &= ~USB_ENDPT_EPSTALL;
            if (endpoint_mapping_[endpoint] != nullptr) {
              endpoint_mapping_[endpoint]->HandleConfigured(endpoint);
            }
            SendEmptyEndpoint0Packet();
            return;
          }
          // We should never get kDeviceRemoteWakeup because we don't advertise
          // support for it in our configuration descriptors.
          // We should never get kTestMode because we're not high-speed.
          break;

        case standard_setup_requests::kSetFeature:
          if (in || setup_packet.length != 0) {
            break;
          }
          if (recipient == standard_setup_recipients::kEndpoint &&
              setup_packet.value == standard_feature_selectors::kEndpointHalt) {
            const int endpoint =
                G_SETUP_REQUEST_INDEX_ENDPOINT(setup_packet.index);
            // Our endpoint 0 doesn't support the halt feature because that's
            // weird and not recommended by the standard.
            if (endpoint == 0) {
              break;
            }
            if (endpoint >= number_endpoints()) {
              break;
            }
            StallEndpoint(endpoint);
            // TODO(Brian): Tell the appropriate function it's now stalled.
            SendEmptyEndpoint0Packet();
            return;
          }
          // We should never get kDeviceRemoteWakeup because we don't advertise
          // support for it in our configuration descriptors.
          // We should never get kTestMode because we're not high-speed.
          break;

        case standard_setup_requests::kGetConfiguration:
          if (!in || recipient != standard_setup_recipients::kDevice ||
              setup_packet.index != 0 || setup_packet.length != 1) {
            break;
          }
          endpoint0_transmit_buffer_[0] = configuration_;
          QueueEndpoint0Data(endpoint0_transmit_buffer_, 1);
          return;

        case standard_setup_requests::kGetInterface:
          if (!in || recipient != standard_setup_recipients::kInterface ||
              setup_packet.value != 0 || setup_packet.length != 1) {
            break;
          }
          // Standard says it's unspecified in the default state and must
          // respond with an error in the address state, so just do an error for
          // both of them.
          if (configuration_ == 0) {
            break;
          }
          // TODO(Brian): Ask the appropriate function what alternate setting
          // the interface has, and stall if there isn't one.
          endpoint0_transmit_buffer_[0] = 0;
          QueueEndpoint0Data(endpoint0_transmit_buffer_, 1);
          return;

        case standard_setup_requests::kSetInterface:
          if (in || recipient != standard_setup_recipients::kInterface ||
              setup_packet.length != 0) {
            break;
          }
          // Standard says it's unspecified in the default state and must
          // respond with an error in the address state, so just do an error for
          // both of them.
          if (configuration_ == 0) {
            break;
          }

          // TODO(Brian): Pass to the appropriate function instead.
          if (setup_packet.value != 0) {
            break;
          }
          SendEmptyEndpoint0Packet();
          return;

        case standard_setup_requests::kGetStatus:
          if (!in || setup_packet.value != 0 || setup_packet.length != 2) {
            break;
          }
          if (recipient == standard_setup_recipients::kDevice) {
            if (setup_packet.index != 0) {
              break;
            }
            // Say that we're currently self powered.
            endpoint0_transmit_buffer_[0] = 1;
            endpoint0_transmit_buffer_[1] = 0;
            QueueEndpoint0Data(endpoint0_transmit_buffer_, 2);
            return;
          }
          if ((recipient == standard_setup_recipients::kInterface &&
               setup_packet.index == 0) ||
              (recipient == standard_setup_recipients::kEndpoint &&
               G_SETUP_REQUEST_INDEX_ENDPOINT(setup_packet.index) == 0)) {
            endpoint0_transmit_buffer_[0] = 0;
            endpoint0_transmit_buffer_[1] = 0;
            QueueEndpoint0Data(endpoint0_transmit_buffer_, 2);
            return;
          }
          // Standard says it's unspecified in the default state and must
          // respond with an error in the address state, so just do an error
          // for both of them.
          if (configuration_ == 0) {
            break;
          }

          if (recipient == standard_setup_recipients::kInterface) {
            // TODO(Brian): Check if it's actually an interface we have?
            endpoint0_transmit_buffer_[0] = 0;
            endpoint0_transmit_buffer_[1] = 0;
            QueueEndpoint0Data(endpoint0_transmit_buffer_, 2);
            return;
          }

          if (recipient == standard_setup_recipients::kEndpoint) {
            const int endpoint =
                G_SETUP_REQUEST_INDEX_ENDPOINT(setup_packet.index);
            // TODO(Brian): Check if it's actually an endpoint we have?
            if (USB0_ENDPTn(endpoint) & USB_ENDPT_EPSTALL) {
              endpoint0_transmit_buffer_[0] = 1;
            } else {
              endpoint0_transmit_buffer_[0] = 0;
            }
            endpoint0_transmit_buffer_[1] = 0;
            QueueEndpoint0Data(endpoint0_transmit_buffer_, 2);
            return;
          }
          break;

        case standard_setup_requests::kSetDescriptor:
          // Not implementing anything for this.
          break;

        case standard_setup_requests::kSynchFrame:
          // We don't implement any classes which use this.
          break;

        case standard_setup_requests::kGetDescriptor:
          if (!in) {
            break;
          }

          const uint8_t descriptor_type_byte = (setup_packet.value >> 8) & 0xFF;
          if (G_DESCRIPTOR_TYPE_TYPE(descriptor_type_byte) !=
              standard_descriptor_type_types::kStandard) {
            for (UsbFunction *function : functions_) {
              switch (function->HandleGetDescriptor(setup_packet)) {
                case SetupResponse::kIgnored:
                  continue;
                case SetupResponse::kHandled:
                  return;
                case SetupResponse::kStall:
                  break;
              }
              break;
            }
            break;
          }

          if (recipient != standard_setup_recipients::kDevice) {
            break;
          }
          if (descriptor_type_byte < kUsbDescriptorTypeMin ||
              descriptor_type_byte > kUsbDescriptorTypeMax) {
            break;
          }
          const UsbDescriptorType descriptor_type =
              static_cast<UsbDescriptorType>(descriptor_type_byte);
          const uint8_t descriptor_index = setup_packet.value & 0xFF;
          switch (descriptor_type) {
            case UsbDescriptorType::kDevice:
              if (setup_packet.index != 0 || descriptor_index != 0) {
                break;
              }
              QueueEndpoint0Data(
                  device_descriptor_list_.data_.data(),
                  ::std::min<int>(setup_packet.length,
                                  device_descriptor_list_.data_.size()));
              return;

            case UsbDescriptorType::kConfiguration:
              if (setup_packet.index != 0 || descriptor_index != 0) {
                break;
              }
              QueueEndpoint0Data(
                  config_descriptor_list_.data_.data(),
                  ::std::min<int>(setup_packet.length,
                                  config_descriptor_list_.data_.size()));
              return;

            case UsbDescriptorType::kString:
              // Skip any other checks on the other fields. Who knows what
              // Microsoft is going to set them to; not like they document it
              // anywhere obvious...
              if (descriptor_index == 0xEE && setup_packet.index == 0) {
                static uint8_t
                    kMicrosoftOsStringDescriptor
                        [] = {
                            0x12,  // bLength
                            static_cast<uint8_t>(
                                UsbDescriptorType::kString),  // bDescriptorType
                            0x4D,
                            0x00, 0x53, 0x00, 0x46, 0x00, 0x54, 0x00, 0x31,
                            0x00, 0x30, 0x00, 0x30,
                            0x00,                     // qwSignature
                            microsoft_vendor_code(),  // bMS_VendorCode
                            0x00                      // bPad
                        };
                QueueEndpoint0Data(
                    reinterpret_cast<char *>(kMicrosoftOsStringDescriptor),
                    ::std::min<int>(setup_packet.length,
                                    sizeof(kMicrosoftOsStringDescriptor)));
                return;
              }
              if (descriptor_index != 0 &&
                  setup_packet.index != english_us_code()) {
                break;
              }
              if (descriptor_index >= strings_.size()) {
                break;
              }
              QueueEndpoint0Data(
                  strings_[descriptor_index].data(),
                  ::std::min<int>(setup_packet.length,
                                  strings_[descriptor_index].size()));
              return;

            default:
              // TODO(Brian): Handle other types of descriptor too.
              break;
          }
      }
      break;

    case SetupRequestType::kVendor:
      switch (setup_packet.request) {
        case microsoft_vendor_code():
          if (!in) {
            break;
          }

          switch (recipient) {
            case standard_setup_recipients::kDevice:
              if (setup_packet.value != 0) {
                // Ignoring weird things and descriptors larger than 64K for
                // now.
                break;
              }
              switch (setup_packet.index) {
                case microsoft_feature_descriptors::kExtendedCompatibilityId:
                  QueueEndpoint0Data(
                      microsoft_extended_id_descriptor_.data(),
                      ::std::min<int>(
                          setup_packet.length,
                          microsoft_extended_id_descriptor_.size()));
                  return;
              }
              break;

            case standard_setup_recipients::kInterface:
              switch (setup_packet.index) {
                case microsoft_feature_descriptors::kExtendedProperties:
                  const int interface = setup_packet.value & 0xFF;
                  const int page_number = (setup_packet.value >> 8) & 0xFF;

                  if (page_number != 0) {
                    // Ignoring weird things and descriptors larger than 64K for
                    // now.
                    break;
                  }

                  const UsbFunction *const function =
                      interface_mapping_[interface];
                  const ::std::string &descriptor =
                      function->microsoft_extended_property_descriptor_;
                  if (descriptor.empty() ||
                      interface != function->first_interface_) {
                    break;
                  }
                  QueueEndpoint0Data(
                      descriptor.data(),
                      ::std::min<int>(setup_packet.length, descriptor.size()));
                  return;
              }
              break;
          }
          break;
      }
      break;

    default:
      for (UsbFunction *function : functions_) {
        switch (function->HandleEndpoint0SetupPacket(setup_packet)) {
          case SetupResponse::kIgnored:
            continue;
          case SetupResponse::kHandled:
            return;
          case SetupResponse::kStall:
            break;
        }
        break;
      }
      break;
  }

  StallEndpoint0();
}

// We're supposed to continue returning stalls until the next kSetup packet.
// Code might continue putting stuff in the TX buffers, but the hardware won't
// actually send it as long as the EPSTALL bit is set.
void UsbDevice::StallEndpoint0() {
  USB0_ENDPT0 = USB_ENDPT_EPSTALL | USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN |
                USB_ENDPT_EPHSHK;
}

bool UsbDevice::BufferEndpoint0TxPacket() {
  if (endpoint0_data_ == nullptr) {
    return false;
  }

  const int to_transmit = ::std::min(endpoint0_data_left_, kEndpoint0MaxSize);
  BdtEntry *const tx_bdt_entry =
      MutableBdtEntry(0, Direction::kTx, endpoint0_tx_odd_);
  // const_cast is safe because the hardware is only going to read from
  // this, not write.
  tx_bdt_entry->address =
      const_cast<void *>(static_cast<const void *>(endpoint0_data_));
  dma_memory_barrier();
  tx_bdt_entry->buffer_descriptor =
      V_USB_BD_BC(to_transmit) | static_cast<uint32_t>(endpoint0_tx_toggle_) |
      M_USB_BD_OWN | M_USB_BD_DTS;

  endpoint0_tx_odd_ = EvenOddInverse(endpoint0_tx_odd_);
  endpoint0_tx_toggle_ = Data01Inverse(endpoint0_tx_toggle_);

  endpoint0_data_ += to_transmit;
  endpoint0_data_left_ -= to_transmit;
  if (to_transmit < kEndpoint0MaxSize) {
    endpoint0_data_ = nullptr;
  }

  return true;
}

void UsbDevice::SendEmptyEndpoint0Packet() {
  // Really doesn't matter what we put here as long as it's not nullptr.
  endpoint0_data_ = reinterpret_cast<char *>(this);
  endpoint0_data_left_ = 0;
  BufferEndpoint0TxPacket();
}

void UsbDevice::QueueEndpoint0Data(const char *data, int size) {
  endpoint0_data_ = data;
  endpoint0_data_left_ = size;
  // There are 2 TX buffers, so fill them both up.
  BufferEndpoint0TxPacket();
  BufferEndpoint0TxPacket();
}

void UsbDevice::StallEndpoint(int endpoint) {
  for (Direction direction : {Direction::kTx, Direction::kRx}) {
    for (EvenOdd odd : {EvenOdd::kOdd, EvenOdd::kEven}) {
      MutableBdtEntry(endpoint, direction, odd)->buffer_descriptor = 0;
      dma_memory_barrier();
      MutableBdtEntry(endpoint, direction, odd)->address = nullptr;
    }
  }
  USB0_ENDPTn(endpoint) |= USB_ENDPT_EPSTALL;
}

void UsbDevice::ConfigureEndpointFor(int endpoint, bool rx, bool tx,
                                     bool handshake) {
  uint8_t control = 0;
  if (rx) {
    control |= USB_ENDPT_EPRXEN;
  }
  if (tx) {
    control |= USB_ENDPT_EPTXEN;
  }
  if (handshake) {
    control |= USB_ENDPT_EPHSHK;
  }
  USB0_ENDPTn(endpoint) = control;
}

int UsbFunction::AddEndpoint() {
  const int r = device_->endpoint_mapping_.size();
  assert(r < number_endpoints());
  device_->endpoint_mapping_.push_back(this);
  return r;
}

int UsbFunction::AddInterface() {
  const int r = device_->interface_mapping_.size();
  // bInterfaceNumber is only one byte.
  assert(r < 255);
  device_->interface_mapping_.push_back(this);
  return r;
}

void UsbFunction::CreateIadDescriptor(int first_interface, int interface_count,
                                      int function_class, int function_subclass,
                                      int function_protocol,
                                      const ::std::string &function) {
  first_interface_ = first_interface;
  const auto iad_descriptor = CreateDescriptor(
      iad_descriptor_length(), UsbDescriptorType::kInterfaceAssociation);
  iad_descriptor->AddByte(first_interface);                // bFirstInterface
  iad_descriptor->AddByte(interface_count);                // bInterfaceCount
  iad_descriptor->AddByte(function_class);                 // bFunctionClass
  iad_descriptor->AddByte(function_subclass);              // bFunctionSubClass
  iad_descriptor->AddByte(function_protocol);              // bFunctionProtocol
  iad_descriptor->AddByte(device()->AddString(function));  // iFunction
}

void UsbFunction::SetMicrosoftDeviceInterfaceGuids(const ::std::string &guids) {
  ::std::map<::std::string, ::std::string> properties;
  properties["DeviceInterfaceGUIDs"] = guids + ::std::string(1, '\0');
  ::std::string *const descriptor = &microsoft_extended_property_descriptor_;

  uint32_t length = 10;
  for (auto &pair : properties) {
    length += 14;
    length += (pair.first.size() + 1) * 2;
    length += (pair.second.size() + 1) * 2;
  }
  descriptor->resize(length);
  int index = 0;

  // dwLength
  (*descriptor)[index++] = length & 0xFF;
  (*descriptor)[index++] = (length >> UINT32_C(8)) & 0xFF;
  (*descriptor)[index++] = (length >> UINT32_C(16)) & 0xFF;
  (*descriptor)[index++] = (length >> UINT32_C(24)) & 0xFF;

  // bcdVersion
  (*descriptor)[index++] = 0x00;
  (*descriptor)[index++] = 0x01;

  // wIndex
  (*descriptor)[index++] = microsoft_feature_descriptors::kExtendedProperties;
  (*descriptor)[index++] = 0;

  // wCount
  (*descriptor)[index++] = properties.size() & 0xFF;
  (*descriptor)[index++] = (properties.size() >> 8) & 0xFF;

  for (auto &pair : properties) {
    const uint32_t size = 14 + (pair.first.size() + pair.second.size() + 2) * 2;
    // dwSize
    (*descriptor)[index++] = size & 0xFF;
    (*descriptor)[index++] = (size >> UINT32_C(8)) & 0xFF;
    (*descriptor)[index++] = (size >> UINT32_C(16)) & 0xFF;
    (*descriptor)[index++] = (size >> UINT32_C(24)) & 0xFF;

    // Need to get this from the map if we ever do others in the future.
    const uint32_t data_type = 7;  // REG_MULTI_SZ
    // dwPropertyDataType
    (*descriptor)[index++] = data_type & 0xFF;
    (*descriptor)[index++] = (data_type >> UINT32_C(8)) & 0xFF;
    (*descriptor)[index++] = (data_type >> UINT32_C(16)) & 0xFF;
    (*descriptor)[index++] = (data_type >> UINT32_C(24)) & 0xFF;

    // wPropertyNameLength
    (*descriptor)[index++] = ((pair.first.size() + 1) * 2) & 0xFF;
    (*descriptor)[index++] = (((pair.first.size() + 1) * 2) >> 8) & 0xFF;

    // bPropertyName
    for (size_t i = 0; i < pair.first.size(); ++i) {
      (*descriptor)[index] = pair.first[i];
      index += 2;
    }
    index += 2;

    // wPropertyDataLength
    (*descriptor)[index++] = ((pair.second.size() + 1) * 2) & 0xFF;
    (*descriptor)[index++] =
        (((pair.second.size() + 1) * 2) >> UINT8_C(8)) & 0xFF;
    (*descriptor)[index++] =
        (((pair.second.size() + 1) * 2) >> UINT8_C(16)) & 0xFF;
    (*descriptor)[index++] =
        (((pair.second.size() + 1) * 2) >> UINT8_C(24)) & 0xFF;

    // bPropertyData
    for (size_t i = 0; i < pair.second.size(); ++i) {
      (*descriptor)[index] = pair.second[i];
      index += 2;
    }
    index += 2;
  }

  assert(index == length);
}

void UsbDevice::SetBdtEntry(int endpoint, Direction direction, EvenOdd odd,
                            BdtEntry bdt_entry) {
  *MutableBdtEntry(endpoint, direction, odd) = bdt_entry;
}

}  // namespace teensy
}  // namespace frc971
