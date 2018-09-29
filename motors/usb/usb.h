#ifndef MOTORS_USB_USB_H_
#define MOTORS_USB_USB_H_

#include <assert.h>
#include <string.h>
#include <string>
#include <vector>
#include <memory>

#include "aos/macros.h"
#include "motors/core/kinetis.h"
#include "motors/usb/constants.h"
#include "motors/util.h"

namespace frc971 {
namespace teensy {

// A sufficient memory barrier between writing some data and telling the USB
// hardware to read it or having the USB hardware say some data is readable and
// actually reading it.
static inline void dma_memory_barrier() { DmaMemoryBarrier(); }

// Aligned for faster access via memcpy etc.
//
// Also, the Freescale example stack forces aligned buffers to work around some
// hardware limitations which may or may not apply to our chips.
typedef void *DataPointer __attribute__((aligned(4)));

// An entry in the Buffer Descriptor Table.
struct BdtEntry {
  uint32_t buffer_descriptor;
  DataPointer address;
};

#define V_USB_BD_BC(value) \
  static_cast<uint32_t>(static_cast<uint32_t>(value) << 16)
#define G_USB_BD_BC(bd) (((bd) >> 16) & UINT32_C(0x3FF))
#define M_USB_BD_OWN UINT32_C(1 << 7)
#define M_USB_BD_DATA1 UINT32_C(1 << 6)
static_assert(static_cast<uint32_t>(Data01::kData1) == M_USB_BD_DATA1,
              "Wrong value");
#define M_USB_BD_KEEP UINT32_C(1 << 5)
#define M_USB_BD_NINC UINT32_C(1 << 4)
#define M_USB_BD_DTS UINT32_C(1 << 3)
#define M_USB_BD_STALL UINT32_C(1 << 2)
#define V_USB_BD_PID(value) \
  static_cast<uint32_t>(static_cast<uint32_t>(value) << 2)
#define G_USB_BD_PID(bd) static_cast<UsbPid>(((bd) >> 2) & UINT32_C(0xF))

#define G_USB_STAT_ENDP(stat) (((stat) >> 4) & UINT32_C(0xF))
#define M_USB_STAT_TX UINT32_C(1 << 3)
#define M_USB_STAT_ODD UINT32_C(1 << 2)

// The various types of descriptors defined in the standard for retrieval via
// GetDescriptor.
static constexpr uint8_t kUsbDescriptorTypeMin = 1;
static constexpr uint8_t kUsbDescriptorTypeMax = 11;
enum class UsbDescriptorType : uint8_t {
  kDevice = 1,
  kConfiguration = 2,
  kString = 3,
  kInterface = 4,
  kEndpoint = 5,
  kDeviceQualifier = 6,
  kOtherSpeedConfiguration = 7,
  kInterfacePower = 8,
  kOtg = 9,
  kDebug = 10,
  kInterfaceAssociation = 11,
};

// The class-specific descriptor types.
enum class UsbClassDescriptorType : uint8_t {
  kDevice = 0x21,
  kConfiguration = 0x22,
  kString = 0x23,
  kInterface = 0x24,
  kEndpoint = 0x25,

  kHidHid = 0x21,
  kHidReport = 0x22,
  kHidPhysical = 0x23,
};

// The names of the setup request types from the standard.
enum class SetupRequestType {
  kStandard = 0,
  kClass = 1,
  kVendor = 2,
  kReserved = 3,
};

// Set means device-to-host, clear means host-to-device.
#define M_SETUP_REQUEST_TYPE_IN UINT8_C(1 << 7)
#define G_SETUP_REQUEST_TYPE_TYPE(type) \
  static_cast<SetupRequestType>(((type) >> 5) & UINT8_C(3))
#define G_SETUP_REQUEST_TYPE_RECIPIENT(type) ((type)&UINT8_C(0x1F))
#define G_SETUP_REQUEST_INDEX_ENDPOINT(index) ((index)&UINT8_C(0x7F))

// The names of the standard recipients for setup requests.
namespace standard_setup_recipients {
constexpr int kDevice = 0;
constexpr int kInterface = 1;
constexpr int kEndpoint = 2;
constexpr int kOther = 3;
}  // namespace standard_setup_recipients

namespace microsoft_feature_descriptors {
constexpr int kExtendedCompatibilityId = 4;
constexpr int kExtendedProperties = 5;
}  // namespace microsoft_feature_descriptors

// The HID class specification says this. Can't find any mention in the main
// standard.
#define G_DESCRIPTOR_TYPE_TYPE(descriptor_type) \
  ((descriptor_type) >> 5 & UINT8_C(3))
namespace standard_descriptor_type_types {
constexpr int kStandard = 0;
constexpr int kClass = 1;
constexpr int kVendor = 2;
}  // namespace standard_descriptor_type_types

constexpr uint8_t vendor_specific_class() { return 0xFF; }

class UsbFunction;

// Allows building up a list of descriptors. This supports a much nicer API than
// the usual "hard-code a char[] with all the sizes and offsets at compile
// time". Space for each descriptor is reserved, and then it may be filled out
// from beginning to end at any time.
//
// An instance is the thing that the GetDescriptor operation sends to the host.
// This is not the concept that the core and class standards call "Foo
// Descriptor" etc; see Descriptor for that.
class UsbDescriptorList {
 public:
  // Represents a single descriptor. All of the contents must be written before
  // this object is destroyed.
  //
  // Create one via UsbDescriptorList::CreateDescriptor.
  class Descriptor {
   public:
    // All of the allocated space must be filled first.
    ~Descriptor() {
      if (descriptor_list_ == nullptr) {
        return;
      }
      // Verify we wrote all the bytes first.
      assert(next_index_ == end_index_);
      --descriptor_list_->open_descriptors_;
    }

    void AddUint16(uint16_t value) {
      AddByte(value & 0xFF);
      AddByte((value >> 8) & 0xFF);
    }

    void AddByte(uint8_t value) {
      assert(next_index_ < end_index_);
      data()[next_index_] = value;
      ++next_index_;
    }

    // Overwrites an already-written byte.
    void SetByte(int index, uint8_t value) {
      assert(index + start_index_ < end_index_);
      data()[index + start_index_] = value;
    }

   private:
    Descriptor(UsbDescriptorList *descriptor_list, int start_index,
               int end_index)
        : descriptor_list_(descriptor_list),
          start_index_(start_index),
          end_index_(end_index),
          next_index_(start_index_) {}

    char *data() const {
      return &descriptor_list_->data_[0];
    }

    UsbDescriptorList *const descriptor_list_;
    const int start_index_, end_index_;
    int next_index_;

    friend class UsbDescriptorList;

    DISALLOW_COPY_AND_ASSIGN(Descriptor);
  };

  UsbDescriptorList() = default;
  ~UsbDescriptorList() = default;

  // Creates a new descriptor at the end of the list.
  // length is the number of bytes, including the length byte.
  // descriptor_type is the descriptor type, which is the second byte after the
  // length.
  ::std::unique_ptr<Descriptor> CreateDescriptor(
      uint8_t length, UsbDescriptorType descriptor_type) {
    return CreateDescriptor(length, static_cast<uint8_t>(descriptor_type));
  }

  ::std::unique_ptr<Descriptor> CreateDescriptor(
      uint8_t length, UsbClassDescriptorType descriptor_type) {
    assert(data_.size() > 0);
    return CreateDescriptor(length, static_cast<uint8_t>(descriptor_type));
  }

  void AddPremadeDescriptor(const uint8_t *data, int length) {
    const int start_index = data_.size();
    const int end_index = start_index + length;
    data_.resize(end_index);
    memcpy(&data_[start_index], data, length);
  }

  void AddPremadeDescriptor(const UsbDescriptorList &other_list) {
    other_list.CheckFinished();
    AddPremadeDescriptor(
        reinterpret_cast<const uint8_t *>(other_list.data_.data()),
        other_list.data_.size());
  }

  void CheckFinished() const { assert(open_descriptors_ == 0); }

  int CurrentSize() const { return data_.size(); }

  const char *GetData() const {
    CheckFinished();
    return data_.data();
  }

 private:
  ::std::unique_ptr<Descriptor> CreateDescriptor(uint8_t length,
                                                 uint8_t descriptor_type) {
    const int start_index = data_.size();
    const int end_index = start_index + length;
    data_.resize(end_index);
    ++open_descriptors_;
    auto r = ::std::unique_ptr<Descriptor>(
        new Descriptor(this, start_index, end_index));
    r->AddByte(length);           // bLength
    r->AddByte(descriptor_type);  // bDescriptorType
    return r;
  }

  int open_descriptors_ = 0;

  ::std::string data_;

  friend class UsbDevice;

  DISALLOW_COPY_AND_ASSIGN(UsbDescriptorList);
};

extern "C" void usb_isr(void);

// USB state events are managed by asking each function if it wants to handle
// them, sequentially. For the small number of functions which can be
// practically supported with the limited number of endpoints, this performs
// better than fancier things like hash maps.

// Manages one of the Teensy's USB peripherals as a USB slave device.
//
// This supports being a composite device with multiple functions.
//
// Attaching functions etc is called "setup", and must be completed before
// Initialize() is called.
//
// Detaching functions is called "teardown" and must happen after Shutdown().
// TODO(Brian): Implement Shutdown().
class UsbDevice final {
 public:
  // Represents the data that comes with a UsbPid::kSetup.
  // Note that the order etc is important because we memcpy into this.
  struct SetupPacket {
    uint8_t request_type;  // bmRequestType
    uint8_t request;       // bRequest
    uint16_t value;        // wValue
    uint16_t index;        // wIndex
    uint16_t length;       // wLength
  } __attribute__((aligned(4)));
  static_assert(sizeof(SetupPacket) == 8, "wrong size");

  enum class SetupResponse {
    // Indicates this function doesn't recognize the setup packet.
    kIgnored,

    // Indicates the endpoint should be stalled.
    //
    // Don't return this if the packet is for another function.
    kStall,

    // Indicates this setup packet was handled. Functions must avoid eating
    // packets intended for other functions.
    kHandled,
  };

  static constexpr int kEndpoint0MaxSize = 64;

  // The only language code we support.
  static constexpr uint16_t english_us_code() { return 0x0409; }

  UsbDevice(int index, uint16_t vendor_id, uint16_t product_id);
  ~UsbDevice();

  // Ends setup and starts being an actual USB device.
  void Initialize();

  // Adds a string to the table and returns its index.
  //
  // For simplicity, we only support strings with english_us_code().
  //
  // May only be called during setup.
  int AddString(const ::std::string &string) {
    assert(!is_set_up_);
    const int r = strings_.size();
    strings_.emplace_back(string.size() * 2 + 2, '\0');
    strings_.back()[0] = 2 + string.size() * 2;
    strings_.back()[1] = static_cast<uint8_t>(UsbDescriptorType::kString);
    for (size_t i = 0; i < string.size(); ++i) {
      strings_.back()[i * 2 + 2] = string[i];
    }
    return r;
  }

  // Sets the manufacturer string.
  //
  // May only be called during setup.
  void SetManufacturer(const ::std::string &string) {
    device_descriptor_->SetByte(14, AddString(string));  // iManufacturer
  }

  // Sets the product string.
  //
  // May only be called during setup.
  void SetProduct(const ::std::string &string) {
    device_descriptor_->SetByte(15, AddString(string));  // iProduct
  }

  // Sets the serial number string.
  //
  // May only be called during setup.
  void SetSerialNumber(const ::std::string &string) {
    device_descriptor_->SetByte(16, AddString(string));  // iSerialNumber
  }

  // Queues up an empty IN packet for endpoint 0. This is a common way to
  // respond to various kinds of configuration commands.
  //
  // This may only be called from the appropriate function callbacks.
  void SendEmptyEndpoint0Packet();

  // Queues some data to send on endpoint 0. This includes putting the initial
  // packets into the TX buffers.
  //
  // This may only be called from the appropriate function callbacks.
  void QueueEndpoint0Data(const char *data, int size);

  // Stalls an endpoint until it's cleared.
  //
  // This should only be called by or on behalf of the function which owns
  // endpoint.
  void StallEndpoint(int endpoint);

  // Configures an endpoint to send and/or receive, with or without DATA0/DATA1
  // handshaking. handshake should probably be true for everything except
  // isochronous endpoints.
  //
  // This should only be called by or on behalf of the function which owns
  // endpoint.
  void ConfigureEndpointFor(int endpoint, bool rx, bool tx, bool handshake);

  void SetBdtEntry(int endpoint, Direction direction, EvenOdd odd,
                   BdtEntry bdt_entry);

 private:
  // Clears all pending interrupts.
  void ClearInterrupts();

  // Deals with an interrupt that has occured.
  void HandleInterrupt();

  // Processes a token on endpoint 0.
  void HandleEndpoint0Token(uint8_t stat);

  // Processes a setup packet on endpoint 0.
  void HandleEndpoint0SetupPacket(const SetupPacket &setup_packet);

  // Sets endpoint 0 to return STALL tokens. We clear this condition upon
  // receiving the next SETUP token.
  void StallEndpoint0();

  // Places the first packet from {endpoint0_data_, endpoint0_data_left_} into
  // the TX buffers (if there is any data). This may only be called when the
  // next TX buffer is empty.
  bool BufferEndpoint0TxPacket();

  // Which USB peripheral this is.
  const int index_;

  // The string descriptors in order.
  ::std::vector<::std::string> strings_;

  // TODO(Brian): Refactor into something more generic, because I think this is
  // shared with all non-isochronous endpoints?
  Data01 endpoint0_tx_toggle_;
  EvenOdd endpoint0_tx_odd_;
  uint8_t endpoint0_receive_buffer_[2][kEndpoint0MaxSize]
      __attribute__((aligned(4)));

  // A temporary buffer for holding data to transmit on endpoint 0. Sometimes
  // this is used and sometimes the data is sent directly from some other
  // location (like for descriptors).
  char endpoint0_transmit_buffer_[kEndpoint0MaxSize];

  // The data we're waiting to send from endpoint 0. The data must remain
  // constant until this transmission is done.
  //
  // When overwriting this, we ignore if it's already non-nullptr. The host is
  // supposed to read all of the data before asking for more. If it doesn't do
  // that, it will just get garbage data because it's unclear what it expects.
  //
  // Do note that endpoint0_data_ != nullptr && endpoint0_data_left_ == 0 is an
  // important state. This means we're going to return a 0-length packet the
  // next time the host asks. However, depending on the length it asked for,
  // that might never happen.
  const char *endpoint0_data_ = nullptr;
  int endpoint0_data_left_ = 0;

  // If non-0, the new address we're going to start using once the status stage
  // of the current setup request is finished.
  uint16_t new_address_ = 0;

  UsbDescriptorList device_descriptor_list_;
  UsbDescriptorList config_descriptor_list_;

  ::std::unique_ptr<UsbDescriptorList::Descriptor> device_descriptor_,
      config_descriptor_;

  int configuration_ = 0;

  bool is_set_up_ = false;

  // The function which owns each endpoint.
  ::std::vector<UsbFunction *> endpoint_mapping_;
  // The function which owns each interface.
  ::std::vector<UsbFunction *> interface_mapping_;
  // All of the functions (without duplicates).
  ::std::vector<UsbFunction *> functions_;

  // Filled out during Initialize().
  ::std::string microsoft_extended_id_descriptor_;

  friend void usb_isr(void);
  friend class UsbFunction;
};

// Represents a USB function. This consists of a set of descriptors and
// interfaces.
//
// Each instance is a single function, so there can be multiple instances of the
// same subclass in the same devices (ie two serial ports).
class UsbFunction {
 public:
  UsbFunction(UsbDevice *device) : device_(device) {
    device_->functions_.push_back(this);
  }
  virtual ~UsbFunction() = default;

 protected:
  using SetupResponse = UsbDevice::SetupResponse;

  static constexpr uint8_t iad_descriptor_length() { return 8; }
  static constexpr uint8_t interface_descriptor_length() { return 9; }
  static constexpr uint8_t endpoint_descriptor_length() { return 7; }

  static constexpr uint8_t m_endpoint_address_in() { return 1 << 7; }
  static constexpr uint8_t m_endpoint_attributes_control() { return 0x00; }
  static constexpr uint8_t m_endpoint_attributes_isochronous() { return 0x01; }
  static constexpr uint8_t m_endpoint_attributes_bulk() { return 0x02; }
  static constexpr uint8_t m_endpoint_attributes_interrupt() { return 0x03; }

  // Adds a new endpoint and returns its index.
  //
  // Note that at least one descriptor for this newly created endpoint must be
  // added via CreateConfigDescriptor.
  //
  // TODO(Brian): Does this hardware actually only support a single direction
  // per endpoint number, or can it get a total of 30 endpoints max?
  //
  // May only be called during setup.
  int AddEndpoint();

  // Adds a new interface and returns its index.
  //
  // You'll probably want to put this new interface in at least one descriptor
  // added via CreateConfigDescriptor.
  //
  // May only be called during setup.
  int AddInterface();

  // Adds a new descriptor in the configuration descriptor list. See
  // UsbDescriptorList::CreateDescriptor for details.
  //
  // Note that the order of calls to this is highly significant. In general,
  // this should only be called from Initialize().
  //
  // May only be called during setup.
  template <typename T>
  ::std::unique_ptr<UsbDescriptorList::Descriptor> CreateDescriptor(
      uint8_t length, T descriptor_type) {
    return device_->config_descriptor_list_.CreateDescriptor(length,
                                                             descriptor_type);
  }
  void AddPremadeDescriptor(const uint8_t *data, int length) {
    device_->config_descriptor_list_.AddPremadeDescriptor(data, length);
  }
  void AddPremadeDescriptor(const UsbDescriptorList &other_list) {
    device_->config_descriptor_list_.AddPremadeDescriptor(other_list);
  }

  UsbDevice *device() const { return device_; }

  void CreateIadDescriptor(int first_interface, int interface_count,
                           int function_class, int function_subclass,
                           int function_protocol,
                           const ::std::string &function);

  // Sets the interface GUIDs for this function. Each GUID (one per interface?)
  // should be followed by a NUL.
  //
  // If this is never called, no GUID extended property will be reported.
  //
  // This is needed to pass to Windows so WinUSB will be happy. Generate them at
  // https://www.guidgenerator.com/online-guid-generator.aspx (Uppcase, Braces,
  // and Hyphens).
  //
  // May only be called during setup.
  void SetMicrosoftDeviceInterfaceGuids(const ::std::string &guids);

 private:
  virtual void Initialize() = 0;

  virtual SetupResponse HandleEndpoint0SetupPacket(
      const UsbDevice::SetupPacket & /*setup_packet*/) {
    return SetupResponse::kIgnored;
  }

  virtual SetupResponse HandleEndpoint0OutPacket(void * /*data*/,
                                                 int /*data_length*/) {
    return SetupResponse::kIgnored;
  }

  virtual SetupResponse HandleGetDescriptor(
      const UsbDevice::SetupPacket & /*setup_packet*/) {
    return SetupResponse::kIgnored;
  }

  // Returns the concatenated compatible ID and subcompatible ID.
  virtual ::std::string MicrosoftExtendedCompatibleId() {
    // Default to both of them being "unused".
    return ::std::string(16, '\0');
  }

  virtual void HandleOutFinished(int /*endpoint*/, BdtEntry * /*bdt_entry*/) {}
  virtual void HandleInFinished(int /*endpoint*/, BdtEntry * /*bdt_entry*/,
                                EvenOdd /*odd*/) {}

  // Called when a given interface is configured (aka "experiences a
  // configuration event"). This means all rx and tx buffers have been cleared
  // and should be filled as appropriate, starting from data0. Also,
  // ConfigureEndpointFor should be called with the appropriate arguments.
  virtual void HandleConfigured(int endpoint) = 0;

  // Should reset everything to use the even buffers next.
  virtual void HandleReset() = 0;

  int first_interface_ = -1;

  // Filled out during Initialize().
  ::std::string microsoft_extended_property_descriptor_;

  UsbDevice *const device_;

  friend class UsbDevice;
};

}  // namespace teensy
}  // namespace frc971

#endif  // MOTORS_USB_USB_H_
